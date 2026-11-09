#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/dart/manual_dispatch.hpp"

namespace rmcs_core::controller::dart {

// ─────────────────────────────────────────────────────────────────────────────
// CarriageNewController
//   Board Lite 新镖架的弹舱 carriage 控制器（合并原 ManualCarriageControl
//   与 CarriageHoming 两个半成品组件）。
//
//   本控制器只做两件事：
//     ① 手动遥控器映射控制 —— 档位与摇杆由 kManual 说明书声明，
//        经 ManualDispatcher 遍历分发驱动；
//     ② 归零定位 —— 向上撞微动开关，退离后下行到工作位。
//
//   归零由上电时自动触发，是否启动由 yaml 的 enabled 开关控制：
//     enabled: false —— 上电不跑归零，只响应手动摇杆（当前机械未装完）
//     enabled: true  —— 上电自动归零，期间屏蔽摇杆，跑完/失败后交还手动
//
//   状态机：
//     kUp ──撞开关──> kBackOff ──退离到位──> kDownRun ──到位──> kDone
//       │                 │                      │
//       └─超时─> kFailed   └─超时─> kFailed       └─超时─> kFailed
//
//   输出语义：任何状态下都只输出有限速度，绝不用 NaN
//   （竖直滑台 NaN = 电调零电流 = 重力自由下滑）。
// ─────────────────────────────────────────────────────────────────────────────
class CarriageNewController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CarriageNewController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/dart/micro_switch", micro_switch_, false);
        register_input("/dart/carriage_motor/angle", angle_, false);

        register_output("/dart/carriage_motor/control_velocity", control_velocity_, 0.0);

        load_parameters();
        ManualRegistry::add(&kManual, this);
    }

    CarriageNewController(const CarriageNewController&) = delete;
    CarriageNewController& operator=(const CarriageNewController&) = delete;
    CarriageNewController(CarriageNewController&&) = delete;
    CarriageNewController& operator=(CarriageNewController&&) = delete;

    ~CarriageNewController() override = default;

    void before_updating() override {
        if (!micro_switch_.ready()) {
            micro_switch_.make_and_bind_directly(false);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/micro_switch\". Set to false.");
        }
        if (!angle_.ready()) {
            angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/carriage_motor/angle\". Set to 0.");
        }
    }

    // 分发由 ManualDispatcher 统一驱动，本组件 update 不做档位判断。
    // 空实现是 Component 纯虚函数的接口要求。
    void update() override {}

    // ── ManualRegistry 的两个动作 ─────────────────────────────────────────────
    static void on_manual(void* self, const ManualBinding& binding, const StickSnapshot& sticks) {
        static_cast<CarriageNewController*>(self)->update_manual(binding, sticks);
    }

    static void on_auto(void* self) {
        CarriageNewController* me = static_cast<CarriageNewController*>(self);
        me->update_auto();
    }

private:
    // 归零状态机
    void load_parameters() {
        const auto d = [this](const char* n) { return get_parameter(n).as_double(); };

        // 向上对应的速度符号：+1 = 正速度向上，-1 = 负速度向上
        up_sign_ = d("up_sign") >= 0.0 ? 1.0 : -1.0;

        sensitivity_forward_ = d("sensitivity_forward");
        sensitivity_reverse_ = d("sensitivity_reverse");
        deadzone_ = std::abs(d("deadzone"));

        homing_speed_ = d("homing_speed");
        homing_timeout_ms_ = get_parameter("homing_timeout_ms").as_int();

        back_off_speed_ = d("back_off_speed");
        back_off_angle_ = d("back_off_angle");

        down_speed_ = d("down_speed");
        down_angle_ = d("down_angle");

        // 上电是否自动启动归零
        enabled_ = get_parameter("enabled").as_bool();
    }

    // ── ① 手动遥控器映射控制 ──────────────────────────────────────────────────
    void update_manual(const ManualBinding& binding, const StickSnapshot& sticks) {
        maybe_startup_homing();

        // 归零在跑时由它接管输出，屏蔽摇杆，避免边跑边归零
        if (step_homing())
            return;

        const double raw = ManualRegistry::axis_value(binding, sticks);
        const double normalized = deadzone_normalize(raw, deadzone_);

        // 方向由 sensitivity 的符号决定，配反了就翻符号
        const double sensitivity =
            normalized >= 0.0 ? sensitivity_forward_ : sensitivity_reverse_;
        *control_velocity_ = normalized * sensitivity;
    }

    void update_auto() {
        maybe_startup_homing();

        if (step_homing())
            return;

        *control_velocity_ = 0.0;
    }

    // 上电首次被调用时，若 enabled 则自动启动归零。只触发一次。
    void maybe_startup_homing() {
        if (startup_checked_)
            return;
        startup_checked_ = true;

        if (!enabled_) {
            RCLCPP_INFO(
                get_logger(), "[CarriageNew] enabled=false, skip startup homing");
            return;
        }
        start_homing();
    }

    // ── ② 归零定位 ────────────────────────────────────────────────────────────
    //
    // 纯线性顺序动作，只有「完成」和「超时失败」两个出口，不需要状态机：
    //   阶段 1：向上找微动开关
    //   阶段 2：向下走（先慢速退离 back_off_angle_，再按 down_speed 走到工作位）
    //   阶段 3：完成，停在原处
    //   阶段 4：失败，停住并报错
    enum : int { kIdle = 0, kFindSwitch = 1, kRunDown = 2, kDone = 3, kFailed = 4 };

    void start_homing() {
        phase_ = kFindSwitch;
        homing_begin_ = std::chrono::steady_clock::now();
        hit_angle_ = 0.0;
        *control_velocity_ = 0.0;
        RCLCPP_INFO(get_logger(), "[CarriageNew] homing start (up_sign=%+.0f)", up_sign_);
    }

    // 推进归零。返回 true 表示归零正在接管输出。
    bool step_homing() {
        if (phase_ != kFindSwitch && phase_ != kRunDown)
            return false;

        if (!angle_.ready() || !micro_switch_.ready()) {
            *control_velocity_ = 0.0;
            return true;
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - homing_begin_)
                                 .count();
        if (elapsed >= homing_timeout_ms_) {
            phase_ = kFailed;
            *control_velocity_ = 0.0;
            RCLCPP_ERROR(
                get_logger(),
                "[CarriageNew] homing FAILED: timeout %ld ms (phase=%d, angle=%.3f)",
                static_cast<long>(homing_timeout_ms_), phase_, *angle_);
            return true;
        }

        // 阶段 1：向上找开关
        if (phase_ == kFindSwitch) {
            if (!*micro_switch_) {
                *control_velocity_ = up_sign_ * homing_speed_;
                return true;
            }
            hit_angle_ = *angle_;
            phase_ = kRunDown;
            RCLCPP_INFO(
                get_logger(), "[CarriageNew] switch HIT at angle %.3f, running down", hit_angle_);
        }

        // 阶段 2：向下走。退离段慢、之后快，用同一个 travelled 判断
        const double travelled = (hit_angle_ - *angle_) * up_sign_;
        if (travelled >= back_off_angle_ + down_angle_) {
            phase_ = kDone;
            *control_velocity_ = 0.0;
            RCLCPP_INFO(
                get_logger(), "[CarriageNew] HOMING DONE, working position (%.3f rad)",
                travelled);
            return true;
        }
        *control_velocity_ =
            -up_sign_ * (travelled < back_off_angle_ ? back_off_speed_ : down_speed_);
        return true;
    }


    // ── 说明书 ────────────────────────────────────────────────────────────────
    // 左拨杆 UP + 右拨杆 MIDDLE → 手动；输入取右摇杆 Y 轴。
    // 该档位下左摇杆 Y 归 belt，与 carriage 互不干扰。
    static constexpr ManualOps kOps{&CarriageNewController::on_manual,
                                    &CarriageNewController::on_auto};

    static constexpr ManualBinding kManual{
        .name = "carriage",
        .left = rmcs_msgs::Switch::UP,
        .right = rmcs_msgs::Switch::MIDDLE,
        .side = StickSide::kRight,
        .axis = StickAxis::kY,
        .ops = &kOps,
    };

    InputInterface<bool> micro_switch_;
    InputInterface<double> angle_;
    OutputInterface<double> control_velocity_;

    // 手动
    double sensitivity_forward_{};
    double sensitivity_reverse_{};
    double deadzone_{};

    // 归零
    bool enabled_{false};          // 上电是否自动启动归零
    bool startup_checked_{false};  // 上电检查只做一次
    double up_sign_{1.0};

    double homing_speed_{};
    std::int64_t homing_timeout_ms_{};

    double back_off_speed_{};
    double back_off_angle_{};

    double down_speed_{};
    double down_angle_{};

    int phase_{kIdle};
    std::chrono::steady_clock::time_point homing_begin_{std::chrono::steady_clock::now()};
    double hit_angle_{};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::CarriageNewController, rmcs_executor::Component)


