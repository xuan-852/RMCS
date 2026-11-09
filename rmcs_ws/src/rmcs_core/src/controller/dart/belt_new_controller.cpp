#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/dart/manual_dispatch.hpp"

namespace rmcs_core::controller::dart {

// ─────────────────────────────────────────────────────────────────────────────
// BeltNewController
//   Board Lite 新镖架的同步带控制器。
//
//   只做三件事：
//     ① 归零 —— 两侧各自顶硬限位，堵转后把该电机当前位置设为 0 点。
//        上电自动执行，由 yaml 的 enabled 开关控制。
//     ② 手动遥控器控制 —— 档位由 kManual 说明书声明，经 ManualDispatcher 分发。
//     ③ 左右同步 —— 两侧不共轴，必须闭环，否则位置差累积会损坏机械。
//
//   ⚠️ 归零未完成前手动控制完全无效。
//      因为同步依赖「两侧编码器都为 0」这个前提，未归零时 angle_L - angle_R
//      是随机值，拿它做同步会输出错误修正量，把滑台拽歪。
//
//   同步公式（交叉耦合，两侧各担一半，总速度守恒）：
//     err  = angle_L - angle_R          两侧都已归零，理想值 0
//     corr = clamp(sync_gain * err, ±sync_correction_max)
//     l_vel -= corr / 2
//     r_vel += corr / 2
// ─────────────────────────────────────────────────────────────────────────────
class BeltNewController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    BeltNewController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/dart/belt/left_motor/angle", left_angle_, false);
        register_input("/dart/belt/right_motor/angle", right_angle_, false);
        register_input("/dart/belt/left_motor/velocity", left_velocity_, false);
        register_input("/dart/belt/right_motor/velocity", right_velocity_, false);
        register_input("/dart/belt/left_motor/torque", left_torque_, false);
        register_input("/dart/belt/right_motor/torque", right_torque_, false);

        register_output("/dart/belt/left_motor/control_velocity", left_control_velocity_, 0.0);
        register_output("/dart/belt/right_motor/control_velocity", right_control_velocity_, 0.0);
        register_output(
            "/dart/belt/left_motor/control_torque_limit", left_control_torque_limit_, 0.0);
        register_output(
            "/dart/belt/right_motor/control_torque_limit", right_control_torque_limit_, 0.0);

        // 归零请求：写 true 让硬件层调 DjiMotor::calibrate_zero_point()
        register_output("/dart/belt/left_motor/calibrate_zero", left_calibrate_, false);
        register_output("/dart/belt/right_motor/calibrate_zero", right_calibrate_, false);

        load_parameters();
        ManualRegistry::add(&kManual, this);
    }

    BeltNewController(const BeltNewController&) = delete;
    BeltNewController& operator=(const BeltNewController&) = delete;
    BeltNewController(BeltNewController&&) = delete;
    BeltNewController& operator=(BeltNewController&&) = delete;

    ~BeltNewController() override = default;

    void before_updating() override {
        bind_or(left_angle_, 0.0, "/dart/belt/left_motor/angle");
        bind_or(right_angle_, 0.0, "/dart/belt/right_motor/angle");
        bind_or(left_velocity_, 0.0, "/dart/belt/left_motor/velocity");
        bind_or(right_velocity_, 0.0, "/dart/belt/right_motor/velocity");
        bind_or(left_torque_, 0.0, "/dart/belt/left_motor/torque");
        bind_or(right_torque_, 0.0, "/dart/belt/right_motor/torque");
    }

    void update() override {}

    static void on_manual(void* self, const ManualBinding& binding, const StickSnapshot& sticks) {
        static_cast<BeltNewController*>(self)->run(&binding, &sticks);
    }

    static void on_auto(void* self) {
        static_cast<BeltNewController*>(self)->run(nullptr, nullptr);
    }

private:
    enum : int { kIdle = 0, kFindLimit = 1, kDone = 2, kFailed = 3 };

    template <typename T, typename V>
    void bind_or(InputInterface<T>& interface, V&& value, const char* name) {
        if (interface.ready())
            return;
        interface.make_and_bind_directly(T(std::forward<V>(value)));
        RCLCPP_WARN(get_logger(), "Failed to fetch \"%s\". Set to default.", name);
    }

    void load_parameters() {
        const auto d = [this](const char* n) { return get_parameter(n).as_double(); };
        const auto i = [this](const char* n) { return get_parameter(n).as_int(); };

        enabled_ = get_parameter("enabled").as_bool();

        // 向上对应的速度符号：+1 = 正速度为向上
        up_sign_ = d("up_sign") >= 0.0 ? 1.0 : -1.0;

        homing_speed_ = d("homing_speed");
        homing_timeout_ms_ = i("homing_timeout_ms");
        homing_torque_limit_ = d("homing_torque_limit");

        stall_velocity_ = d("stall_velocity_threshold");
        stall_torque_ = d("stall_torque_threshold");
        stall_ticks_ = i("stall_ticks");

        sensitivity_forward_ = d("sensitivity_forward");
        sensitivity_reverse_ = d("sensitivity_reverse");
        deadzone_ = std::abs(d("deadzone"));
        manual_torque_limit_ = d("manual_torque_limit");

        sync_gain_ = d("sync_gain");
        sync_correction_max_ = d("sync_correction_max");
        sync_err_limit_ = d("sync_err_limit");
        sync_err_fault_ticks_ = i("sync_err_fault_ticks");
    }

    // 统一入口：手动和自动都走这里
    void run(const ManualBinding* binding, const StickSnapshot* sticks) {
        maybe_startup_homing();

        // 归零在跑：由它接管输出，摇杆完全无效
        if (step_homing())
            return;

        // 归零未完成：禁止任何运动。
        // 同步依赖两侧零位一致，未归零时位置差是随机值，动了会把机械拽歪。
        if (!homed_) {
            hold_still();
            return;
        }

        // 归零完成但摇杆不在本机构档位 / 自动模式：停车
        if (binding == nullptr || sticks == nullptr) {
            hold_still();
            return;
        }

        const double raw = ManualRegistry::axis_value(*binding, *sticks);
        const double normalized = deadzone_normalize(raw, deadzone_);
        const double sensitivity =
            normalized >= 0.0 ? sensitivity_forward_ : sensitivity_reverse_;

        double l_vel = normalized * sensitivity;
        double r_vel = l_vel;

        apply_sync(l_vel, r_vel);

        *left_control_velocity_ = l_vel;
        *right_control_velocity_ = r_vel;
        *left_control_torque_limit_ = manual_torque_limit_;
        *right_control_torque_limit_ = manual_torque_limit_;
    }

    // ── 同步：交叉耦合，各担一半，总速度守恒 ──────────────────────────────────
    void apply_sync(double& l_vel, double& r_vel) {
        if (!homed_ || sync_gain_ <= 0.0)
            return;
        if (!left_angle_.ready() || !right_angle_.ready())
            return;

        // 两侧都已归零，理想状态是角度相等
        const double err = *left_angle_ - *right_angle_;
        sync_err_ = err;

        const double corr = std::clamp(sync_gain_ * err, -sync_correction_max_, sync_correction_max_);

        l_vel -= corr * 0.5;
        r_vel += corr * 0.5;

        // 同步误差故障检测：持续超限说明打滑或单侧卡死
        if (std::abs(err) > sync_err_limit_) {
            if (++sync_err_ticks_ >= sync_err_fault_ticks_) {
                sync_fault_ = true;
                l_vel = 0.0;
                r_vel = 0.0;
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "[BeltNew] SYNC FAULT: |err|=%.3f > %.3f for %d ticks, stopping",
                    std::abs(err), sync_err_limit_, sync_err_ticks_);
            }
        } else {
            sync_err_ticks_ = 0;
            sync_fault_ = false;
        }
    }

    // ── 归零：两侧各自顶限位，堵转后设零点 ────────────────────────────────────
    void maybe_startup_homing() {
        if (startup_checked_)
            return;
        startup_checked_ = true;

        if (!enabled_) {
            RCLCPP_INFO(get_logger(), "[BeltNew] enabled=false, skip startup homing");
            return;
        }
        start_homing();
    }

    void start_homing() {
        phase_ = kFindLimit;
        homing_begin_ = std::chrono::steady_clock::now();
        left_stall_count_ = 0;
        right_stall_count_ = 0;
        left_zeroed_ = false;
        right_zeroed_ = false;
        homed_ = false;
        *left_calibrate_ = false;
        *right_calibrate_ = false;
        RCLCPP_INFO(get_logger(), "[BeltNew] homing start (up_sign=%+.0f)", up_sign_);
    }

    // 推进归零。返回 true 表示归零正在接管输出。
    bool step_homing() {
        if (phase_ != kFindLimit)
            return false;

        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - homing_begin_)
                                 .count();
        if (elapsed >= homing_timeout_ms_) {
            phase_ = kFailed;
            hold_still();
            RCLCPP_ERROR(
                get_logger(),
                "[BeltNew] homing FAILED: timeout %ld ms (L_zeroed=%d R_zeroed=%d)",
                static_cast<long>(homing_timeout_ms_), left_zeroed_, right_zeroed_);
            return true;
        }

        // 两侧独立检测堵转，谁先到谁先设零并停住
        if (!left_zeroed_
            && stall_detected(*left_velocity_, *left_torque_, left_stall_count_)) {
            *left_calibrate_ = true;   // 上升沿：硬件层下一 tick 设零点
            left_zeroed_ = true;
            RCLCPP_INFO(get_logger(), "[BeltNew] LEFT stalled, zero point set");
        } else if (left_zeroed_) {
            *left_calibrate_ = false;
        }
        if (!right_zeroed_
            && stall_detected(*right_velocity_, *right_torque_, right_stall_count_)) {
            *right_calibrate_ = true;
            right_zeroed_ = true;
            RCLCPP_INFO(get_logger(), "[BeltNew] RIGHT stalled, zero point set");
        } else if (right_zeroed_) {
            *right_calibrate_ = false;
        }

        // 已归零的一侧停住，未归零的继续顶
        *left_control_velocity_ = left_zeroed_ ? 0.0 : up_sign_ * homing_speed_;
        *right_control_velocity_ = right_zeroed_ ? 0.0 : up_sign_ * homing_speed_;
        *left_control_torque_limit_ = homing_torque_limit_;
        *right_control_torque_limit_ = homing_torque_limit_;

        if (left_zeroed_ && right_zeroed_) {
            phase_ = kDone;
            homed_ = true;
            hold_still();
            RCLCPP_INFO(get_logger(), "[BeltNew] HOMING DONE, both sides zeroed");
        }
        return true;
    }

    bool stall_detected(double velocity, double torque, int& counter) {
        if (std::abs(velocity) < stall_velocity_ && std::abs(torque) > stall_torque_) {
            ++counter;
        } else {
            counter = 0;
        }
        return counter >= stall_ticks_;
    }

    // 输出零速保持（闭环保持，绝不用 NaN：竖直滑台 NaN = 零电流 = 下滑）
    void hold_still() {
        *left_control_velocity_ = 0.0;
        *right_control_velocity_ = 0.0;
        *left_control_torque_limit_ = manual_torque_limit_;
        *right_control_torque_limit_ = manual_torque_limit_;
    }

    // ── 说明书 ────────────────────────────────────────────────────────────────
    // 左拨杆 UP + 右拨杆 MIDDLE → 手动；输入取左摇杆 Y 轴。
    // 与 carriage 同档，左摇杆归 belt、右摇杆归 carriage。
    static constexpr ManualOps kOps{&BeltNewController::on_manual,
                                    &BeltNewController::on_auto};

    static constexpr ManualBinding kManual{
        .name = "belt",
        .left = rmcs_msgs::Switch::UP,
        .right = rmcs_msgs::Switch::MIDDLE,
        .side = StickSide::kLeft,
        .axis = StickAxis::kY,
        .ops = &kOps,
    };

    InputInterface<double> left_angle_;
    InputInterface<double> right_angle_;
    InputInterface<double> left_velocity_;
    InputInterface<double> right_velocity_;
    InputInterface<double> left_torque_;
    InputInterface<double> right_torque_;

    OutputInterface<double> left_control_velocity_;
    OutputInterface<double> right_control_velocity_;
    OutputInterface<double> left_control_torque_limit_;
    OutputInterface<double> right_control_torque_limit_;

    OutputInterface<bool> left_calibrate_;
    OutputInterface<bool> right_calibrate_;

    // 归零
    bool enabled_{false};
    bool startup_checked_{false};
    double up_sign_{1.0};
    double homing_speed_{};
    std::int64_t homing_timeout_ms_{};
    double homing_torque_limit_{};
    double stall_velocity_{};
    double stall_torque_{};
    int stall_ticks_{};

    int phase_{kIdle};
    std::chrono::steady_clock::time_point homing_begin_{std::chrono::steady_clock::now()};
    int left_stall_count_{};
    int right_stall_count_{};
    bool left_zeroed_{false};
    bool right_zeroed_{false};
    bool homed_{false};

    // 手动
    double sensitivity_forward_{};
    double sensitivity_reverse_{};
    double deadzone_{};
    double manual_torque_limit_{};

    // 同步
    double sync_gain_{};
    double sync_correction_max_{};
    double sync_err_limit_{};
    int sync_err_fault_ticks_{};
    double sync_err_{};
    int sync_err_ticks_{};
    bool sync_fault_{false};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::BeltNewController, rmcs_executor::Component)


