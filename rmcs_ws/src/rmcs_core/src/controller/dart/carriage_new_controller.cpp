#include <algorithm>
#include <cmath>
#include <cstdint>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/dart/manual_dispatch.hpp"

namespace rmcs_core::controller::dart {

// ─────────────────────────────────────────────────────────────────────────────
// CarriageNewController
//   Board Lite 新镖架弹舱电机的执行器。只做一件事：
//
//   把「目标速度」转发给弹舱电机，把「标零请求」转发给硬件层。
//   手动遥控器档位由 kManual 说明书声明，经 ManualDispatcher 分发。
//
//   不判断到位、不跑归零状态机、不含超时。
//   归零流程（撞开关 → 退离 → 下行到工作位）由上位机任务层按 tick 推进：
//   上位机读 /dart/carriage/limit_switch 与 /angle 自行判断，再写目标。
//
//   输出永远是有限速度，绝不用 NaN
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
        register_input("/dart/carriage_motor/velocity", velocity_, false);

        register_output("/dart/carriage_motor/control_velocity", control_velocity_, 0.0);
        register_output("/dart/carriage_motor/calibrate_zero", calibrate_, false);

        // 上位机目标输入
        register_input("/dart/carriage_motor/cmd_velocity", cmd_velocity_, false);
        register_input("/dart/carriage_motor/calibrate_request", cmd_calibrate_, false);

        load_parameters();
        ManualRegistry::add(debug_mode_ ? &kManualDebug : &kManual, this);
    }

    CarriageNewController(const CarriageNewController&) = delete;
    CarriageNewController& operator=(const CarriageNewController&) = delete;
    CarriageNewController(CarriageNewController&&) = delete;
    CarriageNewController& operator=(CarriageNewController&&) = delete;

    ~CarriageNewController() override = default;

    void before_updating() override {
        bind_or(micro_switch_, false, "/dart/micro_switch");
        bind_or(angle_, 0.0, "/dart/carriage_motor/angle");
        bind_or(velocity_, 0.0, "/dart/carriage_motor/velocity");
        bind_or(cmd_velocity_, 0.0, "/dart/carriage_motor/cmd_velocity");
        bind_or(cmd_calibrate_, false, "/dart/carriage_motor/calibrate_request");
    }

    // 手动档位命中时用摇杆值，否则转发上位机目标。
    // manual_active_ 每 tick 复位，离档立刻回到上位机控制。
    void update() override {
        forward_targets();
        manual_active_ = false;
    }

    // ── ManualRegistry 的两个动作 ─────────────────────────────────────────────
    static void on_manual(void* self, const ManualBinding& binding, const StickSnapshot& sticks) {
        static_cast<CarriageNewController*>(self)->run_manual(binding, sticks);
    }

    static void on_auto(void* self) { (void)self; }
    template <typename T, typename V>
    void bind_or(InputInterface<T>& interface, V&& value, const char* name) {
        if (interface.ready())
            return;
        interface.make_and_bind_directly(T(std::forward<V>(value)));
        RCLCPP_WARN(get_logger(), "Failed to fetch \"%s\". Set to default.", name);
    }

    void load_parameters() {
        const auto d = [this](const char* n) { return get_parameter(n).as_double(); };

        debug_mode_ = get_parameter("debug_mode").as_bool();
        debug_torque_limit_unused_ = d("debug_torque_limit");

        sensitivity_forward_ = d("sensitivity_forward");
        sensitivity_reverse_ = d("sensitivity_reverse");
        deadzone_ = std::abs(d("deadzone"));
    }

    // ── 把目标转发给弹舱电机 ──────────────────────────────────────────────────
    void forward_targets() {
        double velocity = cmd_velocity_.ready() ? *cmd_velocity_ : 0.0;
        bool calibrate = cmd_calibrate_.ready() && *cmd_calibrate_;

        if (manual_active_) {
            velocity = manual_velocity_;
            calibrate = false;
        }

        *control_velocity_ = velocity;
        *calibrate_ = calibrate;
    }

    // ── 手动：摇杆直驱 ────────────────────────────────────────────────────────
    void run_manual(const ManualBinding& binding, const StickSnapshot& sticks) {
        manual_active_ = true;

        const double raw = ManualRegistry::axis_value(binding, sticks);
        const double normalized = deadzone_normalize(raw, deadzone_);
        const double sensitivity =
            normalized >= 0.0 ? sensitivity_forward_ : sensitivity_reverse_;

        manual_velocity_ = normalized * sensitivity;
    }

    // ── 说明书 ────────────────────────────────────────────────────────────────
    // 左拨杆 UP + 右拨杆 MIDDLE → 手动；输入取右摇杆 Y 轴。
    // 与 belt 同档，左摇杆归 belt、右摇杆归 carriage（可同时操作）。
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

    // 调试档位：同一拨杆组合，保留独立名字便于日志区分
    static constexpr ManualBinding kManualDebug{
        .name = "carriage-dbg",
        .left = rmcs_msgs::Switch::UP,
        .right = rmcs_msgs::Switch::MIDDLE,
        .side = StickSide::kRight,
        .axis = StickAxis::kY,
        .ops = &kOps,
    };

    // 反馈
    InputInterface<bool> micro_switch_;
    InputInterface<double> angle_;
    InputInterface<double> velocity_;

    // 上位机目标输入
    InputInterface<double> cmd_velocity_;
    InputInterface<bool> cmd_calibrate_;

    // 电机指令
    OutputInterface<double> control_velocity_;
    OutputInterface<bool> calibrate_;

    // 手动
    bool manual_active_{false};
    double manual_velocity_{0.0};
    double sensitivity_forward_{};
    double sensitivity_reverse_{};
    double deadzone_{};

    // 调试模式（carriage 无独立力矩限幅，保留参数以免 yaml 报未声明）
    bool debug_mode_{false};
    double debug_torque_limit_unused_{};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::CarriageNewController, rmcs_executor::Component)


