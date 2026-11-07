#include <chrono>
#include <cstdint>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

// ─────────────────────────────────────────────────────────────────────────────
// MotorSpinTest
//   电机方向验证控制器：周期性在 +speed / -speed 之间切换目标速度。
//
//   它只做一件事——往 /dart/carriage_motor/control_velocity 写目标值。
//   真正闭环的 PID 由 PidController 组件负责，CAN 发送由硬件组件负责。
//
//   想换成真实业务逻辑时，在 yaml 的 components 里换掉本组件即可，
//   硬件组件与 PID 组件都不用改。
// ─────────────────────────────────────────────────────────────────────────────
class MotorSpinTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MotorSpinTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_output("/dart/carriage_motor/control_velocity", control_velocity_, 0.0);

        speed_ = get_parameter("speed").as_double();
        switch_period_ms_ = get_parameter("switch_period_ms").as_int();
        cycle_enabled_ = get_parameter("cycle_enabled").as_bool();
    }

    MotorSpinTest(const MotorSpinTest&) = delete;
    MotorSpinTest& operator=(const MotorSpinTest&) = delete;
    MotorSpinTest(MotorSpinTest&&) = delete;
    MotorSpinTest& operator=(MotorSpinTest&&) = delete;

    ~MotorSpinTest() override = default;

    void update() override {
        if (!cycle_enabled_) {
            *control_velocity_ = speed_;
            return;
        }

        const auto now = std::chrono::steady_clock::now();

        if (!cycle_started_) {
            cycle_started_ = true;
            cycle_begin_ = now;
            positive_ = true;
            RCLCPP_INFO(get_logger(), "[MotorSpinTest] cycle START forward %.2f rad/s", speed_);
        } else if (
            std::chrono::duration_cast<std::chrono::milliseconds>(now - cycle_begin_).count()
            >= switch_period_ms_) {
            cycle_begin_ = now;
            positive_ = !positive_;
            RCLCPP_INFO(
                get_logger(), "[MotorSpinTest] cycle -> %s %.2f rad/s",
                positive_ ? "forward" : "reverse", positive_ ? speed_ : -speed_);
        }

        *control_velocity_ = positive_ ? speed_ : -speed_;
    }

private:
    OutputInterface<double> control_velocity_;

    double speed_{};
    std::int64_t switch_period_ms_{};
    bool cycle_enabled_{};

    bool cycle_started_{false};
    std::chrono::steady_clock::time_point cycle_begin_{};
    bool positive_{true};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::MotorSpinTest, rmcs_executor::Component)
