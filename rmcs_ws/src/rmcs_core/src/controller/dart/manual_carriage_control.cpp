#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

// ─────────────────────────────────────────────────────────────────────────────
// ManualCarriageControl
//   遥控器摇杆手动控制弹舱 carriage 速度。
//
//   摇杆值 [-1, 1] × sensitivity → 目标速度（rad/s，输出轴）。
//   正/负方向可用不同 sensitivity，因为向上（撞开关方向）应比向下慢。
//   死区内输出 0，避免摇杆回中时电机蠕动。
//
//   只写 /dart/carriage_motor/control_velocity，闭环由 PidController 负责。
// ─────────────────────────────────────────────────────────────────────────────
class ManualCarriageControl
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ManualCarriageControl()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        const auto axis = get_parameter("joystick_axis").as_string();

        if (axis == "left_x" || axis == "left_y") {
            register_input("/remote/joystick/left", joystick_, false);
        } else if (axis == "right_x" || axis == "right_y") {
            register_input("/remote/joystick/right", joystick_, false);
        } else {
            throw std::runtime_error(
                "manual_carriage_control: invalid joystick_axis '" + axis
                + "' (expected left_x/left_y/right_x/right_y)");
        }

        use_y_axis_ = (axis == "left_y" || axis == "right_y");

        sensitivity_forward_ = get_parameter("sensitivity_forward").as_double();
        sensitivity_reverse_ = get_parameter("sensitivity_reverse").as_double();
        deadzone_ = std::abs(get_parameter("deadzone").as_double());

        register_output("/dart/carriage_motor/control_velocity", control_velocity_, 0.0);
    }

    ManualCarriageControl(const ManualCarriageControl&) = delete;
    ManualCarriageControl& operator=(const ManualCarriageControl&) = delete;
    ManualCarriageControl(ManualCarriageControl&&) = delete;
    ManualCarriageControl& operator=(ManualCarriageControl&&) = delete;

    ~ManualCarriageControl() override = default;

    void update() override {
        if (!joystick_.ready()) {
            *control_velocity_ = 0.0;
            return;
        }

        const double raw = use_y_axis_ ? joystick_->y() : joystick_->x();
        const double value = std::abs(raw) < deadzone_ ? 0.0 : raw;

        const double sensitivity = value >= 0.0 ? sensitivity_forward_ : sensitivity_reverse_;
        *control_velocity_ = value * sensitivity;
    }

private:
    InputInterface<Eigen::Vector2d> joystick_;
    OutputInterface<double> control_velocity_;

    bool use_y_axis_{false};

    double sensitivity_forward_{};
    double sensitivity_reverse_{};
    double deadzone_{};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::ManualCarriageControl, rmcs_executor::Component)
