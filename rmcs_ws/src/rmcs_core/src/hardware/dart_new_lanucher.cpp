#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/limit_switch.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/pwm_servo.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/uart_servo.hpp"
#include "librmcs/board/rmcs_board_pro.hpp"
#include "librmcs/data/datas.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>

namespace rmcs_core::hardware {

class DartNewlanucher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartNewlanucher()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
            , command_component_(
                create_partner_component<DartNewlanucherCommand>(
                get_component_name() + "_command", *this)) {

        remote_control_ = std::make_unique<device::RemoteControl>(*this);

        dart_board_ = std::make_unique<DartBoard>(
            *this, *command_component_, get_parameter("board_serial_top_board").as_string());                
              }

    DartNewlanucher(const DartNewlanucher&) = delete;
    DartNewlanucher& operator=(const DartNewlanucher&) = delete;
    DartNewlanucher(DartNewlanucher&&) = delete;
    DartNewlanucher& operator=(DartNewlanucher&&) = delete;

    ~DartNewlanucher() override = default;

    void update() override {

    }

private:
    class DartNewlanucherCommand : public rmcs_executor::Component {
        explicit DartNewlanucherCommand(DartNewlanucherCommand& hero)
            : hero_(hero) {}

        void update() override { hero_.command_update(); }

        DartNewlanucherCommand& hero_;

    }

};
}// namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartNewlanucher, rmcs_executor::Component)
