#include <cstddef>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/dart/manual_dispatch.hpp"

namespace rmcs_core::controller::dart {

// ─────────────────────────────────────────────────────────────────────────────
// ManualDispatcher
//   遥控器手动模式分发组件。
//
//   职责：每 tick 读一次拨杆与摇杆，然后遍历注册表，把每个机构分到手动或自动。
//   它不认识任何具体机构——各控制器自己带着说明书（ManualBinding）注册进来。
//
//   新增机构时本文件不用改，只要控制器在自己的构造函数里
//   ManualRegistry::add(&kManual, this) 即可。
// ─────────────────────────────────────────────────────────────────────────────
class ManualDispatcher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ManualDispatcher()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/remote/joystick/right", joystick_right_, false);

        register_output("/dart/manual/active_count", active_count_, 0.0);
    }

    ManualDispatcher(const ManualDispatcher&) = delete;
    ManualDispatcher& operator=(const ManualDispatcher&) = delete;
    ManualDispatcher(ManualDispatcher&&) = delete;
    ManualDispatcher& operator=(ManualDispatcher&&) = delete;

    ~ManualDispatcher() override = default;

    void before_updating() override {
        if (!switch_left_.ready()) {
            switch_left_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }
        if (!joystick_left_.ready()) {
            joystick_left_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/joystick/left\". Set to zero.");
        }
        if (!joystick_right_.ready()) {
            joystick_right_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/joystick/right\". Set to zero.");
        }

        dump_registry();
    }

    void update() override {
        sticks_.left = *joystick_left_;
        sticks_.right = *joystick_right_;

        const auto count = ManualRegistry::dispatch(*switch_left_, *switch_right_, sticks_);
        *active_count_ = static_cast<double>(count);
    }

private:
    // 上电打印档位占用表：现场直接核对拨杆映射，不用翻文档
    void dump_registry() {
        const auto& entries = ManualRegistry::entries();
        RCLCPP_INFO(get_logger(), "[ManualDispatcher] %zu mechanism(s) registered", entries.size());

        for (const auto& entry : entries) {
            const auto& b = *entry.binding;
            RCLCPP_INFO(
                get_logger(), "[ManualDispatcher]   %-10s left=%-4s right=%-4s stick=%-5s axis=%s",
                b.name, switch_name(b.left), switch_name(b.right),
                b.side == StickSide::kRight ? "right" : "left",
                b.axis == StickAxis::kY ? "Y" : "X");
        }
    }

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    OutputInterface<double> active_count_;

    StickSnapshot sticks_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::ManualDispatcher, rmcs_executor::Component)
