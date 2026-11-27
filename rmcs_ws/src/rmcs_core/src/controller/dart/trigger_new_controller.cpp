#include <algorithm>
#include <cmath>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/dart/manual_dispatch.hpp"

namespace rmcs_core::controller::dart {

// ─────────────────────────────────────────────────────────────────────────────
// TriggerNewController
//   Board Pro 新镖架扳机舵机控制器。
//   输出归一化位置 /dart/trigger_servo/value ∈ [0.0, 1.0]：
//     - 0.0 → 1900 us（释放/发射）
//     - 1.0 → 2050 us（锁住）
//
//   手动滚轮采用「拨到最右上升沿切换并保持」逻辑（适配 DR16 自动回弹滚轮）：
//     - 使能初始状态为 initial_value（默认 0.0 → 1900 us）；
//     - 每次拨到最右越过 wheel_trigger_threshold，触发一次 0.0 ⇄ 1.0 循环切换，
//       滚轮回弹到 wheel_release_threshold 以下后解锁下一次触发；
//     - 遥控断联 / 双下急停时由 safe_lock_value（及硬件层 motor_disabled_）强制锁在 1.0（2050 us）。
// ─────────────────────────────────────────────────────────────────────────────
class TriggerNewController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TriggerNewController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/dart/trigger_servo/cmd_value", cmd_value_, false);

        load_parameters();
        toggled_value_ = initial_value_;
        register_output("/dart/trigger_servo/value", servo_value_, safe_lock_value_);

        ManualRegistry::add(wheel_always_active_ ? &kManualAlways : &kManualSwitch, this);
    }

    TriggerNewController(const TriggerNewController&) = delete;
    TriggerNewController& operator=(const TriggerNewController&) = delete;
    TriggerNewController(TriggerNewController&&) = delete;
    TriggerNewController& operator=(TriggerNewController&&) = delete;

    ~TriggerNewController() override = default;

    void before_updating() override {
        if (!cmd_value_.ready()) {
            cmd_value_.make_and_bind_directly(safe_lock_value_);
            RCLCPP_INFO(
                get_logger(),
                "[TriggerNewController] \"/dart/trigger_servo/cmd_value\" unbound, "
                "default to safe_lock_value=%.2f",
                safe_lock_value_);
        }
    }

    void update() override {
        double target_value = safe_lock_value_;

        if (manual_active_) {
            target_value = toggled_value_;
        } else {
            wheel_pressed_ = false;
            if (!wheel_always_active_ && cmd_value_.ready() && std::isfinite(*cmd_value_)) {
                target_value = std::clamp(*cmd_value_, 0.0, 1.0);
            }
        }

        manual_active_ = false;
        *servo_value_ = std::clamp(target_value, 0.0, 1.0);
    }

private:
    static void on_manual(void* self, const ManualBinding& binding, const StickSnapshot& sticks) {
        static_cast<TriggerNewController*>(self)->run_manual(binding, sticks);
    }

    static void on_auto(void* self) { (void)self; }

    void load_parameters() {
        get_parameter_or("wheel_always_active", wheel_always_active_, true);
        get_parameter_or("wheel_invert", wheel_invert_, false);
        get_parameter_or("wheel_trigger_threshold", wheel_trigger_threshold_, 0.75);
        get_parameter_or("wheel_release_threshold", wheel_release_threshold_, 0.45);
        get_parameter_or("initial_value", initial_value_, 0.0);
        get_parameter_or("safe_lock_value", safe_lock_value_, 1.0);

        wheel_trigger_threshold_ = std::clamp(wheel_trigger_threshold_, 0.1, 0.98);
        wheel_release_threshold_ =
            std::clamp(wheel_release_threshold_, 0.0, wheel_trigger_threshold_ - 0.05);
        initial_value_ = (initial_value_ >= 0.5) ? 1.0 : 0.0;
        safe_lock_value_ = std::clamp(safe_lock_value_, 0.0, 1.0);

        RCLCPP_INFO(
            get_logger(),
            "[TriggerNewController] wheel_always_active=%s, wheel_invert=%s, "
            "threshold=[%.2f, %.2f], initial=%.1f, safe_lock=%.1f",
            wheel_always_active_ ? "true" : "false", wheel_invert_ ? "true" : "false",
            wheel_release_threshold_, wheel_trigger_threshold_, initial_value_, safe_lock_value_);
    }

    void run_manual(const ManualBinding& binding, const StickSnapshot& sticks) {
        manual_active_ = true;
        const double raw_knob = ManualRegistry::axis_value(binding, sticks);
        const double signed_knob = wheel_invert_ ? -raw_knob : raw_knob;

        if (!wheel_pressed_ && signed_knob >= wheel_trigger_threshold_) {
            wheel_pressed_ = true;
            toggled_value_ = (toggled_value_ < 0.5) ? 1.0 : 0.0;
            RCLCPP_INFO(
                get_logger(), "[TriggerNewController] wheel toggle -> %.1f (%s)",
                toggled_value_, toggled_value_ >= 0.5 ? "LOCK 2050us" : "FREE 1900us");
        } else if (wheel_pressed_ && signed_knob <= wheel_release_threshold_) {
            wheel_pressed_ = false;
        }
    }

    static constexpr ManualOps kOps{
        &TriggerNewController::on_manual, &TriggerNewController::on_auto};

    static constexpr ManualBinding kManualAlways{
        .name = "trigger",
        .left = rmcs_msgs::Switch::UNKNOWN,
        .right = rmcs_msgs::Switch::UNKNOWN,
        .side = StickSide::kLeft,
        .axis = StickAxis::kRotary,
        .ops = &kOps,
        .any_enabled_switch = true,
    };

    static constexpr ManualBinding kManualSwitch{
        .name = "trigger",
        .left = rmcs_msgs::Switch::UP,
        .right = rmcs_msgs::Switch::MIDDLE,
        .side = StickSide::kLeft,
        .axis = StickAxis::kRotary,
        .ops = &kOps,
        .any_enabled_switch = false,
    };

    InputInterface<double> cmd_value_;
    OutputInterface<double> servo_value_;

    bool wheel_always_active_{true};
    bool wheel_invert_{false};
    double wheel_trigger_threshold_{0.75};
    double wheel_release_threshold_{0.45};
    double initial_value_{0.0};
    double safe_lock_value_{1.0};

    bool manual_active_{false};
    bool wheel_pressed_{false};
    double toggled_value_{0.0};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::TriggerNewController, rmcs_executor::Component)
