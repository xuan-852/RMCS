#include <chrono>
#include <cmath>
#include <cstdint>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

// ─────────────────────────────────────────────────────────────────────────────
// CarriageHoming
//   弹舱 carriage 全链路验证控制器：向上撞微动开关归零，再向下运行到工作位。
//
//   状态机：
//     HomingUp ──撞开关──> BackOff ──退离──> DownRun ──走完──> Done
//        └──超时──────────> Failed
//
//   本组件只写 /dart/carriage_motor/control_velocity，
//   速度闭环由 PidController 负责，CAN 收发由硬件组件负责。
// ─────────────────────────────────────────────────────────────────────────────
class CarriageHoming
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CarriageHoming()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        // 读取
        register_input("/dart/micro_switch", micro_switch_, false);
        register_input("/dart/carriage_motor/angle", angle_, false);

        // 下发
        register_output("/dart/carriage_motor/control_velocity", control_velocity_, 0.0);

        enabled_ = get_parameter("enabled").as_bool();
        // 向上对应的速度符号：+1 表示 velocity > 0 是向上，-1 反之
        up_sign_ = get_parameter("up_sign").as_double() >= 0.0 ? 1.0 : -1.0;

        homing_speed_ = get_parameter("homing_speed").as_double();
        homing_timeout_ms_ = get_parameter("homing_timeout_ms").as_int();

        back_off_speed_ = get_parameter("back_off_speed").as_double();
        back_off_angle_ = get_parameter("back_off_angle").as_double();

        down_speed_ = get_parameter("down_speed").as_double();
        down_angle_ = get_parameter("down_angle").as_double();
    }

    CarriageHoming(const CarriageHoming&) = delete;
    CarriageHoming& operator=(const CarriageHoming&) = delete;
    CarriageHoming(CarriageHoming&&) = delete;
    CarriageHoming& operator=(CarriageHoming&&) = delete;

    ~CarriageHoming() override = default;

    void update() override {
        if (!enabled_) {
            *control_velocity_ = 0.0;
            return;
        }
        if (!angle_.ready() || !micro_switch_.ready()) {
            *control_velocity_ = 0.0;
            return;
        }

        const auto now = std::chrono::steady_clock::now();

        switch (state_) {
        case State::kHomingUp: {
            // 撞到微动开关：记录现场，转入退离
            if (*micro_switch_) {
                angle_at_hit_ = *angle_;
                state_ = State::kBackOff;
                state_begin_ = now;
                RCLCPP_INFO(
                    get_logger(), "[CarriageHoming] switch HIT at angle %.3f, backing off",
                    angle_at_hit_);
                *control_velocity_ = -up_sign_ * back_off_speed_;
                return;
            }
            // 超时保护：走了太久还没撞到，说明开关坏了或方向反了
            if (elapsed_ms(now, state_begin_) >= homing_timeout_ms_) {
                state_ = State::kFailed;
                *control_velocity_ = 0.0;
                RCLCPP_ERROR(
                    get_logger(),
                    "[CarriageHoming] homing TIMEOUT after %ld ms (angle %.3f), stopping. "
                    "Check micro switch wiring or up_sign.",
                    static_cast<long>(homing_timeout_ms_), *angle_);
                return;
            }
            *control_velocity_ = up_sign_ * homing_speed_;
            return;
        }

        case State::kBackOff: {
            // 退离距离：以撞到开关那一刻为基准
            const double travelled = (angle_at_hit_ - *angle_) * up_sign_;
            if (travelled >= back_off_angle_) {
                state_ = State::kDownRun;
                state_begin_ = now;
                RCLCPP_INFO(
                    get_logger(), "[CarriageHoming] back off done (%.3f rad), running down",
                    travelled);
                *control_velocity_ = -up_sign_ * down_speed_;
                return;
            }
            *control_velocity_ = -up_sign_ * back_off_speed_;
            return;
        }

        case State::kDownRun: {
            const double travelled = (angle_at_hit_ - *angle_) * up_sign_;
            if (travelled >= back_off_angle_ + down_angle_) {
                state_ = State::kDone;
                *control_velocity_ = 0.0;
                RCLCPP_INFO(
                    get_logger(), "[CarriageHoming] DONE, carriage at working position (%.3f rad)",
                    travelled);
                return;
            }
            *control_velocity_ = -up_sign_ * down_speed_;
            return;
        }

        case State::kDone:
        case State::kFailed: *control_velocity_ = 0.0; return;
        }
    }

private:
    enum class State { kHomingUp, kBackOff, kDownRun, kDone, kFailed };

    static std::int64_t elapsed_ms(
        const std::chrono::steady_clock::time_point& now,
        const std::chrono::steady_clock::time_point& begin) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count();
    }

    InputInterface<bool> micro_switch_;
    InputInterface<double> angle_;
    OutputInterface<double> control_velocity_;

    bool enabled_{true};
    double up_sign_{1.0};

    double homing_speed_{};
    std::int64_t homing_timeout_ms_{};

    double back_off_speed_{};
    double back_off_angle_{};

    double down_speed_{};
    double down_angle_{};

    State state_{State::kHomingUp};
    std::chrono::steady_clock::time_point state_begin_{std::chrono::steady_clock::now()};
    double angle_at_hit_{};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::CarriageHoming, rmcs_executor::Component)
