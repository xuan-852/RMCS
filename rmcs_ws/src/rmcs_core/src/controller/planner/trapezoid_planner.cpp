#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/smart_input.hpp"

namespace rmcs_core::controller::planner {

// ─────────────────────────────────────────────────────────────────────────────
// TrapezoidPlanner
//   极简梯形速度规划组件：基于实测位置做制动前瞻，并限制速度斜坡率。
//
//   接口约定（全部在 YAML 中配置）：
//     measurement     当前实际位置
//     setpoint        目标位置（NaN 时复位并输出 NaN）
//     control         输出给下级速度环的目标速度
//     passthrough     [可选，默认 NaN] 有限值时直接透传并同步内部速度状态
//     velocity_limit  [可选，默认 +inf] 动态速度上限（与 max_velocity 取较小值）
// ─────────────────────────────────────────────────────────────────────────────
class TrapezoidPlanner
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TrapezoidPlanner()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , measurement_(*this, "measurement")
        , setpoint_(*this, "setpoint")
        , passthrough_(*this, "passthrough", kNan)
        , velocity_limit_(*this, "velocity_limit", std::numeric_limits<double>::infinity()) {

        register_input("/predefined/update_rate", update_rate_, false);
        register_output(get_parameter("control").as_string(), control_, kNan);

        max_velocity_ = std::max(std::abs(get_parameter("max_velocity").as_double()), 1e-6);
        max_acceleration_ = std::max(std::abs(get_parameter("max_acceleration").as_double()), 1e-6);
        get_parameter("dt", fallback_dt_);
    }

    void update() override {
        const double passthrough = *passthrough_;
        if (std::isfinite(passthrough)) {
            last_velocity_ = passthrough;
            *control_ = passthrough;
            return;
        }

        const double meas = *measurement_;
        const double target = *setpoint_;
        const double dt = (update_rate_.ready() && *update_rate_ > 0.0)
                            ? (1.0 / *update_rate_)
                            : fallback_dt_;
        if (!std::isfinite(meas) || !std::isfinite(target) || !(dt > 0.0)) {
            last_velocity_ = 0.0;
            *control_ = kNan;
            return;
        }

        const double v_limit = std::isfinite(*velocity_limit_)
            ? std::min(max_velocity_, std::abs(*velocity_limit_))
            : max_velocity_;

        const double err = target - meas;
        const double dist = std::abs(err);
        const double dir = err >= 0.0 ? 1.0 : -1.0;

        // 离散无超调制动前瞻：保证在 max_acceleration_ 约束下到达终点时速度降为 0
        const double max_dv = max_acceleration_ * dt;
        const double v_brake = max_dv * (std::sqrt(2.0 * dist / (max_dv * dt) + 0.25) - 0.5);
        const double v_des = (dist <= 1e-5) ? 0.0 : dir * std::min(v_limit, v_brake);

        last_velocity_ += std::clamp(v_des - last_velocity_, -max_dv, max_dv);
        *control_ = last_velocity_;
    }

private:
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    pid::SmartInput measurement_;
    pid::SmartInput setpoint_;
    pid::SmartInput passthrough_;
    pid::SmartInput velocity_limit_;

    InputInterface<double> update_rate_;
    OutputInterface<double> control_;

    double max_velocity_{10.0};
    double max_acceleration_{20.0};
    double fallback_dt_{0.001};
    double last_velocity_{0.0};
};

} // namespace rmcs_core::controller::planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::planner::TrapezoidPlanner, rmcs_executor::Component)
