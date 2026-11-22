#include <cmath>
#include <limits>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/smart_input.hpp"
#include "controller/planner/s_curve.hpp"

namespace rmcs_core::controller::planner {

// ─────────────────────────────────────────────────────────────────────────────
// SCurvePlanner
//   通用 7 段 S 曲线轨迹规划组件（与 PidController 同构，不含具体机构业务）。
//
//   接口约定（全部在 YAML 中配置）：
//     measurement     当前实际位置
//     setpoint        目标位置（NaN 时复位内部状态并输出 NaN）
//     control         输出给下级速度环/执行器的速度指令
//     passthrough     [可选，默认 NaN] 有限值时复位规划器并直接透传（堵转归零用）
//     velocity_limit  [可选，默认 +inf] 动态速度上限（与 max_velocity 取较小值）
// ─────────────────────────────────────────────────────────────────────────────
class SCurvePlanner
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SCurvePlanner()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , measurement_(*this, "measurement")
        , setpoint_(*this, "setpoint")
        , passthrough_(*this, "passthrough", std::numeric_limits<double>::quiet_NaN())
        , velocity_limit_(*this, "velocity_limit", std::numeric_limits<double>::infinity()) {

        register_input("/predefined/update_rate", update_rate_, false);
        register_output(get_parameter("control").as_string(), control_, kNan);

        SCurveTrajectory::Config cfg;
        cfg.max_velocity = get_parameter("max_velocity").as_double();
        cfg.max_acceleration = get_parameter("max_acceleration").as_double();
        cfg.max_jerk = get_parameter("max_jerk").as_double();
        get_parameter("position_kp", cfg.position_kp);
        trajectory_.set_config(cfg);

        get_parameter("dt", fallback_dt_);
    }

    void update() override {
        const double dt = (update_rate_.ready() && *update_rate_ > 0.0)
                            ? (1.0 / *update_rate_)
                            : fallback_dt_;

        const double passthrough = *passthrough_;
        if (std::isfinite(passthrough)) {
            trajectory_.reset();
            *control_ = passthrough;
            return;
        }

        *control_ = trajectory_.update(*measurement_, *setpoint_, dt, *velocity_limit_);
    }

private:
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    pid::SmartInput measurement_;
    pid::SmartInput setpoint_;
    pid::SmartInput passthrough_;
    pid::SmartInput velocity_limit_;

    InputInterface<double> update_rate_;
    OutputInterface<double> control_;

    SCurveTrajectory trajectory_;
    double fallback_dt_{0.001};
};

} // namespace rmcs_core::controller::planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::planner::SCurvePlanner, rmcs_executor::Component)
