#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace rmcs_core::controller::planner {

// ─────────────────────────────────────────────────────────────────────────────
// SCurveTrajectory
//   7 段式加加速度受限（S 曲线）位置-速度规划器（纯数学实现，无 ROS 依赖）。
//
//   功能：
//     - 给定起点 x0、终点 x1 与约束 (v_max, a_max, j_max)，按解析式生成
//       C² 连续（位置、速度、加速度均连续）的 7 段 S 曲线。
//     - 自动处理达不到 v_max / 达不到 a_max 的短行程退化。
//     - 配合位置外环 P 纠偏：v_cmd = v_ref + pos_kp * (x_ref - x_meas)，
//       既保留 S 曲线平滑特性，又抑制弹簧变负载带来的跟踪滞后。
//     - 运动中途若目标变化，先在 j_max / a_max 约束下平滑刹停，再无缝重规划到新目标。
// ─────────────────────────────────────────────────────────────────────────────
class SCurveTrajectory {
public:
    struct Config {
        double max_velocity{10.0};
        double max_acceleration{20.0};
        double max_jerk{100.0};
        double position_kp{0.0};
    };

    struct State {
        double position{0.0};
        double velocity{0.0};
        double acceleration{0.0};
        bool finished{true};
    };

    SCurveTrajectory() = default;
    explicit SCurveTrajectory(const Config& config) { set_config(config); }

    void set_config(const Config& config) {
        cfg_.max_velocity = std::max(std::abs(config.max_velocity), kEps);
        cfg_.max_acceleration = std::max(std::abs(config.max_acceleration), kEps);
        cfg_.max_jerk = std::max(std::abs(config.max_jerk), kEps);
        cfg_.position_kp = std::max(config.position_kp, 0.0);
    }

    const Config& config() const { return cfg_; }
    const State& state() const { return state_; }
    bool active() const { return active_; }

    void reset() {
        active_ = false;
        braking_ = false;
        elapsed_ = 0.0;
        total_time_ = 0.0;
        planned_v_max_ = 0.0;
        state_ = State{};
    }

    // 推进一个步长 dt，返回用于控制的速度指令（含位置外环纠偏）
    double update(double measurement, double setpoint, double dt, double dynamic_v_limit = std::numeric_limits<double>::infinity()) {
        if (!std::isfinite(measurement) || !std::isfinite(setpoint) || !(dt > 0.0)) {
            reset();
            return std::numeric_limits<double>::quiet_NaN();
        }

        const double effective_v_max = std::isfinite(dynamic_v_limit)
            ? std::max(std::min(cfg_.max_velocity, std::abs(dynamic_v_limit)), kEps)
            : cfg_.max_velocity;

        if (!active_) {
            active_ = true;
            braking_ = false;
            state_ = State{measurement, 0.0, 0.0, false};
            plan_from_rest(measurement, setpoint, effective_v_max);
        } else if (std::abs(setpoint - target_) > kTargetChangeEps
                   || (std::abs(effective_v_max - planned_v_max_) > kVelLimitChangeEps
                       && !state_.finished)) {
            if (std::abs(state_.velocity) <= kStopEps && std::abs(state_.acceleration) <= kStopEps) {
                state_.velocity = 0.0;
                state_.acceleration = 0.0;
                braking_ = false;
                plan_from_rest(state_.position, setpoint, effective_v_max);
            } else {
                // 运动中途改目标/改限速：先在加速度与加加速度约束内平滑刹停，再重规划
                braking_ = true;
                pending_target_ = setpoint;
                pending_v_max_ = effective_v_max;
            }
        }

        if (braking_) {
            step_brake(dt);
        } else if (!state_.finished) {
            elapsed_ += dt;
            if (elapsed_ >= total_time_) {
                state_.position = target_;
                state_.velocity = 0.0;
                state_.acceleration = 0.0;
                state_.finished = true;
            } else {
                evaluate(elapsed_);
            }
        }

        return state_.velocity + cfg_.position_kp * (state_.position - measurement);
    }

private:
    struct Segment {
        double duration{0.0};
        double t_start{0.0};
        double x0{0.0};
        double v0{0.0};
        double a0{0.0};
        double jerk{0.0};
    };

    static constexpr double kEps = 1e-6;
    static constexpr double kTargetChangeEps = 1e-5;
    static constexpr double kVelLimitChangeEps = 1e-3;
    static constexpr double kStopEps = 1e-4;

    void plan_from_rest(double x0, double x1, double v_max) {
        target_ = x1;
        planned_v_max_ = v_max;
        elapsed_ = 0.0;

        const double diff = x1 - x0;
        const double dist = std::abs(diff);
        if (dist <= kTargetChangeEps) {
            total_time_ = 0.0;
            state_ = State{x1, 0.0, 0.0, true};
            return;
        }

        const double dir = diff >= 0.0 ? 1.0 : -1.0;
        const double a_max = cfg_.max_acceleration;
        const double j_max = cfg_.max_jerk;

        double tj1 = 0.0;
        double ta = 0.0;
        double tv = 0.0;

        // 1) 先按达到 v_max 试算加速时间 Ta_v 与加减速总位移 D_acc_dec
        const double v_crit = (a_max * a_max) / j_max;
        double tj1_v = 0.0;
        double ta_v = 0.0;
        if (v_max >= v_crit) {
            tj1_v = a_max / j_max;
            ta_v = tj1_v + v_max / a_max;
        } else {
            tj1_v = std::sqrt(v_max / j_max);
            ta_v = 2.0 * tj1_v;
        }
        const double d_acc_dec = v_max * ta_v;

        if (dist >= d_acc_dec) {
            tj1 = tj1_v;
            ta = ta_v;
            tv = (dist - d_acc_dec) / v_max;
        } else {
            // 2) 短行程：达不到 v_max，Tv = 0，再判是否能达到 a_max
            const double d_reach_a = 2.0 * (a_max * a_max * a_max) / (j_max * j_max);
            if (dist >= d_reach_a) {
                tj1 = a_max / j_max;
                ta = 0.5 * (tj1 + std::sqrt(tj1 * tj1 + 4.0 * dist / a_max));
            } else {
                tj1 = std::cbrt(dist / (2.0 * j_max));
                ta = 2.0 * tj1;
            }
        }

        const double t_const_a = std::max(0.0, ta - 2.0 * tj1);
        const std::array<double, 7> durations{tj1, t_const_a, tj1, tv, tj1, t_const_a, tj1};
        const std::array<double, 7> jerks{
            dir * j_max, 0.0, -dir * j_max, 0.0, -dir * j_max, 0.0, dir * j_max};

        double t_acc = 0.0;
        double x = x0;
        double v = 0.0;
        double a = 0.0;
        for (std::size_t i = 0; i < 7; ++i) {
            const double T = durations[i];
            const double j = jerks[i];
            segments_[i] = Segment{T, t_acc, x, v, a, j};
            t_acc += T;
            x += v * T + 0.5 * a * T * T + (1.0 / 6.0) * j * T * T * T;
            v += a * T + 0.5 * j * T * T;
            a += j * T;
        }

        total_time_ = t_acc;
        state_ = State{x0, 0.0, 0.0, false};
    }

    void evaluate(double t) {
        const Segment* active_seg = &segments_[6];
        for (const auto& seg : segments_) {
            if (t <= seg.t_start + seg.duration) {
                active_seg = &seg;
                break;
            }
        }
        const double tau = std::max(0.0, t - active_seg->t_start);
        state_.position = active_seg->x0 + active_seg->v0 * tau
                        + 0.5 * active_seg->a0 * tau * tau
                        + (1.0 / 6.0) * active_seg->jerk * tau * tau * tau;
        state_.velocity = active_seg->v0 + active_seg->a0 * tau
                        + 0.5 * active_seg->jerk * tau * tau;
        state_.acceleration = active_seg->a0 + active_seg->jerk * tau;
        state_.finished = false;
    }

    // 运动中途目标突变时，在 (a_max, j_max) 约束下平稳刹停后再重规划
    void step_brake(double dt) {
        const double j_max = cfg_.max_jerk;
        const double a_max = cfg_.max_acceleration;
        const double v = state_.velocity;
        const double a = state_.acceleration;
        const double max_da = j_max * dt;

        // 若加速度只差一步就能归零、且速度也接近 0，直接平稳落到 0 并重规划
        if (std::abs(a) <= max_da + kEps && std::abs(v) <= 0.5 * std::abs(a) * dt + max_da * dt) {
            state_.position += 0.5 * v * dt;
            state_.velocity = 0.0;
            state_.acceleration = 0.0;
            braking_ = false;
            plan_from_rest(state_.position, pending_target_, pending_v_max_);
            return;
        }

        // 从当前加速度按 j_max 斜坡归零所需的速度变化量
        const double dv_ramp = (v >= 0.0 && a < 0.0) || (v <= 0.0 && a > 0.0)
                                 ? 0.5 * a * std::abs(a) / j_max
                                 : 0.0;
        const double v_stop_est = v + dv_ramp;

        double a_desired = 0.0;
        if (v > 0.0 && v_stop_est > 0.0) {
            a_desired = -a_max;
        } else if (v < 0.0 && v_stop_est < 0.0) {
            a_desired = a_max;
        }

        const double a_next = a + std::clamp(a_desired - a, -max_da, max_da);
        const double v_next = v + 0.5 * (a + a_next) * dt;
        const double x_next = state_.position + 0.5 * (v + v_next) * dt;

        state_.position = x_next;
        state_.velocity = v_next;
        state_.acceleration = a_next;
        state_.finished = false;
    }

    Config cfg_{};
    State state_{};
    std::array<Segment, 7> segments_{};

    bool active_{false};
    bool braking_{false};
    double target_{0.0};
    double planned_v_max_{0.0};
    double pending_target_{0.0};
    double pending_v_max_{0.0};
    double elapsed_{0.0};
    double total_time_{0.0};
};

} // namespace rmcs_core::controller::planner
