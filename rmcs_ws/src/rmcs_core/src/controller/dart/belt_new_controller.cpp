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
// BeltNewController
//   Board Lite 新镖架同步带的两台电机执行器。只做两件事：
//
//   ① 把手动摇杆映射成两台电机的目标速度（档位由 kManual 说明书声明，
//      经 ManualDispatcher 分发，不含任何业务判断）。
//   ② 左右同步 —— 两侧不共轴，位置差会累积并拽坏机械，
//      因此把「两台电机」对外表现为「一个执行器」。
//
//   除此之外不判断堵转、不跑归零、不做位置闭环、不含超时。
//   这些是决策，属于上位机任务层：上位机读 /dart/belt/*/velocity 与 /torque
//   自行判断，再写控制目标接口。
//
//   同步公式（交叉耦合，两侧各担一半，总速度守恒）：
//     err  = angle_L - angle_R
//     corr = clamp(sync_gain * err, ±sync_correction_max)
//     l_vel -= corr / 2
//     r_vel += corr / 2
// ─────────────────────────────────────────────────────────────────────────────
class BeltNewController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    BeltNewController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/dart/belt/left_motor/angle", left_angle_, false);
        register_input("/dart/belt/right_motor/angle", right_angle_, false);
        register_input("/dart/belt/left_motor/velocity", left_velocity_, false);
        register_input("/dart/belt/right_motor/velocity", right_velocity_, false);
        register_input("/dart/belt/left_motor/torque", left_torque_, false);
        register_input("/dart/belt/right_motor/torque", right_torque_, false);

        register_output("/dart/belt/left_motor/control_velocity", left_control_velocity_, 0.0);
        register_output("/dart/belt/right_motor/control_velocity", right_control_velocity_, 0.0);
        register_output(
            "/dart/belt/left_motor/control_torque_limit", left_control_torque_limit_, 0.0);
        register_output(
            "/dart/belt/right_motor/control_torque_limit", right_control_torque_limit_, 0.0);
        register_output("/dart/belt/left_motor/calibrate_zero", left_calibrate_, false);
        register_output("/dart/belt/right_motor/calibrate_zero", right_calibrate_, false);

        // 上位机直写目标（经 ManualDispatcher 之外的通路）
        register_input("/dart/belt/left_motor/cmd_velocity", cmd_l_velocity_, false);
        register_input("/dart/belt/right_motor/cmd_velocity", cmd_r_velocity_, false);
        register_input("/dart/belt/left_motor/cmd_torque_limit", cmd_l_torque_limit_, false);
        register_input("/dart/belt/right_motor/cmd_torque_limit", cmd_r_torque_limit_, false);
        register_input("/dart/belt/left_motor/calibrate_request", cmd_l_calibrate_, false);
        register_input("/dart/belt/right_motor/calibrate_request", cmd_r_calibrate_, false);

        load_parameters();
        // 调试模式占左摇杆 X+Y（左右电机各一轴），正常模式只占左摇杆 Y。
        // 两种模式用同一个拨杆档位，不额外占键位。
        ManualRegistry::add(debug_mode_ ? &kManualDebug : &kManual, this);
    }

    BeltNewController(const BeltNewController&) = delete;
    BeltNewController& operator=(const BeltNewController&) = delete;
    BeltNewController(BeltNewController&&) = delete;
    BeltNewController& operator=(BeltNewController&&) = delete;

    ~BeltNewController() override = default;

    void before_updating() override {
        bind_or(left_angle_, 0.0, "/dart/belt/left_motor/angle");
        bind_or(right_angle_, 0.0, "/dart/belt/right_motor/angle");
        bind_or(left_velocity_, 0.0, "/dart/belt/left_motor/velocity");
        bind_or(right_velocity_, 0.0, "/dart/belt/right_motor/velocity");
        bind_or(left_torque_, 0.0, "/dart/belt/left_motor/torque");
        bind_or(right_torque_, 0.0, "/dart/belt/right_motor/torque");

        bind_or(cmd_l_velocity_, 0.0, "/dart/belt/left_motor/cmd_velocity");
        bind_or(cmd_r_velocity_, 0.0, "/dart/belt/right_motor/cmd_velocity");
        bind_or(cmd_l_torque_limit_, 0.0, "/dart/belt/left_motor/cmd_torque_limit");
        bind_or(cmd_r_torque_limit_, 0.0, "/dart/belt/right_motor/cmd_torque_limit");
        bind_or(cmd_l_calibrate_, false, "/dart/belt/left_motor/calibrate_request");
        bind_or(cmd_r_calibrate_, false, "/dart/belt/right_motor/calibrate_request");
    }

    // 每 tick 一律把「当前目标」转发给电机。手动/自动由 ManualDispatcher 决定谁在写目标。
    // manual_active_ 每 tick 复位，run_manual 命中档位时才置位 —— 松杆离档立刻回到上位机目标。
    void update() override {
        forward_targets();
        manual_active_ = false;
    }

    static void on_manual(void* self, const ManualBinding& binding, const StickSnapshot& sticks) {
        static_cast<BeltNewController*>(self)->run_manual(binding, sticks);
    }

    static void on_auto(void* self) { (void)self; }

private:
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
        debug_torque_limit_ = d("debug_torque_limit");

        sensitivity_forward_ = d("sensitivity_forward");
        sensitivity_reverse_ = d("sensitivity_reverse");
        deadzone_ = std::abs(d("deadzone"));
        manual_torque_limit_ = d("manual_torque_limit");

        sync_gain_ = d("sync_gain");
        sync_correction_max_ = d("sync_correction_max");
        sync_err_limit_ = d("sync_err_limit");
        sync_err_fault_ticks_ = static_cast<int>(get_parameter("sync_err_fault_ticks").as_int());
    }

    // ── 把目标转发给两台电机 ──────────────────────────────────────────────────
    // 上位机通路：直接读 cmd_* 输入并转发（本类不保存状态）。
    // 只有手动拨杆接管时，才用摇杆值覆盖。
    void forward_targets() {
        double l = cmd_l_velocity_.ready() ? *cmd_l_velocity_ : 0.0;
        double r = cmd_r_velocity_.ready() ? *cmd_r_velocity_ : 0.0;
        double l_limit = cmd_l_torque_limit_.ready() ? *cmd_l_torque_limit_ : 0.0;
        double r_limit = cmd_r_torque_limit_.ready() ? *cmd_r_torque_limit_ : 0.0;
        bool l_cal = cmd_l_calibrate_.ready() && *cmd_l_calibrate_;
        bool r_cal = cmd_r_calibrate_.ready() && *cmd_r_calibrate_;

        // 上一 tick 手动写过目标且摇杆仍在本档位 → 摇杆值优先
        if (manual_active_) {
            l = manual_l_velocity_;
            r = manual_r_velocity_;
            l_limit = r_limit = manual_torque_limit_cmd_;
            l_cal = r_cal = false;
        }

        if (!debug_mode_)
            apply_sync(l, r);

        *left_control_velocity_ = l;
        *right_control_velocity_ = r;
        *left_control_torque_limit_ = l_limit;
        *right_control_torque_limit_ = r_limit;
        *left_calibrate_ = l_cal;
        *right_calibrate_ = r_cal;
    }

    // ── 手动：摇杆直驱 ────────────────────────────────────────────────────────
    // 只记录摇杆换算结果，实际输出由 forward_targets 统一产生。
    void run_manual(const ManualBinding& binding, const StickSnapshot& sticks) {
        manual_active_ = true;

        if (debug_mode_) {
            const auto to_velocity = [this](double v) {
                const double n = deadzone_normalize(v, deadzone_);
                return n * (n >= 0.0 ? sensitivity_forward_ : sensitivity_reverse_);
            };
            manual_l_velocity_ = to_velocity(sticks.left.y());
            manual_r_velocity_ = to_velocity(sticks.left.x());
            manual_torque_limit_cmd_ = debug_torque_limit_;
            return;
        }

        const double raw = ManualRegistry::axis_value(binding, sticks);
        const double normalized = deadzone_normalize(raw, deadzone_);
        const double sensitivity =
            normalized >= 0.0 ? sensitivity_forward_ : sensitivity_reverse_;

        manual_l_velocity_ = normalized * sensitivity;
        manual_r_velocity_ = manual_l_velocity_;
        manual_torque_limit_cmd_ = manual_torque_limit_;
    }
    // ── 同步：交叉耦合，各担一半，总速度守恒 ──────────────────────────────────
    void apply_sync(double& l_vel, double& r_vel) {
        if (sync_gain_ <= 0.0)
            return;
        if (!left_angle_.ready() || !right_angle_.ready())
            return;

        const double err = *left_angle_ - *right_angle_;
        const double corr =
            std::clamp(sync_gain_ * err, -sync_correction_max_, sync_correction_max_);

        l_vel -= corr * 0.5;
        r_vel += corr * 0.5;

        // 同步误差持续超限说明打滑或单侧卡死，停车保护机械
        if (std::abs(err) > sync_err_limit_) {
            if (++sync_err_ticks_ >= sync_err_fault_ticks_) {
                l_vel = 0.0;
                r_vel = 0.0;
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "[BeltNew] SYNC FAULT: |err|=%.3f > %.3f for %d ticks, stopping",
                    std::abs(err), sync_err_limit_, sync_err_ticks_);
            }
        } else {
            sync_err_ticks_ = 0;
        }
    }

    // ── 说明书 ────────────────────────────────────────────────────────────────
    // 左拨杆 UP + 右拨杆 MIDDLE → 手动；输入取左摇杆 Y 轴。
    // 与 carriage 同档，左摇杆归 belt、右摇杆归 carriage。
    static constexpr ManualOps kOps{&BeltNewController::on_manual,
                                    &BeltNewController::on_auto};

    static constexpr ManualBinding kManual{
        .name = "belt",
        .left = rmcs_msgs::Switch::UP,
        .right = rmcs_msgs::Switch::MIDDLE,
        .side = StickSide::kLeft,
        .axis = StickAxis::kY,
        .ops = &kOps,
    };

    // 调试档位：同一拨杆组合，但左摇杆 Y/X 分别驱动左右电机。
    static constexpr ManualBinding kManualDebug{
        .name = "belt-dbg(Ly=L,Lx=R)",
        .left = rmcs_msgs::Switch::UP,
        .right = rmcs_msgs::Switch::MIDDLE,
        .side = StickSide::kLeft,
        .axis = StickAxis::kY,
        .ops = &kOps,
    };

    // 反馈
    InputInterface<double> left_angle_;
    InputInterface<double> right_angle_;
    InputInterface<double> left_velocity_;
    InputInterface<double> right_velocity_;
    InputInterface<double> left_torque_;
    InputInterface<double> right_torque_;

    // 上位机目标输入
    InputInterface<double> cmd_l_velocity_;
    InputInterface<double> cmd_r_velocity_;
    InputInterface<double> cmd_l_torque_limit_;
    InputInterface<double> cmd_r_torque_limit_;
    InputInterface<bool> cmd_l_calibrate_;
    InputInterface<bool> cmd_r_calibrate_;

    // 手动结果（run_manual 写，forward_targets 读）
    bool manual_active_{false};
    double manual_l_velocity_{0.0};
    double manual_r_velocity_{0.0};
    double manual_torque_limit_cmd_{0.0};

    // 电机指令
    OutputInterface<double> left_control_velocity_;
    OutputInterface<double> right_control_velocity_;
    OutputInterface<double> left_control_torque_limit_;
    OutputInterface<double> right_control_torque_limit_;
    OutputInterface<bool> left_calibrate_;
    OutputInterface<bool> right_calibrate_;

    // 手动
    double sensitivity_forward_{};
    double sensitivity_reverse_{};
    double deadzone_{};
    double manual_torque_limit_{};

    // 调试模式
    bool debug_mode_{false};
    double debug_torque_limit_{};

    // 同步
    double sync_gain_{};
    double sync_correction_max_{};
    double sync_err_limit_{};
    int sync_err_fault_ticks_{};
    int sync_err_ticks_{};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::BeltNewController, rmcs_executor::Component)


