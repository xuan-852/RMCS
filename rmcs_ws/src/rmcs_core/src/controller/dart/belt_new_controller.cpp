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
//
//   ── 驱动方式（yaml: sync_mode）───────────────────────────────────────────
//     sync_mode: "same"   两侧同速（老镖架行为，默认）
//       —— 摇杆值原样给两台电机，各自跑自己的速度环，互不干涉。
//          漂移靠「每次 dart-init 向上归零」重置，不在此处纠正。
//
//     sync_mode: "sync"   同步闭环（交叉耦合修正）
//       —— 按 sync_gain 把位置差折算成速度修正，另含超限停车保护。
//
//     sync_mode: "split"  左右独立（调试用，左摇杆 Y=左电机，X=右电机）
//
//   NaN 透传：上位机写 NaN 表示「放手」（见 NewBeltResource::release()）。
//   本控制器不拦截 NaN —— NaN 经 PID 变成 NaN，DJI 电调收到 NaN 输出零电流，
//   弹簧把滑块顶回顶部零点。这是故障安全态，不是异常。
//   本机构被弹簧顶住，没有重力自由下落这回事。
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
        register_output("/dart/belt/stalled", stalled_, false);

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
        ManualRegistry::add(&kManual, this);
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
        double l = cmd_l_velocity_.ready() ? *cmd_l_velocity_ : 0.0;
        double r = cmd_r_velocity_.ready() ? *cmd_r_velocity_ : 0.0;
        double l_limit = cmd_l_torque_limit_.ready() ? *cmd_l_torque_limit_ : 0.0;
        double r_limit = cmd_r_torque_limit_.ready() ? *cmd_r_torque_limit_ : 0.0;
        bool l_cal = cmd_l_calibrate_.ready() && *cmd_l_calibrate_;
        bool r_cal = cmd_r_calibrate_.ready() && *cmd_r_calibrate_;

        if (manual_active_) {
            l = manual_l_velocity_;
            r = manual_r_velocity_;
            l_limit = r_limit = manual_torque_limit_cmd_;
            l_cal = r_cal = false;
        }
        manual_active_ = false;

        // 同步模式下叠加修正与保护；same / split 直接原样输出
        if (sync_mode_ == SyncMode::kSync)
            apply_sync(l, r);

        *left_control_velocity_ = l;
        *right_control_velocity_ = r;
        *left_control_torque_limit_ = l_limit;
        *right_control_torque_limit_ = r_limit;
        *left_calibrate_ = l_cal;
        *right_calibrate_ = r_cal;
        *stalled_ = update_stall_detection(l_limit, r_limit);
    }

    static void on_manual(void* self, const ManualBinding& binding, const StickSnapshot& sticks) {
        static_cast<BeltNewController*>(self)->run_manual(binding, sticks);
    }

    static void on_auto(void* self) { (void)self; }

private:
    // 手动档的驱动方式（启动时由 yaml 参数 sync_mode 决定）
    enum class SyncMode { kSame, kSync, kSplit };

    template <typename T, typename V>
    void bind_or(InputInterface<T>& interface, V&& value, const char* name) {
        if (interface.ready())
            return;
        interface.make_and_bind_directly(T(std::forward<V>(value)));
        RCLCPP_WARN(get_logger(), "Failed to fetch \"%s\". Set to default.", name);
    }

    void load_parameters() {
        sensitivity_forward_ = get_parameter("sensitivity_forward").as_double();
        sensitivity_reverse_ = get_parameter("sensitivity_reverse").as_double();
        deadzone_ = std::abs(get_parameter("deadzone").as_double());
        manual_torque_limit_ = get_parameter("manual_torque_limit").as_double();

        sync_gain_ = get_parameter("sync_gain").as_double();
        sync_correction_max_ = get_parameter("sync_correction_max").as_double();
        sync_err_limit_ = get_parameter("sync_err_limit").as_double();
        sync_err_fault_ticks_ = static_cast<int>(get_parameter("sync_err_fault_ticks").as_int());

        debug_torque_limit_ = get_parameter("debug_torque_limit").as_double();
        apply_sync_mode(get_parameter("sync_mode").as_string());

        get_parameter_or("stall_velocity_threshold", stall_velocity_threshold_, 0.05);
        get_parameter_or("stall_torque_ratio", stall_torque_ratio_, 0.8);
        get_parameter_or("stall_min_torque", stall_min_torque_, 0.3);
        int64_t stall_ticks = 50;
        get_parameter_or("stall_confirm_ticks", stall_ticks, int64_t{50});
        stall_confirm_ticks_ = static_cast<int>(std::max<int64_t>(1, stall_ticks));
    }

    // ── sync_mode 解析 ────────────────────────────────────────────────────────
    // "same"  两侧同速（老镖架行为）
    // "sync"  同步闭环
    // "split" 左右独立（调试）
    void apply_sync_mode(const std::string& mode) {
        sync_mode_ = SyncMode::kSame;  // 未知值一律退化为 same，不静默进入危险模式
        if (mode == "sync")
            sync_mode_ = SyncMode::kSync;
        else if (mode == "split")
            sync_mode_ = SyncMode::kSplit;
        else if (mode != "same")
            RCLCPP_ERROR(
                get_logger(), "[BeltNew] unknown sync_mode \"%s\", falling back to \"same\"",
                mode.c_str());

        RCLCPP_INFO(
            get_logger(), "[BeltNew] sync_mode = %s %s", mode.c_str(),
            sync_mode_ == SyncMode::kSync
                ? "(sync loop ON)"
                : (sync_mode_ == SyncMode::kSplit ? "(split: Ly=left, Lx=right)"
                                                  : "(both sides same speed)"));
    }

    // ── 手动：摇杆直驱 ────────────────────────────────────────────────────────
    // 只记录摇杆换算结果，实际输出由 update() 统一产生。
    void run_manual(const ManualBinding& binding, const StickSnapshot& sticks) {
        manual_active_ = true;

        // split：左摇杆 Y 驱动左电机、X 驱动右电机，方便单独验证某一侧
        if (sync_mode_ == SyncMode::kSplit) {
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

    // ── 物理堵转检测 ──────────────────────────────────────────────────────────
    // 任一侧速度掉到阈值以下且力矩顶到当前限幅的比例，持续足够 tick 即报堵转。
    // 力矩限幅为 NaN（放手）或 0 时直接清零，不误报。
    bool update_stall_detection(double l_limit, double r_limit) {
        const double active_limit = std::max(std::abs(l_limit), std::abs(r_limit));
        if (!std::isfinite(active_limit) || active_limit <= 1e-6) {
            stall_ticks_ = 0;
            return false;
        }

        const double max_vel = std::max(std::abs(*left_velocity_), std::abs(*right_velocity_));
        const double max_tq = std::max(std::abs(*left_torque_), std::abs(*right_torque_));
        const double tq_threshold = std::max(stall_min_torque_, active_limit * stall_torque_ratio_);

        if (max_vel < stall_velocity_threshold_ && max_tq > tq_threshold) {
            if (stall_ticks_ < stall_confirm_ticks_)
                ++stall_ticks_;
        } else {
            stall_ticks_ = 0;
        }
        return stall_ticks_ >= stall_confirm_ticks_;
    }

    // ── 说明书 ────────────────────────────────────────────────────────────────
    // 左拨杆 UP + 右拨杆 MIDDLE → 手动；输入取左摇杆 Y 轴。
    // 与 carriage 同档，左摇杆归 belt、右摇杆归 carriage。
    static constexpr ManualOps kOps{&BeltNewController::on_manual,
                                    &BeltNewController::on_auto};

    // 手动档只有一个拨杆组合，具体驱动方式由 sync_mode 决定。
    static constexpr ManualBinding kManual{
        .name = "belt",
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
    OutputInterface<bool> stalled_;

    // 手动
    double sensitivity_forward_{};
    double sensitivity_reverse_{};
    double deadzone_{};
    double manual_torque_limit_{};

    // 驱动方式（yaml: sync_mode）+ split 模式的独立力矩上限
    SyncMode sync_mode_{SyncMode::kSame};
    double debug_torque_limit_{};

    // 同步
    double sync_gain_{};
    double sync_correction_max_{};
    double sync_err_limit_{};
    int sync_err_fault_ticks_{};
    int sync_err_ticks_{};

    // 堵转检测
    double stall_velocity_threshold_{0.05};
    double stall_torque_ratio_{0.8};
    double stall_min_torque_{0.3};
    int stall_confirm_ticks_{50};
    int stall_ticks_{0};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::BeltNewController, rmcs_executor::Component)


