#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/dart/manual_dispatch.hpp"

namespace rmcs_core::controller::dart {

// ─────────────────────────────────────────────────────────────────────────────
// BeltNewController
//   Board Pro 新镖架同步带双电机执行器：
//     1. 把手动摇杆映射为目标速度与力矩限幅（经 ManualDispatcher 分发）；
//     2. 支持 same / sync（交叉耦合 + 虚拟零点对齐）/ split 三种驱动模式；
//     3. 基于速度与力矩进行物理堵转检测，输出 /dart/belt/stalled；
//     4. 原样透传上位机下发的 NaN（「放手」语义，由弹簧回顶零点）。
// ─────────────────────────────────────────────────────────────────────────────
class BeltNewController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    BeltNewController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        // 反馈输入
        register_input("/dart/belt/left_motor/angle", left_angle_, false);
        register_input("/dart/belt/right_motor/angle", right_angle_, false);
        register_input("/dart/belt/left_motor/velocity", left_velocity_, false);
        register_input("/dart/belt/right_motor/velocity", right_velocity_, false);
        register_input("/dart/belt/left_motor/torque", left_torque_, false);
        register_input("/dart/belt/right_motor/torque", right_torque_, false);

        // 上位机目标输入
        register_input("/dart/belt/left_motor/cmd_velocity", cmd_l_velocity_, false);
        register_input("/dart/belt/right_motor/cmd_velocity", cmd_r_velocity_, false);
        register_input("/dart/belt/left_motor/cmd_torque_limit", cmd_l_torque_limit_, false);
        register_input("/dart/belt/right_motor/cmd_torque_limit", cmd_r_torque_limit_, false);
        register_input("/dart/belt/left_motor/calibrate_request", cmd_l_calibrate_, false);
        register_input("/dart/belt/right_motor/calibrate_request", cmd_r_calibrate_, false);

        // 电机指令与状态输出
        register_output("/dart/belt/left_motor/control_velocity", left_control_velocity_, 0.0);
        register_output("/dart/belt/right_motor/control_velocity", right_control_velocity_, 0.0);
        register_output(
            "/dart/belt/left_motor/control_torque_limit", left_control_torque_limit_, 0.0);
        register_output(
            "/dart/belt/right_motor/control_torque_limit", right_control_torque_limit_, 0.0);
        register_output("/dart/belt/stalled", stalled_, false);

        load_parameters();
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

    // 手动档位命中时用摇杆值，否则转发上位机目标；离档立即恢复上位机控制。
    void update() override {
        double l_velocity_ = *cmd_l_velocity_;
        double r_velocity_ = *cmd_r_velocity_;
        double l_torque_limit_ = *cmd_l_torque_limit_;
        double r_torque_limit_ = *cmd_r_torque_limit_;

        if (manual_active_) {
            l_velocity_ = manual_l_velocity_;
            r_velocity_ = manual_r_velocity_;
            l_torque_limit_ = r_torque_limit_ = manual_torque_limit_cmd_;
        } else {
            // 软件虚拟零点：上升沿记录当前多圈角度，不修改底层硬件编码器状态
            update_zero_offset(*cmd_l_calibrate_, *left_angle_, left_calibrate_latched_, left_zero_offset_);
            update_zero_offset(*cmd_r_calibrate_, *right_angle_, right_calibrate_latched_, right_zero_offset_);
        }
        manual_active_ = false;

        if (sync_mode_ == SyncMode::kSync)
            apply_sync(l_velocity_, r_velocity_);

        *left_control_velocity_ = l_velocity_;
        *right_control_velocity_ = r_velocity_;
        *left_control_torque_limit_ = l_torque_limit_;
        *right_control_torque_limit_ = r_torque_limit_;
        *stalled_ = update_stall_detection(l_torque_limit_, r_torque_limit_);
    }

    // ── ManualRegistry 的两个动作 ─────────────────────────────────────────────
    static void on_manual(void* self, const ManualBinding& binding, const StickSnapshot& sticks) {
        static_cast<BeltNewController*>(self)->run_manual(binding, sticks);
    }

    static void on_auto(void* self) { (void)self; }

private:
    enum class SyncMode { kSame, kSync, kSplit };//yaml里面配置的三套手动控制枚举

    template <typename T, typename V>

    void bind_or(InputInterface<T>& interface, V&& value, const char* name) {
        if (interface.ready())
            return;
        interface.make_and_bind_directly(T(std::forward<V>(value)));
        RCLCPP_WARN(get_logger(), "Failed to fetch \"%s\". Set to default.", name);
    }

    static void update_zero_offset(
        bool request, double angle, bool& latched, double& zero_offset) {
        if (request && !latched)
            zero_offset = angle;
        latched = request;
    }

    void load_parameters() {
        manual_velocity_sensitivity_ = get_parameter("manual_velocity_sensitivity").as_double();
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

    void apply_sync_mode(const std::string& mode) {
        if (mode == "sync") {
            sync_mode_ = SyncMode::kSync;
        } else if (mode == "split") {
            sync_mode_ = SyncMode::kSplit;
        } else {
            sync_mode_ = SyncMode::kSame;
            if (mode != "same") {
                RCLCPP_ERROR(
                    get_logger(), "[BeltNew] unknown sync_mode \"%s\", falling back to \"same\"",
                    mode.c_str());
            }
        }

        const char* desc = sync_mode_ == SyncMode::kSync
                             ? "sync (sync loop ON)"
                             : (sync_mode_ == SyncMode::kSplit ? "split (Ly=left, Lx=right)"
                                                               : "same (both sides same speed)");
        RCLCPP_INFO(get_logger(), "[BeltNew] sync_mode = %s", desc);
    }

    // ── 手动：摇杆直驱 ────────────────────────────────────────────────────────
    void run_manual(const ManualBinding& binding, const StickSnapshot& sticks) {
        manual_active_ = true;

        if (sync_mode_ == SyncMode::kSplit) {
            manual_l_velocity_ = manual_velocity_sensitivity_ * sticks.left.y();
            manual_r_velocity_ = manual_velocity_sensitivity_ * sticks.left.x();
            manual_torque_limit_cmd_ = debug_torque_limit_;
            return;
        }

        manual_l_velocity_ =
            manual_velocity_sensitivity_ * ManualRegistry::axis_value(binding, sticks);
        manual_r_velocity_ = manual_l_velocity_;
        manual_torque_limit_cmd_ = manual_torque_limit_;
    }

    // ── 同步：交叉耦合，以虚拟零点对齐 ────────────────────────────────────────
    void apply_sync(double& l_vel, double& r_vel) {//以归零的零点进行左右差位调速
        if (sync_gain_ <= 0.0)
            return;

        const double err =
            (*left_angle_ - left_zero_offset_) - (*right_angle_ - right_zero_offset_);
        const double corr =
            std::clamp(sync_gain_ * err, -sync_correction_max_, sync_correction_max_);

        l_vel -= corr * 0.5;
        r_vel += corr * 0.5;

        if (std::abs(err) <= sync_err_limit_) {
            sync_err_ticks_ = 0;
            return;
        }

        if (++sync_err_ticks_ >= sync_err_fault_ticks_) {
            l_vel = 0.0;
            r_vel = 0.0;
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "[BeltNew] SYNC FAULT: |err|=%.3f > %.3f for %d ticks, stopping", std::abs(err),
                sync_err_limit_, sync_err_ticks_);
        }
    }

    // ── 物理堵转检测 ──────────────────────────────────────────────────────────
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

    // 电机指令与状态
    OutputInterface<double> left_control_velocity_;
    OutputInterface<double> right_control_velocity_;
    OutputInterface<double> left_control_torque_limit_;
    OutputInterface<double> right_control_torque_limit_;
    OutputInterface<bool> stalled_;

    // 虚拟零点
    double left_zero_offset_{0.0};
    double right_zero_offset_{0.0};
    bool left_calibrate_latched_{false};
    bool right_calibrate_latched_{false};

    // 手动
    bool manual_active_{false};
    double manual_l_velocity_{0.0};
    double manual_r_velocity_{0.0};
    double manual_torque_limit_cmd_{0.0};
    double manual_velocity_sensitivity_{};
    double manual_torque_limit_{};

    // 驱动方式与同步
    SyncMode sync_mode_{SyncMode::kSame};
    double debug_torque_limit_{};
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


