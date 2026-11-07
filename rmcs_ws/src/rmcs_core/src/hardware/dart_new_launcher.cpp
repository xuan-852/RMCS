#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/limit_switch.hpp"

#include <librmcs/board/rmcs_board_lite.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

// ─────────────────────────────────────────────────────────────────────────────
// DartNewLauncher
//   飞镖发射机构硬件组件（Board Lite）。
//
//   职责边界：
//     - 只负责硬件 IO：板子生命周期、GPIO 配置、CAN 收发、目标值下发
//     - PID 运算交给 rmcs_core::controller::pid::PidController 组件
//     - 参数全部来自 yaml，代码内不设默认值
// ─────────────────────────────────────────────────────────────────────────────
class DartNewLauncher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartNewLauncher()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<CommandComponent>(
                  get_component_name() + "_command", *this)) {

        dart_board_ = std::make_unique<DartBoard>(
            *this, *command_component_, get_parameter("board_serial").as_string());
    }

    DartNewLauncher(const DartNewLauncher&) = delete;
    DartNewLauncher& operator=(const DartNewLauncher&) = delete;
    DartNewLauncher(DartNewLauncher&&) = delete;
    DartNewLauncher& operator=(DartNewLauncher&&) = delete;

    ~DartNewLauncher() override = default;

    void update() override { dart_board_->update(); }

private:
    // ── 命令侧组件 ────────────────────────────────────────────────────────────
    // 只负责把 control_torque 发上 CAN。
    // 与主组件分离是为了让 PidController 的依赖方向保持单向（PID ← hardware）。
    class CommandComponent : public rmcs_executor::Component {
    public:
        explicit CommandComponent(DartNewLauncher& owner)
            : owner_(owner) {}

        void update() override { owner_.command_update(); }

    private:
        DartNewLauncher& owner_;
    };

    // ── 板级实现 ──────────────────────────────────────────────────────────────
    struct DartBoard final : librmcs::board::RmcsBoardLite::Callback {
        explicit DartBoard(
            DartNewLauncher& launcher, rmcs_executor::Component& launcher_command,
            std::string_view board_serial = {})
            : logger_(launcher.get_logger())
            , micro_switch_(launcher, "/dart/micro_switch")
            , carriage_motor_(launcher, launcher_command, "/dart/carriage_motor") {

            // 微动开关
            micro_switch_pull_up_ = launcher.get_parameter("micro_switch_pull_up").as_bool();
            micro_switch_active_low_ = launcher.get_parameter("micro_switch_active_low").as_bool();
            const auto read_period_ms = static_cast<std::uint16_t>(std::clamp<std::int64_t>(
                launcher.get_parameter("micro_switch_read_period_ms").as_int(), 0, UINT16_MAX));

            // 调试观察用：LimitSwitch 输出是 bool，无法通过 ValueBroadcaster 上 ROS
            launcher.register_output("/dart/micro_switch/observed", micro_switch_observed_, 0.0);

            // 调试观察用：遥控器拨杆状态与实际下发力矩
            launcher.register_output("/dart/remote/switch_left", remote_switch_left_, 0.0);
            launcher.register_output("/dart/remote/switch_right", remote_switch_right_, 0.0);
            launcher.register_output("/dart/remote/valid", remote_valid_, 0.0);
            launcher.register_output("/dart/carriage_motor/applied_torque", applied_torque_, 0.0);

            // 弹舱电机
            const auto motor_id =
                static_cast<std::uint8_t>(launcher.get_parameter("carriage_motor_id").as_int());
            const auto motor_ratio =
                launcher.get_parameter("carriage_motor_reduction_ratio").as_double();

            auto motor_config = device::DjiMotor::Config{device::DjiMotor::Type::kM3508, motor_id};
            motor_config.set_reduction_ratio(motor_ratio).enable_multi_turn_angle();
            if (launcher.get_parameter("carriage_motor_reversed").as_bool())
                motor_config.set_reversed();
            carriage_motor_.configure(motor_config);

            // 板子
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, board_serial);
            board_->start_transmit().gpio_digital_read(
                Spec::kGpios.kUart1Rx,
                {
                    .period_ms = read_period_ms,
                    .pull = micro_switch_pull_up_ ? librmcs::data::GpioPull::kUp
                                                  : librmcs::data::GpioPull::kNone,
                });

            RCLCPP_INFO(
                logger_, "[DartNewLauncher] ready: micro_switch on UART1_RX, motor id=%u ratio=%.2f",
                motor_id, motor_ratio);
        }

        void update() {
            micro_switch_.update_status();
            *micro_switch_observed_ = micro_switch_.status() ? 1.0 : 0.0;

            dr16_.update_status();
            update_motor_enable();

            // 调试观察
            *remote_switch_left_ = static_cast<double>(dr16_.switch_left());
            *remote_switch_right_ = static_cast<double>(dr16_.switch_right());
            *remote_valid_ = dr16_.valid() ? 1.0 : 0.0;
        }

        void command_update() {
            carriage_motor_.update_status();
            can_transmit();
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus)
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        void gpio_digital_read_result_callback(
            const Spec::Gpio& gpio, const View::GpioDigital& data) override {
            if (gpio != Spec::kGpios.kUart1Rx)
                return;
            micro_switch_.store_status(micro_switch_active_low_ ? !data.high : data.high);
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (!(can == librmcs::spec::rmcs_board_lite::kCanDescriptors.kCan0))
                return;
            carriage_motor_.match_then_store_status(data.can_id, data.can_data);
        }

        // 把 PidController 算出的力矩发到 CAN0。
        // 失能时发 NaN，DJI 电调会输出零电流（自由转动）。
        // 0x200 帧可打包 4 个电机，当前只接 1 个，其余槽位填零。
        void can_transmit() {
            const double torque = motor_disabled_ ? std::numeric_limits<double>::quiet_NaN()
                                                  : carriage_motor_.control_torque();
            *applied_torque_ = torque;

            auto builder = board_->start_transmit();
            builder.can_transmit(
                librmcs::spec::rmcs_board_lite::kCanDescriptors.kCan0,
                {
                    .can_id = carriage_motor_.send_id(),
                    .can_data = device::CanPacket8{
                        carriage_motor_.generate_command(torque),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
                });
        }

        // 失能条件：
        //   1) 任一拨杆为 UNKNOWN —— 遥控器未接 / 掉线 / 数据无效
        //   2) 左右拨杆同时拨到下 —— 手动急停
        // 两者任一成立即失能，电机零电流。
        void update_motor_enable() {
            const auto left = dr16_.switch_left();
            const auto right = dr16_.switch_right();

            const bool disabled = left == rmcs_msgs::Switch::UNKNOWN
                               || right == rmcs_msgs::Switch::UNKNOWN
                               || (left == rmcs_msgs::Switch::DOWN
                                   && right == rmcs_msgs::Switch::DOWN);

            if (disabled == motor_disabled_)
                return;

            motor_disabled_ = disabled;
            RCLCPP_INFO(
                logger_, "[DartNewLauncher] motor %s (switch L/R = %d/%d)",
                disabled ? "DISABLED" : "ENABLED", static_cast<int>(left),
                static_cast<int>(right));
        }

        rclcpp::Logger logger_;

        device::LimitSwitch micro_switch_;
        OutputInterface<double> micro_switch_observed_;
        device::DjiMotor carriage_motor_;
        device::Dr16 dr16_;

        OutputInterface<double> remote_switch_left_;
        OutputInterface<double> remote_switch_right_;
        OutputInterface<double> remote_valid_;
        OutputInterface<double> applied_torque_;

        bool micro_switch_pull_up_{};
        bool micro_switch_active_low_{};
        bool motor_disabled_{false};

        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    void command_update() { dart_board_->command_update(); }

    std::shared_ptr<CommandComponent> command_component_;
    std::unique_ptr<DartBoard> dart_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartNewLauncher, rmcs_executor::Component)

