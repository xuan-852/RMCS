#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/limit_switch.hpp"
#include "hardware/device/pwm_servo.hpp"
#include "hardware/device/remote_control.hpp"

#include <librmcs/board/rmcs_board_pro.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

// ─────────────────────────────────────────────────────────────────────────────
// DartNewLauncher
//   飞镖发射机构硬件组件（Board Pro）。
//   硬件端口、电机 ID/减速比/方向及微动开关配置直接写死在代码内（对齐 4z-dart-launcher）。
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

        std::string board_serial;
        get_parameter_or("board_serial", board_serial, std::string{});
        dart_board_ = std::make_unique<DartBoard>(*this, *command_component_, board_serial);
    }

    DartNewLauncher(const DartNewLauncher&) = delete;
    DartNewLauncher& operator=(const DartNewLauncher&) = delete;
    DartNewLauncher(DartNewLauncher&&) = delete;
    DartNewLauncher& operator=(DartNewLauncher&&) = delete;

    ~DartNewLauncher() override = default;

    void update() override { dart_board_->update(); }

private:
    class CommandComponent : public rmcs_executor::Component {
    public:
        explicit CommandComponent(DartNewLauncher& owner)
            : owner_(owner) {}

        void update() override { owner_.command_update(); }

    private:
        DartNewLauncher& owner_;
    };

    struct DartBoard final : librmcs::board::RmcsBoardPro::Callback {
        explicit DartBoard(
            DartNewLauncher& launcher, rmcs_executor::Component& launcher_command,
            std::string_view board_serial = {})
            : logger_(launcher.get_logger())
            , micro_switch_(launcher, "/dart/micro_switch")
            , carriage_motor_(launcher, launcher_command, "/dart/carriage_motor")
            , belt_left_motor_(launcher, launcher_command, "/dart/belt/left_motor")
            , belt_right_motor_(launcher, launcher_command, "/dart/belt/right_motor")
            , remote_control_(launcher)
            , trigger_servo_{launcher_command, "/dart/trigger_servo", 20.0, 1.90, 2.07} {

            // 调试观察输出
            launcher.register_output("/dart/micro_switch/observed", micro_switch_observed_, 0.0);
            launcher.register_output("/dart/remote/switch_left", remote_switch_left_, 0.0);
            launcher.register_output("/dart/remote/switch_right", remote_switch_right_, 0.0);
            launcher.register_output("/dart/remote/valid", remote_valid_, 0.0);
            launcher.register_output("/dart/carriage_motor/applied_torque", applied_torque_, 0.0);

            // CAN1 3 路 M3508（19:1，多圈角度）：ID1=carriage, ID2=belt_left, ID3=belt_right(reversed)
            carriage_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                    .set_reduction_ratio(19.)
                    .enable_multi_turn_angle());

            belt_left_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}
                    .set_reduction_ratio(19.)
                    .enable_multi_turn_angle());

            belt_right_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                    .set_reversed()
                    .set_reduction_ratio(19.)
                    .enable_multi_turn_angle());

            board_ = std::make_unique<librmcs::board::RmcsBoardPro>(*this, board_serial);

            // 微动开关：Board Pro UART1_RX，内部上拉，10ms 周期读，低电平有效
            // 扳机舵机：Board Pro PWM3（50Hz，1.90ms 释放 ~ 2.05ms 锁住），上电默认输出锁住脉宽
            {
                auto init_tx = board_->start_transmit();
                init_tx.gpio_digital_read(
                    Spec::kGpios.kUart1Rx,
                    {
                        .period_ms = 10,
                        .pull = librmcs::data::GpioPull::kUp,
                    });
                init_tx.gpio_analog_write(
                    Spec::kGpios.kPwm3,
                    librmcs::data::GpioAnalogDataView{
                        .value = trigger_servo_.generate_duty_cycle(1.0)});
            }

            remote_control_.register_dr16(&dr16_);

            RCLCPP_INFO(
                logger_,
                "[DartNewLauncher] ready (Board Pro CAN1: ID1=carriage, ID2=belt_L, ID3=belt_R; "
                "PWM3=trigger_servo 1.90~2.05ms; micro_switch=UART1_RX)");
        }

        void update() {
            micro_switch_.update_status();
            *micro_switch_observed_ = micro_switch_.status() ? 1.0 : 0.0;

            dr16_.update_status();
            remote_control_.update();
            update_motor_enable();

            carriage_motor_.update_status();
            belt_left_motor_.update_status();
            belt_right_motor_.update_status();

            *remote_switch_left_ = static_cast<double>(dr16_.switch_left());
            *remote_switch_right_ = static_cast<double>(dr16_.switch_right());
            *remote_valid_ = dr16_.valid() ? 1.0 : 0.0;
        }

        void command_update() { can_transmit(); }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus)
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        void gpio_digital_read_result_callback(
            const Spec::Gpio& gpio, const View::GpioDigital& data) override {
            if (gpio != Spec::kGpios.kUart1Rx)
                return;
            micro_switch_.store_status(!data.high);
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            auto can_id = data.can_id;

            if (can == Spec::kCans.kCan1) {
                if (can_id == 0x201) {
                    carriage_motor_.store_status(data.can_data);
                } else if (can_id == 0x202) {
                    belt_left_motor_.store_status(data.can_data);
                } else if (can_id == 0x203) {
                    belt_right_motor_.store_status(data.can_data);
                }
            }
        }

        void can_transmit() {
            auto builder = board_->start_transmit();
            const double nan = std::numeric_limits<double>::quiet_NaN();

            *applied_torque_ = carriage_motor_.control_torque();

            // 扳机舵机：Board Pro 硬件 PWM3（0.0=1900us 发射, 1.0=2050us 锁住；断控/急停强制锁住）
            builder.gpio_analog_write(
                Spec::kGpios.kPwm3,
                librmcs::data::GpioAnalogDataView{
                    .value = motor_disabled_ ? trigger_servo_.generate_duty_cycle(1.0)
                                             : trigger_servo_.generate_duty_cycle()});

            builder.can_transmit(
                Spec::kCans.kCan1, //
                {
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            motor_disabled_ ? carriage_motor_.generate_command(nan)
                                            : carriage_motor_.generate_command(),
                            motor_disabled_ ? belt_left_motor_.generate_command(nan)
                                            : belt_left_motor_.generate_command(),
                            motor_disabled_ ? belt_right_motor_.generate_command(nan)
                                            : belt_right_motor_.generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
        }

        // 失能条件：遥控器未接/掉线，或左右拨杆双下急停
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
        device::DjiMotor belt_left_motor_;
        device::DjiMotor belt_right_motor_;
        device::Dr16 dr16_;
        device::RemoteControl remote_control_;
        device::PWMServo trigger_servo_;

        OutputInterface<double> remote_switch_left_;
        OutputInterface<double> remote_switch_right_;
        OutputInterface<double> remote_valid_;
        OutputInterface<double> applied_torque_;

        bool motor_disabled_{true};

        std::unique_ptr<librmcs::board::RmcsBoardPro> board_;
    };

    void command_update() { dart_board_->command_update(); }

    std::shared_ptr<CommandComponent> command_component_;
    std::unique_ptr<DartBoard> dart_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartNewLauncher, rmcs_executor::Component)

