#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include <librmcs/board/rmcs_board_lite.hpp>
#include <librmcs/data/datas.hpp>

namespace rmcs_core::hardware {

// ═════════════════════════════════════════════════════════════════════════════
// PwmTester
//   独立组件，专门在 Board Lite 的 UART1_RX (ch2) 上试 GPIO 输出。
//
//   独立于 DartNewLauncher，因为它要开一块自己的板子句柄，方便反复开关、
//   反复改参数，而不用碰主镖架逻辑。
//
//   ⚠️ 与 DartNewLauncher 互斥：两者都独占同一块板子，不能同时加载。
//
//   测试顺序（由浅入深，前一步不通过就别往下走）：
//
//     1. hold_low     引脚恒定输出低电平  → 万用表应为 0V
//     2. hold_high    引脚恒定输出高电平  → 万用表应为 3.3V
//     两档都是硬要求。只有这两档确认能切换，才说明 gpio_digital_write
//     这条链路真的通到了引脚；此时才能谈「软件 PWM 到底行不行」。
//     若低/高都是 3.3V，则是引脚被 UART 复用占用或固件没处理 digital write，
//     软件 PWM 必然无从谈起，得换硬件方案。
//
//     3. digital_pulse 用 gpio_digital_write 在软件里自己翻电平，
//                      周期/脉宽由 tick 数算。不依赖固件 PWM 能力。
//     4. analog        沿用 gpio_analog_write，复核固件是否解析 value。
//     5. sweep         低频爬 duty，配合万用表看引脚是否有变化。
//     6. off           什么都不写（方便观察引脚默认状态）
//
//   背景（2026-11-09 实测）：
//     给 ch2 打了 librmcs spec 补丁后，主机端能发 gpio_analog_write 不报错，
//     但 duty=0 与 duty=65535 实测输出完全相同（恒定 3.3V），怀疑固件不解析
//     analog value。
//
//   注意：Board Lite 的 gpio descriptor 里 4 个通道都声明了 kDigitalWrite
//   （见 librmcs/spec/rmcs_board_lite/gpio.hpp），所以数字输出理论上不受
//   那个 analog 补丁影响，值得单独验证。
// ═════════════════════════════════════════════════════════════════════════════
class PwmTester
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PwmTester()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {}

    void before_updating() override {
        mode_ = has_parameter("mode") ? get_parameter("mode").as_string() : std::string{"hold_low"};
        level_high_ = mode_ != "hold_low";
        high_us_ = read_double("high_us", 1500.0);
        low_us_ = read_double("low_us", 18500.0);  // 高 + 低 = 周期，默认 20ms
        sweep_step_us_ = read_double("sweep_step_us", 100.0);
        channel_ = read_channel("channel");

        RCLCPP_WARN(
            logger_, "[PwmTester] mode=%s channel=%u (%s)", mode_.c_str(),
            static_cast<unsigned>(channel_), channel_name(channel_).c_str());

        // 每种模式在启动时把「该看什么」讲清楚，省得现场回头翻代码
        if (mode_ == "hold_low" || mode_ == "hold_high") {
            RCLCPP_WARN(
                logger_, "[PwmTester] 引脚应恒定输出 %s 电平（万用表约 %s）。若读不到，说明 "
                         "gpio_digital_write 没通到引脚，软件 PWM 也不可行",
                level_high_ ? "高" : "低", level_high_ ? "3.3V" : "0V");
        } else if (mode_ == "digital_pulse") {
            RCLCPP_WARN(
                logger_,
                "[PwmTester] 软件脉冲 high=%.0fus low=%.0fus -> %.1fHz。示波器看有无方波",
                high_us_, low_us_, 1e6 / (high_us_ + low_us_));
        } else if (mode_ == "analog") {
            RCLCPP_WARN(logger_, "[PwmTester] gpio_analog_write，duty 由 high_us/周期 换算");
        } else if (mode_ == "sweep") {
            RCLCPP_WARN(
                logger_, "[PwmTester] 扫值模式，每次 +%.0fus，用万用表/示波器观察", sweep_step_us_);
        } else if (mode_ == "off") {
            RCLCPP_WARN(logger_, "[PwmTester] off：不写任何值，观察引脚默认状态");
        } else {
            RCLCPP_ERROR(
                logger_, "[PwmTester] 未知 mode='%s'，可选：hold_low/hold_high/digital_pulse/"
                         "analog/sweep/off",
                mode_.c_str());
            mode_ = "off";
        }

        board_ = std::make_unique<librmcs::board::RmcsBoardLite>();
    }

    void update() override {
        if (mode_ == "hold_low" || mode_ == "hold_high")
            hold_level();
        else if (mode_ == "digital_pulse")
            digital_pulse();
        else if (mode_ == "analog")
            analog_write();
        else if (mode_ == "sweep")
            sweep();
    }

private:
    double read_double(const char* n, double def) {
        if (!has_parameter(n))
            return def;
        const auto& p = get_parameter(n);
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            return static_cast<double>(p.as_int());
        return p.as_double();
    }

    // channel: 0=UART0_RX 1=UART0_TX 2=UART1_RX 3=UART1_TX
    std::uint8_t read_channel(const char* n) {
        if (!has_parameter(n))
            return 2;  // 默认 UART1_RX，之前一直测的那根
        const auto& p = get_parameter(n);
        const auto raw = static_cast<int>(p.as_int());
        if (raw < 0 || raw > 3) {
            RCLCPP_ERROR(logger_, "[PwmTester] channel=%d 越界，回退到 2", raw);
            return 2;
        }
        return static_cast<std::uint8_t>(raw);
    }

    static std::string channel_name(std::uint8_t ch) {
        switch (ch) {
        case 0: return "UART0_RX";
        case 1: return "UART0_TX";
        case 2: return "UART1_RX";
        case 3: return "UART1_TX";
        default: return "?";
        }
    }

    // 按 channel_ 取对应的 descriptor
    const librmcs::spec::rmcs_board_lite::GpioDescriptor& descriptor() const {
        using namespace librmcs::spec::rmcs_board_lite;
        switch (channel_) {
        case 0: return kGpioDescriptors.kUart0Rx;
        case 1: return kGpioDescriptors.kUart0Tx;
        case 3: return kGpioDescriptors.kUart1Tx;
        case 2:
        default: return kGpioDescriptors.kUart1Rx;
        }
    }

    void digital_write(bool high) {
        try {
            board_->start_transmit().gpio_digital_write(
                descriptor(), librmcs::data::GpioDigitalDataView{.high = high});
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 1000, "[PwmTester] digital write failed: %s",
                e.what());
        }
    }

    // ── 模式 0：恒定电平 ─────────────────────────────────────────────────────
    // 每条 tick 都重写，避免固件侧有周期刷新把引脚复位。
    // 只在头几帧打日志，之后静默，以免刷屏。
    void hold_level() {
        digital_write(level_high_);
        const auto count = ++tick_;
        if (count <= 3) {
            RCLCPP_WARN(
                logger_, "[PwmTester] hold %s on %s (write #%u)",
                level_high_ ? "HIGH" : "LOW", channel_name(channel_).c_str(),
                static_cast<unsigned>(count));
        }
    }

    // ── 模式 A：软件翻电平 ────────────────────────────────────────────────────
    // 每个 tick 判断是否该翻转，产生 high_us / low_us 的方波。
    // ⚠️ 精度受 update_rate 限制：1kHz 下最小脉宽 1ms，误差 ±1ms。
    //    此模式只能验证「引脚能不能出方波」，不足以精确驱动舵机。
    void digital_pulse() {
        ++tick_;
        const auto high_ticks =
            std::max<std::uint32_t>(1, static_cast<std::uint32_t>(high_us_ * ticks_per_us_));
        const auto low_ticks =
            std::max<std::uint32_t>(1, static_cast<std::uint32_t>(low_us_ * ticks_per_us_));

        const auto phase_ticks = high_ ? high_ticks : low_ticks;
        if (tick_ < phase_ticks)
            return;
        tick_ = 0;
        high_ = !high_;

        digital_write(high_);

        if ((++pulse_count_ % 200) == 0) {
            RCLCPP_INFO(
                logger_, "[PwmTester] pulse %s, edges=%lu", high_ ? "HIGH" : "LOW",
                static_cast<unsigned long>(pulse_count_));
        }
    }

    // ── 模式 B：analog write ─────────────────────────────────────────────────
    void analog_write() {
        const double period = high_us_ + low_us_;
        if (period <= 0.0)
            return;

        const double ratio = std::clamp(high_us_ / period, 0.0, 1.0);
        const auto duty = static_cast<std::uint16_t>(std::lround(ratio * 65535.0));

        const auto count = ++tick_;
        if (count > 2)
            return;
        if (count == 1) {
            RCLCPP_WARN(
                logger_, "[PwmTester] analog high=%.0fus period=%.0fus ratio=%.3f duty=%u",
                high_us_, period, ratio, duty);
        }
        try {
            board_->start_transmit().gpio_analog_write(
                descriptor(), librmcs::data::GpioAnalogDataView{.value = duty});
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "[PwmTester] analog write failed: %s", e.what());
        }
    }

    // ── 模式 C：低频扫值 ─────────────────────────────────────────────────────
    void sweep() {
        if ((++tick_ % 1000) != 0)  // 每秒一步
            return;

        high_us_ += sweep_step_us_;
        if (high_us_ > 3000.0)
            high_us_ = 500.0;

        const auto duty = static_cast<std::uint16_t>(std::lround(high_us_ / 20000.0 * 65535.0));
        try {
            board_->start_transmit().gpio_analog_write(
                descriptor(), librmcs::data::GpioAnalogDataView{.value = duty});
        } catch (const std::exception&) {
        }

        RCLCPP_INFO(logger_, "[PwmTester] sweep high=%.0fus duty=%u", high_us_, duty);
    }

    rclcpp::Logger logger_;
    std::unique_ptr<librmcs::board::RmcsBoardLite> board_;

    std::string mode_{"hold_low"};
    std::uint8_t channel_{2};  // 2 = UART1_RX
    bool level_high_{false};
    double high_us_{1500.0};
    double low_us_{18500.0};
    double sweep_step_us_{100.0};
    double ticks_per_us_{0.001};

    std::uint32_t tick_{0};
    std::uint64_t pulse_count_{0};
    bool high_{false};
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::PwmTester, rmcs_executor::Component)
