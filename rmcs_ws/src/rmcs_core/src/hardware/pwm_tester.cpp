#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>
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
//   ── servo 模式（给舵机标零用）──────────────────────────────────────────────
//     和 digital_pulse 同样的软件翻电平，但面向「舵机」语义：
//       · 参数是 pulse_us（脉宽）与 freq_hz（刷新频率，舵机通常 50Hz）
//       · 周期由 freq_hz 算出，不用手工凑 low_us
//       · pulse_us 支持运行时改，边转边看机械位置
//
//       ros2 param set /pwm_tester pulse_us 1500
//       ros2 param set /pwm_tester pulse_us 1600
//
//     ⚠️ 精度由 update_rate 决定，不是写死的：
//        1kHz → 1000us 粒度  （只能表达 1000/2000 两档）
//        4kHz →  250us 粒度  （500/750/1000/.../2500 共 9 档，够标零）
//        8kHz →  125us 粒度
//     所以舵机标零建议 update_rate: 4000.0，测完记得改回 1000。
//     ticks_per_us_ 现在从 update_rate 实算，不再硬编码。
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
        , logger_(get_logger()) {
        // 必须在构造函数注册 InputInterface，Executor::init() 才会在 before_updating() 之前完成绑定
        register_input("/predefined/update_rate", update_rate_, false);
    }

    void before_updating() override {
        mode_ = has_parameter("mode") ? get_parameter("mode").as_string() : std::string{"hold_low"};
        level_high_ = mode_ != "hold_low";
        high_us_ = read_double("high_us", 1500.0);
        low_us_ = read_double("low_us", 18500.0);  // 高 + 低 = 周期，默认 20ms
        pulse_us_ = read_double("pulse_us", 1500.0);
        freq_hz_ = read_double("freq_hz", 50.0);
        sweep_min_us_ = read_double("sweep_min_us", 500.0);
        sweep_max_us_ = read_double("sweep_max_us", 2500.0);
        sweep_hold_s_ = read_double("sweep_hold_s", 0.0);
        sweep_hold_ticks_ = read_double("sweep_hold_ticks", 1000.0);
        sweep_step_us_ = read_double("sweep_step_us", 100.0);
        channel_ = read_channel("channel");

        // 运行时调参：舵机标零要边转边看，不能每改一次都重启
        parameter_subscriber_ = std::make_unique<rclcpp::ParameterEventHandler>(this);
        parameter_callback_ = parameter_subscriber_->add_parameter_callback(
            "pulse_us", [this](const rclcpp::Parameter& param) {
                pulse_us_ = param.as_double();
                RCLCPP_INFO(
                    logger_, "[PwmTester] pulse_us -> %.0f us", pulse_us_);
            });

        RCLCPP_WARN(
            logger_, "[PwmTester] mode=%s channel=%u (%s)", mode_.c_str(),
            static_cast<unsigned>(channel_), channel_name(channel_).c_str());

        // 每种模式在启动时把「该看什么」讲清楚，省得现场回头翻代码
        if (mode_ == "hold_low" || mode_ == "hold_high") {
            RCLCPP_WARN(
                logger_, "[PwmTester] 引脚应恒定输出 %s 电平（万用表约 %s）。若读不到，说明 "
                         "gpio_digital_write 没通到引脚，软件 PWM 也不可行",
                level_high_ ? "高" : "低", level_high_ ? "3.3V" : "0V");
        } else if (mode_ == "servo") {
            RCLCPP_WARN(
                logger_,
                "[PwmTester] servo: pulse=%.0fus freq=%.1fHz 周期=%.0fus。"
                "运行时调脉宽：ros2 param set /pwm_tester pulse_us <值>",
                pulse_us_, freq_hz_, 1e6 / std::max(freq_hz_, 1.0));
        } else if (mode_ == "servo_sweep") {
            if (sweep_hold_s_ > 0.0) {
                RCLCPP_WARN(
                    logger_,
                    "[PwmTester] servo_sweep: %.0f~%.0fus 往返逐档遍历，每档停留 %.1fs，"
                    "看哪个脉宽对应哪个机械位置",
                    sweep_min_us_, sweep_max_us_, sweep_hold_s_);
            } else {
                RCLCPP_WARN(
                    logger_,
                    "[PwmTester] servo_sweep: %.0f~%.0fus 往返逐档遍历，每档 %.0f 周期，"
                    "看哪个脉宽对应哪个机械位置",
                    sweep_min_us_, sweep_max_us_, sweep_hold_ticks_);
            }
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
                logger_, "[PwmTester] 未知 mode='%s'，可选：hold_low/hold_high/servo/"
                         "servo_sweep/digital_pulse/analog/sweep/off",
                mode_.c_str());
            mode_ = "off";
        }

        board_ = std::make_unique<librmcs::board::RmcsBoardLite>();
    }

    void update() override {
        ensure_rate_initialized();

        if (mode_ == "hold_low" || mode_ == "hold_high")
            hold_level();
        else if (mode_ == "servo")
            servo_pulse();
        else if (mode_ == "servo_sweep")
            servo_sweep();
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

    // ── servo 模式：给舵机发固定脉宽 ─────────────────────────────────────────
    // 周期由 freq_hz 决定（舵机通常 50Hz = 20ms），脉宽由 pulse_us 决定。
    // 两者都换算成 tick，tick 粒度由 update_rate 决定：
    //   4kHz → 250us/tick → 500/750/1000/.../2500 共 9 档
    void servo_pulse() {
        const auto period_us = 1e6 / std::max(freq_hz_, 1.0);

        // 脉宽夹到 [最小1tick, 周期-1tick]，否则 low 段为 0 会连续高电平
        auto high_ticks = static_cast<std::uint32_t>(
            std::max(1.0, std::round(pulse_us_ * ticks_per_us_)));
        auto total_ticks = static_cast<std::uint32_t>(
            std::max(2.0, std::round(period_us * ticks_per_us_)));
        if (high_ticks >= total_ticks)
            high_ticks = total_ticks - 1;
        const auto low_ticks = total_ticks - high_ticks;

        step_pulse(high_ticks, low_ticks);

        // 参数变化时打一行，确认实际生效的 tick 数（对照示波器/机械位置）
        if (pulse_us_ != last_logged_pulse_us_) {
            last_logged_pulse_us_ = pulse_us_;
            RCLCPP_WARN(
                logger_,
                "[PwmTester] servo pulse=%.0fus -> high=%u ticks / low=%u ticks "
                "(实际 %.0fus / %.0fus, 周期 %.1fms)",
                pulse_us_, high_ticks, low_ticks, high_ticks / ticks_per_us_,
                low_ticks / ticks_per_us_, (total_ticks / ticks_per_us_) * 1e-3);
        }
    }

    // ── servo_sweep 模式：自动遍历所有可达档位 ───────────────────────────────
    // 4kHz 下每档 250us，从 min 到 max 再从 max 回 min，每档停留指定时间（或 tick）。
    // 用来看「哪个脉宽对应哪个机械位置」，从而定出扳机的两个目标位置。
    void servo_sweep() {
        const auto min_ticks = static_cast<std::uint32_t>(
            std::max(1.0, std::round(sweep_min_us_ * ticks_per_us_)));
        const auto max_ticks = static_cast<std::uint32_t>(
            std::max<double>(min_ticks, std::round(sweep_max_us_ * ticks_per_us_)));
        const auto period_us = 1e6 / std::max(freq_hz_, 1.0);
        auto total_ticks = static_cast<std::uint32_t>(
            std::max(2.0, std::round(period_us * ticks_per_us_)));

        const double rate_hz = ticks_per_us_ * 1e6;
        const auto dwell = sweep_hold_s_ > 0.0
            ? static_cast<std::uint64_t>(std::max(1.0, std::round(sweep_hold_s_ * rate_hz)))
            : static_cast<std::uint64_t>(std::max(1.0, sweep_hold_ticks_));
        const double dwell_s = static_cast<double>(dwell) / rate_hz;

        if (!sweep_initialized_) {
            sweep_initialized_ = true;
            sweep_index_ = min_ticks;
            sweep_up_ = true;
            sweep_count_ = 0;
            sweep_cycle_ = 0;
            RCLCPP_WARN(
                logger_,
                "[PwmTester] sweep #%lu [正向] pulse=%.0fus (%u ticks, 停留 %.2fs)",
                static_cast<unsigned long>(sweep_cycle_), sweep_index_ / ticks_per_us_,
                sweep_index_, dwell_s);
        }

        if (++sweep_count_ >= dwell) {
            sweep_count_ = 0;
            if (min_ticks < max_ticks) {
                if (sweep_up_) {
                    if (++sweep_index_ >= max_ticks) {
                        sweep_index_ = max_ticks;
                        sweep_up_ = false;  // 到顶，下一档回头
                    }
                } else {
                    if (sweep_index_ <= min_ticks + 1) {
                        sweep_index_ = min_ticks;
                        sweep_up_ = true;   // 到底，下一档回头
                        ++sweep_cycle_;
                    } else {
                        --sweep_index_;
                    }
                }
            }
            RCLCPP_WARN(
                logger_,
                "[PwmTester] sweep #%lu [%s] pulse=%.0fus (%u ticks, 停留 %.2fs)",
                static_cast<unsigned long>(sweep_cycle_),
                sweep_up_ ? "正向" : "反向",
                sweep_index_ / ticks_per_us_, sweep_index_, dwell_s);
        }

        auto high_ticks = std::clamp(sweep_index_, std::uint32_t{1}, total_ticks - 1);
        step_pulse(high_ticks, total_ticks - high_ticks);
    }

    // 软件翻电平的共同实现：数 tick 决定何时翻转
    void step_pulse(std::uint32_t high_ticks, std::uint32_t low_ticks) {
        ++tick_;
        const auto phase_ticks = high_ ? high_ticks : low_ticks;
        if (tick_ < phase_ticks)
            return;

        tick_ = 0;
        high_ = !high_;
        digital_write(high_);
    }

    // PredefinedMsgProvider::set_update_rate() 在 before_updating() 之后才被 Executor 调用，
    // 所以必须在首次 update() 时读取真实 update_rate，再换算 tick 粒度。
    void ensure_rate_initialized() {
        if (rate_initialized_)
            return;
        rate_initialized_ = true;

        if (update_rate_.ready() && *update_rate_ > 0.0) {
            ticks_per_us_ = *update_rate_ / 1e6;
        } else {
            ticks_per_us_ = 0.001;
            RCLCPP_WARN(
                logger_, "[PwmTester] /predefined/update_rate unavailable, assume 1kHz");
        }
        RCLCPP_WARN(
            logger_, "[PwmTester] update_rate=%.0fHz -> 每 tick %.0fus（PWM 精度就是这个粒度）",
            ticks_per_us_ * 1e6, 1.0 / ticks_per_us_);
    }

    // ── 模式 A：软件翻电平 ────────────────────────────────────────────────────
    // 每个 tick 判断是否该翻转，产生 high_us / low_us 的方波。
    //
    // ⚠️ 精度受 update_rate 硬限制：
    //    1kHz 时每 tick = 1ms，脉宽只能取 1ms 的整数倍。
    //    舵机需要 1000~2000us 的脉宽，在 1ms 粒度下 1500us 和 2500us
    //    都会被取整成 1 或 2 tick，**无法分辨**，所以此模式只能验证
    //    "引脚能以可控频率翻转"，不足以精确驱动舵机。
    //
    //    想要驱动舵机，update_rate 至少要 20kHz（每 tick 50us），
    //    但这会吃掉整机的控制周期预算，不现实。软件 PWM 的结论是：
    //    翻转能力成立，精度不成立。
    void digital_pulse() {
        ++tick_;
        // 四舍五入而不是截断，否则 1500us 会算成 1.5 -> 1 tick，误差 33%
        const auto high_ticks =
            std::max<std::uint32_t>(1, static_cast<std::uint32_t>(std::lround(high_us_ * ticks_per_us_)));
        const auto low_ticks =
            std::max<std::uint32_t>(1, static_cast<std::uint32_t>(std::lround(low_us_ * ticks_per_us_)));

        const auto phase_ticks = high_ ? high_ticks : low_ticks;
        if (tick_ < phase_ticks)
            return;
        tick_ = 0;
        high_ = !high_;

        digital_write(high_);

        // 每 500 次翻转打一行，附带实际生效的 tick 数，方便对照示波器
        if ((++pulse_count_ % 500) == 0) {
            RCLCPP_INFO(
                logger_, "[PwmTester] edges=%lu level=%s (high=%u ticks / low=%u ticks)",
                static_cast<unsigned long>(pulse_count_), high_ ? "HIGH" : "LOW",
                static_cast<unsigned>(high_ticks), static_cast<unsigned>(low_ticks));
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

    // 舵机专用
    double pulse_us_{1500.0};       // 目标脉宽（运行时可用 ros2 param set 改）
    double freq_hz_{50.0};          // 刷新频率，舵机通常 50Hz
    double last_logged_pulse_us_{0.0};
    double sweep_min_us_{500.0};    // servo_sweep 下端
    double sweep_max_us_{2500.0};   // servo_sweep 上端
    double sweep_hold_s_{0.0};      // servo_sweep 每档停留秒数（>0 时优先于 sweep_hold_ticks）
    double sweep_hold_ticks_{1000.0};  // 每档停留的控制周期数（sweep_hold_s<=0 时生效）
    std::uint32_t sweep_index_{0};
    std::uint64_t sweep_count_{0};
    std::uint64_t sweep_cycle_{0};
    bool sweep_up_{true};
    bool sweep_initialized_{false};
    bool rate_initialized_{false};

    // 执行器实际频率
    InputInterface<double> update_rate_;

    // 运行时调参
    std::unique_ptr<rclcpp::ParameterEventHandler> parameter_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_;

    std::uint32_t tick_{0};
    std::uint64_t pulse_count_{0};
    bool high_{false};
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::PwmTester, rmcs_executor::Component)
