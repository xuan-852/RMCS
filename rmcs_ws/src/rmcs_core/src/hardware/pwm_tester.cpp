#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rmcs_executor/component.hpp>

#include <librmcs/board/rmcs_board_pro.hpp>
#include <librmcs/data/datas.hpp>

namespace rmcs_core::hardware {

// ═════════════════════════════════════════════════════════════════════════════
// PwmTester（Board Pro 硬件端口摸底工具：PWM + 空闲数字 IO 联动测试）
//
//   仅面向 RmcsBoardPro，且严格不动任何 UART 引脚（11~16 禁止访问）：
//     - PWM / 输出通道：0~3（kPwm0 ~ kPwm3，支持硬件 gpio_analog_write 与数字输出）
//     - 空闲 IO 通道：  4~10（kSpiI2cSocket0 ~ kSpiI2cSocket6，支持数字输入/输出）
//
//   ⚠️ 与 DartNewLauncher 互斥：两者都独占同一块 Board Pro，不能同时加载。
//
//   输出模式 mode（作用于 channel）：
//     - hw_servo       硬件 PWM 输出 50Hz 舵机脉宽（pulse_us 支持运行时 ros2 param set 调整）
//     - hw_servo_sweep 硬件 PWM 往返扫舵机行程（sweep_min_us ~ sweep_max_us）
//     - hold_low       恒定低电平（0V）
//     - hold_high      恒定高电平（3.3V）
//     - digital_pulse  软件翻转方波（high_us / low_us）
//     - servo          软件翻转舵机脉宽（受 update_rate 粒度限制）
//     - servo_sweep    软件翻转往返扫行程
//     - analog         直接硬件占空比（high_us / (high_us + low_us)）
//     - sweep          硬件占空比 1Hz 递增扫值
//     - off            不写输出，仅观察输入 IO 状态
//
//   输入监控（input_channels，从 0~10 中选，默认 [4..10] 且自动避开正在输出的 channel）：
//     启动时配置为 gpio_digital_read（默认上拉），任何引脚电平跳变立即在终端打印，
//     同时输出到 /pwm_tester/gpio_<ch> 供观察。
// ═════════════════════════════════════════════════════════════════════════════
class PwmTester
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PwmTester()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , callback_(*this) {
        register_input("/predefined/update_rate", update_rate_, false);

        for (std::uint8_t ch = 0; ch < kChannelCount; ++ch) {
            raw_inputs_[ch].store(-1, std::memory_order_relaxed);
            last_inputs_[ch] = -1;
            register_output("/pwm_tester/gpio_" + std::to_string(ch), gpio_outputs_[ch], 0.0);
        }
    }

    PwmTester(const PwmTester&) = delete;
    PwmTester& operator=(const PwmTester&) = delete;
    PwmTester(PwmTester&&) = delete;
    PwmTester& operator=(PwmTester&&) = delete;

    ~PwmTester() override = default;

    void before_updating() override {
        board_serial_ =
            has_parameter("board_serial") ? get_parameter("board_serial").as_string() : std::string{};
        mode_ = has_parameter("mode") ? get_parameter("mode").as_string() : std::string{"hw_servo"};
        level_high_ = (mode_ != "hold_low");
        high_us_ = read_double("high_us", 1500.0);
        low_us_ = read_double("low_us", 18500.0);
        pulse_us_.store(read_double("pulse_us", 1500.0), std::memory_order_relaxed);
        freq_hz_ = read_double("freq_hz", 50.0);
        sweep_min_us_ = read_double("sweep_min_us", 500.0);
        sweep_max_us_ = read_double("sweep_max_us", 2500.0);
        sweep_hold_s_ = read_double("sweep_hold_s", 3.0);
        sweep_hold_ticks_ = read_double("sweep_hold_ticks", 1000.0);
        sweep_step_us_ = read_double("sweep_step_us", 250.0);
        channel_ = read_output_channel("channel");

        if (has_parameter("input_pull")) {
            input_pull_ = get_parameter("input_pull").as_string();
        } else if (has_parameter("input_pull_up")) {
            input_pull_ = get_parameter("input_pull_up").as_bool() ? "up" : "none";
        } else {
            input_pull_ = "down";
        }
        input_period_ms_ = static_cast<std::uint16_t>(
            std::clamp<int>(static_cast<int>(read_double("input_period_ms", 20.0)), 1, 65535));

        validate_mode();
        read_input_channels();

        // 运行时调参：支持不重启直接修改 pulse_us
        parameter_subscriber_ = std::make_unique<rclcpp::ParameterEventHandler>(this);
        parameter_callback_ = parameter_subscriber_->add_parameter_callback(
            "pulse_us", [this](const rclcpp::Parameter& param) {
                const double val = param.as_double();
                pulse_us_.store(val, std::memory_order_relaxed);
                RCLCPP_INFO(logger_, "[PwmTester] pulse_us -> %.0f us", val);
            });

        board_ = std::make_unique<librmcs::board::RmcsBoardPro>(callback_, board_serial_);
        configure_input_reads();
    }

    void update() override {
        ensure_rate_initialized();
        poll_input_changes();

        if (mode_ == "hw_servo")
            hw_servo_pulse();
        else if (mode_ == "hw_servo_sweep")
            hw_servo_sweep();
        else if (mode_ == "hold_low" || mode_ == "hold_high")
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
    // Board Pro 可安全测试的通道范围：0~3 (PWM) + 4~10 (SPI/I2C 座子空闲 IO)
    // 11~16 为 UART0/1/2 引脚，本组件严格不碰。
    static constexpr std::uint8_t kChannelCount = 11;
    static constexpr std::uint8_t kMaxPwmChannel = 3;
    static constexpr std::uint8_t kMinSocketChannel = 4;
    static constexpr std::uint8_t kMaxSocketChannel = 10;

    struct BoardCallback final : librmcs::board::RmcsBoardPro::Callback {
        explicit BoardCallback(PwmTester& owner)
            : owner_(owner) {}

        void gpio_digital_read_result_callback(
            const Spec::Gpio& gpio, const View::GpioDigital& data) override {
            if (gpio.channel_index >= kChannelCount)
                return;
            owner_.raw_inputs_[gpio.channel_index].store(
                data.high ? 1 : 0, std::memory_order_relaxed);
        }

        PwmTester& owner_;
    };

    double read_double(const char* n, double def) {
        if (!has_parameter(n))
            return def;
        const auto& p = get_parameter(n);
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            return static_cast<double>(p.as_int());
        return p.as_double();
    }

    std::uint8_t read_output_channel(const char* n) {
        if (!has_parameter(n))
            return 2;  // 默认 kPwm2（与老镖架舵机同索引）
        const auto raw = static_cast<int>(get_parameter(n).as_int());
        if (raw < 0 || raw >= static_cast<int>(kChannelCount)) {
            RCLCPP_ERROR(
                logger_, "[PwmTester] channel=%d 超出安全范围 [0..10]（禁止触碰 UART 11..16），回退到 2",
                raw);
            return 2;
        }
        return static_cast<std::uint8_t>(raw);
    }

    void read_input_channels() {
        input_channels_.clear();
        std::vector<std::int64_t> configured;
        if (has_parameter("input_channels")) {
            configured = get_parameter("input_channels").as_integer_array();
        } else {
            configured = {4, 5, 6, 7, 8, 9, 10};
        }

        for (const auto raw : configured) {
            if (raw < 0 || raw >= static_cast<std::int64_t>(kChannelCount)) {
                RCLCPP_WARN(
                    logger_,
                    "[PwmTester] 跳过非法或 UART 输入通道 %ld（仅允许 [0..10]）",
                    static_cast<long>(raw));
                continue;
            }
            const auto ch = static_cast<std::uint8_t>(raw);
            if (mode_ != "off" && ch == channel_) {
                RCLCPP_WARN(
                    logger_, "[PwmTester] ch%u (%s) 正在作为输出口，自动跳过输入读取",
                    static_cast<unsigned>(ch), channel_name(ch));
                continue;
            }
            input_channels_.push_back(ch);
        }
    }

    void validate_mode() {
        const bool is_hw_pwm = (mode_ == "hw_servo" || mode_ == "hw_servo_sweep"
                             || mode_ == "analog" || mode_ == "sweep");
        if (is_hw_pwm && channel_ > kMaxPwmChannel) {
            RCLCPP_ERROR(
                logger_,
                "[PwmTester] mode='%s' 需要硬件 PWM 口 [0..3]，但 channel=%u (%s) 不支持，切回 off",
                mode_.c_str(), static_cast<unsigned>(channel_), channel_name(channel_));
            mode_ = "off";
        }

        RCLCPP_WARN(
            logger_, "[PwmTester] BoardPro mode=%s output=ch%u (%s)", mode_.c_str(),
            static_cast<unsigned>(channel_), channel_name(channel_));

        if (mode_ == "hw_servo") {
            RCLCPP_WARN(
                logger_,
                "[PwmTester] hw_servo (硬件 PWM): pulse=%.0fus freq=%.1fHz，支持运行时: "
                "ros2 param set /pwm_tester pulse_us <值>",
                pulse_us_.load(std::memory_order_relaxed), freq_hz_);
        } else if (mode_ == "hw_servo_sweep") {
            RCLCPP_WARN(
                logger_,
                "[PwmTester] hw_servo_sweep (硬件 PWM): %.0f~%.0fus 步进 %.0fus，每档停留 %.1fs",
                sweep_min_us_, sweep_max_us_, sweep_step_us_, sweep_hold_s_);
        } else if (mode_ == "hold_low" || mode_ == "hold_high") {
            RCLCPP_WARN(
                logger_, "[PwmTester] 恒定输出 %s 电平（%s）",
                level_high_ ? "HIGH" : "LOW", level_high_ ? "3.3V" : "0V");
        } else if (mode_ == "servo") {
            RCLCPP_WARN(
                logger_, "[PwmTester] servo (软件翻转): pulse=%.0fus freq=%.1fHz",
                pulse_us_.load(std::memory_order_relaxed), freq_hz_);
        } else if (mode_ == "servo_sweep") {
            RCLCPP_WARN(
                logger_, "[PwmTester] servo_sweep (软件翻转): %.0f~%.0fus",
                sweep_min_us_, sweep_max_us_);
        } else if (mode_ == "digital_pulse") {
            RCLCPP_WARN(
                logger_, "[PwmTester] digital_pulse: high=%.0fus low=%.0fus", high_us_, low_us_);
        } else if (mode_ == "analog") {
            RCLCPP_WARN(
                logger_, "[PwmTester] analog: duty = high_us/(high_us+low_us)");
        } else if (mode_ == "sweep") {
            RCLCPP_WARN(logger_, "[PwmTester] sweep: step +%.0fus/s", sweep_step_us_);
        } else if (mode_ == "off") {
            RCLCPP_WARN(logger_, "[PwmTester] off: 不输出，只读取 input_channels");
        } else {
            RCLCPP_ERROR(logger_, "[PwmTester] 未知 mode='%s'，切回 off", mode_.c_str());
            mode_ = "off";
        }
    }

    void configure_input_reads() {
        // 启动时先将 ch4~ch10（SpiI2cSocket0~6）全部拉到内部下拉低电位，清除上次运行残留的电平/上下拉；
        // 之后再仅对 YAML input_channels 中配置的通道按 input_pull 生效（例如 up 拉高）。
        {
            auto reset_tx = board_->start_transmit();
            for (std::uint8_t ch = kMinSocketChannel; ch <= kMaxSocketChannel; ++ch) {
                if (mode_ != "off" && ch == channel_)
                    continue;
                reset_tx.gpio_digital_read(
                    descriptor(ch), {.asap = true, .pull = librmcs::data::GpioPull::kDown});
            }
        }
        RCLCPP_WARN(
            logger_, "[PwmTester] 启动初始化 ch%u~ch%u 为下拉低电位 (kDown)",
            static_cast<unsigned>(kMinSocketChannel), static_cast<unsigned>(kMaxSocketChannel));

        if (input_channels_.empty()) {
            RCLCPP_WARN(logger_, "[PwmTester] 未配置任何输入 IO 监听通道");
            return;
        }

        auto tx = board_->start_transmit();
        auto pull = librmcs::data::GpioPull::kDown;
        const char* pull_label = "下拉";
        if (input_pull_ == "up") {
            pull = librmcs::data::GpioPull::kUp;
            pull_label = "上拉";
        } else if (input_pull_ == "none") {
            pull = librmcs::data::GpioPull::kNone;
            pull_label = "浮空";
        } else if (input_pull_ != "down") {
            RCLCPP_ERROR(
                logger_, "[PwmTester] 未知 input_pull='%s'，回退到 down(下拉)",
                input_pull_.c_str());
        }

        std::string list_str;
        for (const auto ch : input_channels_) {
            tx.gpio_digital_read(
                descriptor(ch), {.period_ms = input_period_ms_, .pull = pull});
            if (!list_str.empty())
                list_str += ", ";
            list_str += "ch" + std::to_string(ch) + "(" + channel_name(ch) + ")";
        }
        RCLCPP_WARN(
            logger_, "[PwmTester] 已开启 %zu 路 IO 读取 (%s, period=%ums): %s",
            input_channels_.size(), pull_label,
            static_cast<unsigned>(input_period_ms_), list_str.c_str());
    }

    void poll_input_changes() {
        for (const auto ch : input_channels_) {
            const auto val = raw_inputs_[ch].load(std::memory_order_relaxed);
            if (val < 0)
                continue;

            *gpio_outputs_[ch] = static_cast<double>(val);
            if (last_inputs_[ch] != val) {
                if (last_inputs_[ch] < 0) {
                    RCLCPP_INFO(
                        logger_, "[PwmTester] IO ch%u (%s) 初始=%s",
                        static_cast<unsigned>(ch), channel_name(ch), val ? "HIGH(1)" : "LOW(0)");
                } else {
                    RCLCPP_WARN(
                        logger_, "[PwmTester] IO ch%u (%s) 跳变: %s -> %s",
                        static_cast<unsigned>(ch), channel_name(ch),
                        last_inputs_[ch] ? "HIGH(1)" : "LOW(0)", val ? "HIGH(1)" : "LOW(0)");
                }
                last_inputs_[ch] = val;
            }
        }
    }

    static const char* channel_name(std::uint8_t ch) {
        switch (ch) {
        case 0: return "Pwm0";
        case 1: return "Pwm1";
        case 2: return "Pwm2";
        case 3: return "Pwm3";
        case 4: return "SpiI2cSocket0";
        case 5: return "SpiI2cSocket1";
        case 6: return "SpiI2cSocket2";
        case 7: return "SpiI2cSocket3";
        case 8: return "SpiI2cSocket4";
        case 9: return "SpiI2cSocket5";
        case 10: return "SpiI2cSocket6";
        default: return "?";
        }
    }

    static const librmcs::spec::rmcs_board_pro::GpioDescriptor& descriptor(std::uint8_t ch) {
        return librmcs::spec::rmcs_board_pro::kGpioDescriptors[ch];
    }

    void digital_write(bool high) {
        try {
            board_->start_transmit().gpio_digital_write(
                descriptor(channel_), librmcs::data::GpioDigitalDataView{.high = high});
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(
                logger_, *get_clock(), 1000, "[PwmTester] digital write failed: %s", e.what());
        }
    }

    void analog_write_duty(std::uint16_t duty) {
        if (channel_ > kMaxPwmChannel)
            return;
        try {
            board_->start_transmit().gpio_analog_write(
                descriptor(channel_), librmcs::data::GpioAnalogDataView{.value = duty});
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(
                logger_, *get_clock(), 1000, "[PwmTester] analog write failed: %s", e.what());
        }
    }

    std::uint16_t pulse_to_duty(double pulse_us) const {
        const double period_us = 1e6 / std::max(freq_hz_, 1.0);
        const double ratio = std::clamp(pulse_us / period_us, 0.0, 1.0);
        return static_cast<std::uint16_t>(std::lround(ratio * 65535.0));
    }

    // ── 硬件 PWM 舵机模式（Pro 板 kPwm0~kPwm3 原生支持）────────────────────────
    void hw_servo_pulse() {
        const double pulse = pulse_us_.load(std::memory_order_relaxed);
        const auto duty = pulse_to_duty(pulse);
        analog_write_duty(duty);

        if (pulse != last_logged_pulse_us_) {
            last_logged_pulse_us_ = pulse;
            RCLCPP_WARN(
                logger_, "[PwmTester] hw_servo ch%u (%s): pulse=%.0fus duty=%u/65535 (%.2f%%)",
                static_cast<unsigned>(channel_), channel_name(channel_), pulse, duty,
                static_cast<double>(duty) / 65535.0 * 100.0);
        }
    }

    void hw_servo_sweep() {
        const double rate_hz = ticks_per_us_ * 1e6;
        const auto dwell = sweep_hold_s_ > 0.0
            ? static_cast<std::uint64_t>(std::max(1.0, std::round(sweep_hold_s_ * rate_hz)))
            : static_cast<std::uint64_t>(std::max(1.0, sweep_hold_ticks_));
        const double step = std::max(1.0, sweep_step_us_);
        const double min_us = std::min(sweep_min_us_, sweep_max_us_);
        const double max_us = std::max(sweep_min_us_, sweep_max_us_);

        if (!sweep_initialized_) {
            sweep_initialized_ = true;
            hw_sweep_pulse_us_ = min_us;
            sweep_up_ = true;
            sweep_count_ = 0;
            sweep_cycle_ = 0;
            RCLCPP_WARN(
                logger_, "[PwmTester] hw_sweep #%lu [正向] pulse=%.0fus duty=%u",
                static_cast<unsigned long>(sweep_cycle_), hw_sweep_pulse_us_,
                pulse_to_duty(hw_sweep_pulse_us_));
        }

        if (++sweep_count_ >= dwell) {
            sweep_count_ = 0;
            if (sweep_up_) {
                hw_sweep_pulse_us_ += step;
                if (hw_sweep_pulse_us_ >= max_us) {
                    hw_sweep_pulse_us_ = max_us;
                    sweep_up_ = false;
                }
            } else {
                hw_sweep_pulse_us_ -= step;
                if (hw_sweep_pulse_us_ <= min_us) {
                    hw_sweep_pulse_us_ = min_us;
                    sweep_up_ = true;
                    ++sweep_cycle_;
                }
            }
            RCLCPP_WARN(
                logger_, "[PwmTester] hw_sweep #%lu [%s] pulse=%.0fus duty=%u",
                static_cast<unsigned long>(sweep_cycle_), sweep_up_ ? "正向" : "反向",
                hw_sweep_pulse_us_, pulse_to_duty(hw_sweep_pulse_us_));
        }

        analog_write_duty(pulse_to_duty(hw_sweep_pulse_us_));
    }

    // ── 恒定高低电平 ──────────────────────────────────────────────────────────
    void hold_level() {
        digital_write(level_high_);
        const auto count = ++tick_;
        if (count <= 3) {
            RCLCPP_WARN(
                logger_, "[PwmTester] hold %s on ch%u (%s) (write #%u)",
                level_high_ ? "HIGH" : "LOW", static_cast<unsigned>(channel_),
                channel_name(channel_), static_cast<unsigned>(count));
        }
    }

    // ── 软件翻转舵机模式（备用：排查任意 0~10 引脚翻转能力）─────────────────
    void servo_pulse() {
        const double pulse = pulse_us_.load(std::memory_order_relaxed);
        const auto period_us = 1e6 / std::max(freq_hz_, 1.0);
        auto high_ticks =
            static_cast<std::uint32_t>(std::max(1.0, std::round(pulse * ticks_per_us_)));
        auto total_ticks =
            static_cast<std::uint32_t>(std::max(2.0, std::round(period_us * ticks_per_us_)));
        if (high_ticks >= total_ticks)
            high_ticks = total_ticks - 1;
        step_pulse(high_ticks, total_ticks - high_ticks);

        if (pulse != last_logged_pulse_us_) {
            last_logged_pulse_us_ = pulse;
            RCLCPP_WARN(
                logger_, "[PwmTester] sw_servo pulse=%.0fus -> high=%u ticks / low=%u ticks",
                pulse, high_ticks, total_ticks - high_ticks);
        }
    }

    void servo_sweep() {
        const auto min_ticks =
            static_cast<std::uint32_t>(std::max(1.0, std::round(sweep_min_us_ * ticks_per_us_)));
        const auto max_ticks = static_cast<std::uint32_t>(
            std::max<double>(min_ticks, std::round(sweep_max_us_ * ticks_per_us_)));
        const auto period_us = 1e6 / std::max(freq_hz_, 1.0);
        auto total_ticks =
            static_cast<std::uint32_t>(std::max(2.0, std::round(period_us * ticks_per_us_)));

        const double rate_hz = ticks_per_us_ * 1e6;
        const auto dwell = sweep_hold_s_ > 0.0
            ? static_cast<std::uint64_t>(std::max(1.0, std::round(sweep_hold_s_ * rate_hz)))
            : static_cast<std::uint64_t>(std::max(1.0, sweep_hold_ticks_));

        if (!sweep_initialized_) {
            sweep_initialized_ = true;
            sweep_index_ = min_ticks;
            sweep_up_ = true;
            sweep_count_ = 0;
            sweep_cycle_ = 0;
            RCLCPP_WARN(
                logger_, "[PwmTester] sw_sweep #%lu [正向] pulse=%.0fus (%u ticks)",
                static_cast<unsigned long>(sweep_cycle_), sweep_index_ / ticks_per_us_,
                sweep_index_);
        }

        if (++sweep_count_ >= dwell) {
            sweep_count_ = 0;
            if (min_ticks < max_ticks) {
                if (sweep_up_) {
                    if (++sweep_index_ >= max_us_clamp(max_ticks, total_ticks)) {
                        sweep_index_ = max_us_clamp(max_ticks, total_ticks);
                        sweep_up_ = false;
                    }
                } else {
                    if (sweep_index_ <= min_ticks + 1) {
                        sweep_index_ = min_ticks;
                        sweep_up_ = true;
                        ++sweep_cycle_;
                    } else {
                        --sweep_index_;
                    }
                }
            }
            RCLCPP_WARN(
                logger_, "[PwmTester] sw_sweep #%lu [%s] pulse=%.0fus (%u ticks)",
                static_cast<unsigned long>(sweep_cycle_), sweep_up_ ? "正向" : "反向",
                sweep_index_ / ticks_per_us_, sweep_index_);
        }

        auto high_ticks = std::clamp(sweep_index_, std::uint32_t{1}, total_ticks - 1);
        step_pulse(high_ticks, total_ticks - high_ticks);
    }

    static std::uint32_t max_us_clamp(std::uint32_t max_ticks, std::uint32_t total_ticks) {
        return std::min(max_ticks, total_ticks - 1);
    }

    void step_pulse(std::uint32_t high_ticks, std::uint32_t low_ticks) {
        ++tick_;
        const auto phase_ticks = high_ ? high_ticks : low_ticks;
        if (tick_ < phase_ticks)
            return;

        tick_ = 0;
        high_ = !high_;
        digital_write(high_);
    }

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
    }

    void digital_pulse() {
        ++tick_;
        const auto high_ticks = std::max<std::uint32_t>(
            1, static_cast<std::uint32_t>(std::lround(high_us_ * ticks_per_us_)));
        const auto low_ticks = std::max<std::uint32_t>(
            1, static_cast<std::uint32_t>(std::lround(low_us_ * ticks_per_us_)));

        const auto phase_ticks = high_ ? high_ticks : low_ticks;
        if (tick_ < phase_ticks)
            return;
        tick_ = 0;
        high_ = !high_;
        digital_write(high_);
    }

    void analog_write() {
        const double period = high_us_ + low_us_;
        if (period <= 0.0)
            return;

        const double ratio = std::clamp(high_us_ / period, 0.0, 1.0);
        const auto duty = static_cast<std::uint16_t>(std::lround(ratio * 65535.0));

        analog_write_duty(duty);
        if (++tick_ == 1) {
            RCLCPP_WARN(
                logger_, "[PwmTester] analog ch%u (%s) ratio=%.3f duty=%u",
                static_cast<unsigned>(channel_), channel_name(channel_), ratio, duty);
        }
    }

    void sweep() {
        const double period_ticks = std::max(1.0, ticks_per_us_ * 1e6);
        if (++tick_ >= static_cast<std::uint32_t>(std::lround(period_ticks))) {
            tick_ = 0;
            high_us_ += sweep_step_us_;
            if (high_us_ > 3000.0)
                high_us_ = 500.0;
            RCLCPP_INFO(
                logger_, "[PwmTester] sweep high=%.0fus duty=%u", high_us_,
                pulse_to_duty(high_us_));
        }
        analog_write_duty(pulse_to_duty(high_us_));
    }

    rclcpp::Logger logger_;
    std::string board_serial_;

    std::array<std::atomic<std::int8_t>, kChannelCount> raw_inputs_{};
    std::array<std::int8_t, kChannelCount> last_inputs_{};
    std::array<OutputInterface<double>, kChannelCount> gpio_outputs_{};
    std::vector<std::uint8_t> input_channels_;
    std::string input_pull_{"down"};
    std::uint16_t input_period_ms_{20};

    BoardCallback callback_;
    std::unique_ptr<librmcs::board::RmcsBoardPro> board_;

    std::string mode_{"hw_servo"};
    std::uint8_t channel_{2};
    bool level_high_{false};
    double high_us_{1500.0};
    double low_us_{18500.0};
    double sweep_step_us_{250.0};
    double ticks_per_us_{0.001};

    std::atomic<double> pulse_us_{1500.0};
    double freq_hz_{50.0};
    double last_logged_pulse_us_{0.0};
    double sweep_min_us_{500.0};
    double sweep_max_us_{2500.0};
    double sweep_hold_s_{3.0};
    double sweep_hold_ticks_{1000.0};
    double hw_sweep_pulse_us_{500.0};
    std::uint32_t sweep_index_{0};
    std::uint64_t sweep_count_{0};
    std::uint64_t sweep_cycle_{0};
    bool sweep_up_{true};
    bool sweep_initialized_{false};
    bool rate_initialized_{false};

    InputInterface<double> update_rate_;
    std::unique_ptr<rclcpp::ParameterEventHandler> parameter_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_;

    std::uint32_t tick_{0};
    bool high_{false};
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::PwmTester, rmcs_executor::Component)
