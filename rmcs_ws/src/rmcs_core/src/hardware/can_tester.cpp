#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <span>
#include <string>
#include <string_view>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

// ═════════════════════════════════════════════════════════════════════════════
// CanTester
//   独立组件：用于验收一块 C 板的 USB 通信与 CAN1 ↔ CAN2 双向硬件链路。
//
//   接线要求：
//     用一根 CAN 线把板上的 CAN1 和 CAN2 直接对接（CAN1_H ↔ CAN2_H，CAN1_L ↔ CAN2_L）。
//     不要接电机或其它 CAN 节点，避免总线干扰或 ID 冲突。
//
//   测试逻辑：
//     - CAN1 发送标准帧（默认 0x301，载荷 "C1TX" + 递增序号），期望从 CAN2 收到并给出硬件 ACK
//     - CAN2 发送标准帧（默认 0x302，载荷 "C2TX" + 递增序号），期望从 CAN1 收到并给出硬件 ACK
//     - 每秒打印一次收发统计与判定结论
//
//   ⚠️ 与 CatapultDart 等使用同一块 C 板的组件互斥，不能同时加载。
// ═════════════════════════════════════════════════════════════════════════════
class CanTester
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CanTester()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {}

    CanTester(const CanTester&) = delete;
    CanTester& operator=(const CanTester&) = delete;
    CanTester(CanTester&&) = delete;
    CanTester& operator=(CanTester&&) = delete;

    ~CanTester() override = default;

    void before_updating() override {
        const auto board_serial =
            has_parameter("board_serial") ? get_parameter("board_serial").as_string() : std::string{};
        const auto can1_tx_id = static_cast<std::uint32_t>(
            read_int("can1_tx_id", 0x301));
        const auto can2_tx_id = static_cast<std::uint32_t>(
            read_int("can2_tx_id", 0x302));
        tx_period_ticks_ = static_cast<std::uint64_t>(
            std::max<std::int64_t>(1, read_int("tx_period_ticks", 1)));
        report_period_ticks_ = static_cast<std::uint64_t>(
            std::max<std::int64_t>(1, read_int("report_period_ticks", 1000)));

        RCLCPP_WARN(
            logger_,
            "[CanTester] 启动 C 板 CAN1↔CAN2 双向回环测试："
            "CAN1_TX=0x%03X -> CAN2_RX | CAN2_TX=0x%03X -> CAN1_RX | "
            "tx_period=%lu ticks | report_period=%lu ticks",
            can1_tx_id, can2_tx_id,
            static_cast<unsigned long>(tx_period_ticks_),
            static_cast<unsigned long>(report_period_ticks_));
        RCLCPP_WARN(
            logger_,
            "[CanTester] 请确认已用线直连 CAN1_H↔CAN2_H、CAN1_L↔CAN2_L（未接电机）");

        board_ = std::make_unique<LoopbackBoard>(
            logger_, *this, can1_tx_id, can2_tx_id, board_serial);
    }

    void update() override {
        ++tick_;

        if ((tick_ % tx_period_ticks_) == 0)
            board_->send_pair();

        if ((tick_ % report_period_ticks_) == 0)
            board_->report();
    }

private:
    std::int64_t read_int(const char* name, std::int64_t def) const {
        if (!has_parameter(name))
            return def;
        return get_parameter(name).as_int();
    }

    struct LoopbackBoard final : librmcs::board::CBoard::Callback {
        LoopbackBoard(
            rclcpp::Logger logger, rclcpp::Node& node, std::uint32_t can1_tx_id,
            std::uint32_t can2_tx_id, std::string_view board_serial)
            : logger_(logger)
            , node_(node)
            , can1_tx_id_(can1_tx_id)
            , can2_tx_id_(can2_tx_id) {
            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
        }

        void send_pair() {
            const std::uint32_t seq = ++tx_seq_;

            const auto p1 = make_payload(kMagicCan1Tx, seq);
            const auto p2 = make_payload(kMagicCan2Tx, seq);

            try {
                auto builder = board_->start_transmit();
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {
                        .can_id = can1_tx_id_,
                        .can_data = std::span<const std::byte>{p1.data(), p1.size()},
                    });
                builder.can_transmit(
                    Spec::kCans.kCan2,
                    {
                        .can_id = can2_tx_id_,
                        .can_data = std::span<const std::byte>{p2.data(), p2.size()},
                    });
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(
                    logger_, *node_.get_clock(), 1000, "[CanTester] can_transmit failed: %s",
                    e.what());
            }
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;

            if (can == Spec::kCans.kCan2) {
                if (data.can_id == can1_tx_id_) {
                    std::uint32_t seq = 0;
                    if (check_payload(data.can_data, kMagicCan1Tx, seq)) {
                        rx_can1_to_can2_ok_.fetch_add(1, std::memory_order_relaxed);
                        last_seq_can1_to_can2_.store(seq, std::memory_order_relaxed);
                    } else {
                        rx_can1_to_can2_bad_.fetch_add(1, std::memory_order_relaxed);
                    }
                } else {
                    rx_can2_other_.fetch_add(1, std::memory_order_relaxed);
                }
                return;
            }

            if (can == Spec::kCans.kCan1) {
                if (data.can_id == can2_tx_id_) {
                    std::uint32_t seq = 0;
                    if (check_payload(data.can_data, kMagicCan2Tx, seq)) {
                        rx_can2_to_can1_ok_.fetch_add(1, std::memory_order_relaxed);
                        last_seq_can2_to_can1_.store(seq, std::memory_order_relaxed);
                    } else {
                        rx_can2_to_can1_bad_.fetch_add(1, std::memory_order_relaxed);
                    }
                } else {
                    rx_can1_other_.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }

        void report() {
            const auto tx = static_cast<std::uint64_t>(tx_seq_);
            const auto c12_ok = rx_can1_to_can2_ok_.load(std::memory_order_relaxed);
            const auto c12_bad = rx_can1_to_can2_bad_.load(std::memory_order_relaxed);
            const auto c12_seq = last_seq_can1_to_can2_.load(std::memory_order_relaxed);

            const auto c21_ok = rx_can2_to_can1_ok_.load(std::memory_order_relaxed);
            const auto c21_bad = rx_can2_to_can1_bad_.load(std::memory_order_relaxed);
            const auto c21_seq = last_seq_can2_to_can1_.load(std::memory_order_relaxed);

            const auto c1_other = rx_can1_other_.load(std::memory_order_relaxed);
            const auto c2_other = rx_can2_other_.load(std::memory_order_relaxed);

            const auto d12_ok = c12_ok - prev_c12_ok_;
            const auto d21_ok = c21_ok - prev_c21_ok_;
            const auto d_tx = tx - prev_tx_;
            prev_c12_ok_ = c12_ok;
            prev_c21_ok_ = c21_ok;
            prev_tx_ = tx;

            const char* verdict = "⏳ 等待首包...";
            if (d_tx > 0) {
                if (d12_ok > 0 && d21_ok > 0 && c12_bad == 0 && c21_bad == 0) {
                    verdict = "✅ 双向收发正常 (CAN1 & CAN2 均完好)";
                } else if (d12_ok == 0 && d21_ok == 0) {
                    verdict = "❌ 双向均未收到 (查接线 H/L 是否接反/虚接，或板载 CAN 异常)";
                } else if (d12_ok == 0) {
                    verdict = "⚠️ CAN1->CAN2 不通 (CAN2->CAN1 正常)";
                } else if (d21_ok == 0) {
                    verdict = "⚠️ CAN2->CAN1 不通 (CAN1->CAN2 正常)";
                } else {
                    verdict = "⚠️ 收到异常校验帧";
                }
            }

            RCLCPP_WARN(
                logger_,
                "[CanTester] %s | 本周期发=%lu | "
                "CAN1->CAN2: +%lu (累计 %lu/%lu, bad=%lu, seq=%u) | "
                "CAN2->CAN1: +%lu (累计 %lu/%lu, bad=%lu, seq=%u) | other(1/2)=%lu/%lu",
                verdict,
                static_cast<unsigned long>(d_tx),
                static_cast<unsigned long>(d12_ok),
                static_cast<unsigned long>(c12_ok),
                static_cast<unsigned long>(tx),
                static_cast<unsigned long>(c12_bad),
                c12_seq,
                static_cast<unsigned long>(d21_ok),
                static_cast<unsigned long>(c21_ok),
                static_cast<unsigned long>(tx),
                static_cast<unsigned long>(c21_bad),
                c21_seq,
                static_cast<unsigned long>(c1_other),
                static_cast<unsigned long>(c2_other));
        }

    private:
        static constexpr std::array<std::byte, 4> kMagicCan1Tx{
            std::byte{'C'}, std::byte{'1'}, std::byte{'T'}, std::byte{'X'}};
        static constexpr std::array<std::byte, 4> kMagicCan2Tx{
            std::byte{'C'}, std::byte{'2'}, std::byte{'T'}, std::byte{'X'}};

        static std::array<std::byte, 8> make_payload(
            const std::array<std::byte, 4>& magic, std::uint32_t seq) {
            std::array<std::byte, 8> buf{};
            std::memcpy(buf.data(), magic.data(), 4);
            std::memcpy(buf.data() + 4, &seq, sizeof(seq));
            return buf;
        }

        static bool check_payload(
            std::span<const std::byte> data, const std::array<std::byte, 4>& expected_magic,
            std::uint32_t& out_seq) {
            if (data.size() != 8)
                return false;
            if (std::memcmp(data.data(), expected_magic.data(), 4) != 0)
                return false;
            std::memcpy(&out_seq, data.data() + 4, sizeof(out_seq));
            return true;
        }

        rclcpp::Logger logger_;
        rclcpp::Node& node_;
        std::uint32_t can1_tx_id_;
        std::uint32_t can2_tx_id_;

        std::uint32_t tx_seq_{0};
        std::uint64_t prev_tx_{0};
        std::uint64_t prev_c12_ok_{0};
        std::uint64_t prev_c21_ok_{0};

        std::atomic<std::uint64_t> rx_can1_to_can2_ok_{0};
        std::atomic<std::uint64_t> rx_can1_to_can2_bad_{0};
        std::atomic<std::uint32_t> last_seq_can1_to_can2_{0};

        std::atomic<std::uint64_t> rx_can2_to_can1_ok_{0};
        std::atomic<std::uint64_t> rx_can2_to_can1_bad_{0};
        std::atomic<std::uint32_t> last_seq_can2_to_can1_{0};

        std::atomic<std::uint64_t> rx_can1_other_{0};
        std::atomic<std::uint64_t> rx_can2_other_{0};

        // 放最后构造，确保成员初始化完后才开启 USB 回调
        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    rclcpp::Logger logger_;
    std::uint64_t tx_period_ticks_{1};
    std::uint64_t report_period_ticks_{1000};
    std::uint64_t tick_{0};
    std::unique_ptr<LoopbackBoard> board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CanTester, rmcs_executor::Component)
