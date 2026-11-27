#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::dart {

// ─────────────────────────────────────────────────────────────────────────────
// 手动控制分发框架
//
// 本文件是「框架」，不含任何具体机构的知识。
// 新增机构（belt / filling / chassis / yaw …）时，本文件一行都不用改。
//
// 用法：
//   1) 控制器在自己的 .cpp 里声明一份 static constexpr ManualBinding「说明书」，
//      写明「我归哪一档、用哪根杆、哪根轴、走哪两个函数」；
//   2) 构造函数里 ManualRegistry::add(&kManual, this) 把自己挂进表；
//   3) ManualDispatcher 组件每 tick 遍历一次，按档位把每个机构分到手动或自动。
//
// 读控制器 = 读到它全部的手动行为，不需要去别处查档位表。
// ─────────────────────────────────────────────────────────────────────────────

enum class StickSide : std::uint8_t { kLeft, kRight };
enum class StickAxis : std::uint8_t { kX, kY, kRotary };

// 一帧摇杆与滚轮快照（x 右为正、y 上为正、rotary_knob 范围 [-1, 1]）
struct StickSnapshot {
    Eigen::Vector2d left{Eigen::Vector2d::Zero()};
    Eigen::Vector2d right{Eigen::Vector2d::Zero()};
    double rotary_knob{0.0};
};

struct ManualBinding;

// 每个机构实现的两个动作，挂在对象上
struct ManualOps {
    void (*update_manual)(void* self, const ManualBinding& binding, const StickSnapshot& sticks);
    void (*update_auto)(void* self);
};

// 说明书：这个机构归哪一档、用哪根杆
struct ManualBinding {
    const char* name;

    rmcs_msgs::Switch left;   // 要求左拨杆 == 此值才进手动
    rmcs_msgs::Switch right;  // 要求右拨杆 == 此值才进手动

    StickSide side;
    StickAxis axis;

    const ManualOps* ops;

    // true 时：除 UNKNOWN 和双下急停外的所有已使能档位均可进入 update_manual（如全局滚轮调试）
    bool any_enabled_switch{false};
};

class ManualRegistry {
public:
    struct Entry {
        const ManualBinding* binding;
        void* self;
    };

    // 控制器在构造时调用
    static void add(const ManualBinding* binding, void* self) { registry().push_back(Entry{binding, self}); }

    static const std::vector<Entry>& entries() { return registry(); }

    // 一行遍历完成分发，返回本 tick 处于手动模式的机构数
    static std::size_t dispatch(
        rmcs_msgs::Switch left, rmcs_msgs::Switch right, const StickSnapshot& sticks) {
        std::size_t manual_count = 0;
        const bool enabled = left != rmcs_msgs::Switch::UNKNOWN
                          && right != rmcs_msgs::Switch::UNKNOWN
                          && !(left == rmcs_msgs::Switch::DOWN && right == rmcs_msgs::Switch::DOWN);

        for (const auto& entry : entries()) {
            const auto& binding = *entry.binding;
            const bool hit = binding.any_enabled_switch
                               ? enabled
                               : (left == binding.left && right == binding.right);

            if (hit) {
                binding.ops->update_manual(entry.self, binding, sticks);
                ++manual_count;
            } else {
                binding.ops->update_auto(entry.self);
            }
        }

        return manual_count;
    }

    // 按说明书取出对应摇杆轴或滚轮值
    static double axis_value(const ManualBinding& binding, const StickSnapshot& sticks) {
        if (binding.axis == StickAxis::kRotary)
            return sticks.rotary_knob;
        const auto& stick = binding.side == StickSide::kRight ? sticks.right : sticks.left;
        return binding.axis == StickAxis::kY ? stick.y() : stick.x();
    }

private:
    static std::vector<Entry>& registry() {
        static std::vector<Entry> list;
        return list;
    }
};

inline const char* switch_name(rmcs_msgs::Switch s) {
    switch (s) {
    case rmcs_msgs::Switch::UP: return "UP";
    case rmcs_msgs::Switch::DOWN: return "DOWN";
    case rmcs_msgs::Switch::MIDDLE: return "MID";
    case rmcs_msgs::Switch::UNKNOWN: return "?";
    }
    return "?";
}

} // namespace rmcs_core::controller::dart
