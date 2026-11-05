#pragma once
#include "robot/runtime/FeedbackBuffer.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <vector>

namespace robot {
// 采样结果标志；未发布命令的行不得把零值误解释为真实指令。
enum class ControlTickStatus : std::uint8_t {
    PUBLISHED, STOP_GATE, INVALID_FEEDBACK, NO_TARGET, MOTION_REJECTED,
    SAFETY_BLOCKED
};
struct ControlTelemetryFrame {
    std::uint64_t tick = 0;
    std::int64_t time_us = 0;  // steady_clock 自 epoch 的微秒；仅本次运行内比较
    std::int64_t start_lateness_us = 0;
    std::int64_t control_duration_us = 0;
    std::uint64_t feedback_sequence = 0;
    ControlTickStatus status = ControlTickStatus::STOP_GATE;
    std::uint8_t safety_state = 0; // SafetyState underlying value
    std::array<float,kServoCount> target_deg{};
    std::array<float,kServoCount> reference_deg{};
    std::array<float,kServoCount> command_deg{};
    std::array<float,kServoCount> measured_deg{};
    std::array<float,kServoCount> measured_velocity_deg_s{};
    std::array<std::int64_t,kServoCount> feedback_age_us{};
    std::array<bool,kServoCount> feedback_valid{};
};

// 单生产者（控制线程）/单消费者（外部诊断线程）无锁队列。
// 满时丢弃新样本，绝不覆盖消费者正在读取的槽；没有动态内存/锁/IO。
class ControlTelemetryRing {
public:
    explicit ControlTelemetryRing(std::size_t capacity);
    ControlTelemetryRing(const ControlTelemetryRing&)=delete;
    ControlTelemetryRing& operator=(const ControlTelemetryRing&)=delete;
    bool tryPush(const ControlTelemetryFrame& frame) noexcept;
    bool tryPop(ControlTelemetryFrame& frame) noexcept;
    std::uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }
    std::size_t capacity() const noexcept { return slots_.size(); }
private:
    std::vector<ControlTelemetryFrame> slots_;
    std::atomic<std::uint64_t> write_{0};
    std::atomic<std::uint64_t> read_{0};
    std::atomic<std::uint64_t> dropped_{0};
};

// 在控制线程外调用；一行对应一个 tick 和一个 joint，CSV 长表便于筛选。
void writeTelemetryCsvHeader(std::ostream& out);
void writeTelemetryCsvFrame(std::ostream& out,const ControlTelemetryFrame& frame);
} // namespace robot
