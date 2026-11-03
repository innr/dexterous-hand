#pragma once
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>

// 16 路反馈：每个舵机独立记录位置、速度、采集时刻和有效性。
namespace robot {
constexpr std::size_t kServoCount = 16;
using ServoClock = std::chrono::steady_clock;
struct FeedbackSnapshot {
    std::array<float, kServoCount> position{};
    std::array<float, kServoCount> velocity{};
    std::array<ServoClock::time_point, kServoCount> timestamp{};
    std::array<bool, kServoCount> valid{};
    std::uint64_t sequence = 0;
};
class FeedbackBuffer {
public:
    void publish(const FeedbackSnapshot& value);
    FeedbackSnapshot read() const;
private:
    mutable std::mutex mutex_;
    FeedbackSnapshot value_{};
};
} // namespace robot
