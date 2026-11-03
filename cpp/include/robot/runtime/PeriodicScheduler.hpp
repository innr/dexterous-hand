#pragma once

#include <chrono>
#include <cstdint>

// 周期调度器：使用单调时钟，过期周期直接跳过，不执行补跑。
namespace robot {
class PeriodicScheduler {
public:
    using Clock = std::chrono::steady_clock;
    using Duration = Clock::duration;
    using TimePoint = Clock::time_point;

    explicit PeriodicScheduler(Duration period, TimePoint start = Clock::now());

    // 返回严格晚于 now 的下次启动时刻，并统计被跳过的计划时刻。
    TimePoint advance(TimePoint now);
    // 等待当前计划时刻；正常执行不计为跳过，迟到时跳过后续过期周期。
    void waitNext();
    void reset(TimePoint start = Clock::now());

    Duration period() const noexcept { return period_; }
    TimePoint nextTime() const noexcept { return next_; }
    std::uint64_t skippedCycles() const noexcept { return skipped_; }

private:
    Duration period_;
    TimePoint next_;
    std::uint64_t skipped_ = 0;
};
} // namespace robot
