#include "robot/runtime/PeriodicScheduler.hpp"

#include <stdexcept>
#include <thread>

namespace robot {
PeriodicScheduler::PeriodicScheduler(Duration period, TimePoint start)
    : period_(period), next_(start) {
    if (period_ <= Duration::zero()) throw std::invalid_argument("period must be positive");
}

PeriodicScheduler::TimePoint PeriodicScheduler::advance(TimePoint now) {
    if (now < next_) return next_;
    // now == next_ 时该计划时刻也已到期；避免补跑过期任务。
    const auto count = (now - next_) / period_ + 1;
    next_ += period_ * count;
    skipped_ += static_cast<std::uint64_t>(count);
    return next_;
}

void PeriodicScheduler::waitNext() {
    const auto scheduled = next_;
    std::this_thread::sleep_until(scheduled);
    const auto now = Clock::now();
    next_ = scheduled + period_; // 当前时刻对应的任务将由调用方执行
    if (now >= next_) advance(now);
}

void PeriodicScheduler::reset(TimePoint start) {
    next_ = start;
    skipped_ = 0;
}
} // namespace robot
