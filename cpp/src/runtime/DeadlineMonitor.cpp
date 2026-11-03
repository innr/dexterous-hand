#include "robot/runtime/DeadlineMonitor.hpp"

#include <cmath>
#include <stdexcept>

namespace robot {
DeadlineMonitor::DeadlineMonitor() : DeadlineMonitor(Config{}) {}

DeadlineMonitor::DeadlineMonitor(Config config) : config_(config) {
    if (config_.min_samples == 0 || config_.min_samples > kWindowSize ||
        !std::isfinite(config_.warning_rate) ||
        config_.warning_rate <= 0.0 || config_.warning_rate > 1.0 ||
        config_.critical_consecutive == 0)
        throw std::invalid_argument("invalid deadline monitor configuration");
}

void DeadlineMonitor::record(bool missed) noexcept {
    if (size_ == kWindowSize) {
        if (samples_[head_]) --missed_;
    } else {
        ++size_;
    }
    samples_[head_] = missed;
    if (missed) ++missed_;
    head_ = (head_ + 1) % kWindowSize;
    consecutive_ = missed ? consecutive_ + 1 : 0;
}

void DeadlineMonitor::reset() noexcept {
    samples_.fill(false);
    head_ = size_ = missed_ = consecutive_ = 0;
}

double DeadlineMonitor::missRate() const noexcept {
    return size_ == 0 ? 0.0 : static_cast<double>(missed_) / static_cast<double>(size_);
}

DeadlineHealth DeadlineMonitor::health() const noexcept {
    if (consecutive_ >= config_.critical_consecutive) return DeadlineHealth::CRITICAL;
    if (size_ >= config_.min_samples && missRate() >= config_.warning_rate)
        return DeadlineHealth::WARNING;
    return DeadlineHealth::NORMAL;
}
} // namespace robot
