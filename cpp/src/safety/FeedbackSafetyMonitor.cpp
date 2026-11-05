#include "robot/safety/FeedbackSafetyMonitor.hpp"
#include <cmath>
#include <stdexcept>

namespace robot {
FeedbackSafetyMonitor::FeedbackSafetyMonitor(FeedbackSafetyConfig config) : config_(config) {
    if (config_.startup_grace < std::chrono::milliseconds::zero() ||
        config_.warning_age <= std::chrono::milliseconds::zero() ||
        config_.hard_timeout <= config_.warning_age ||
        config_.warning_ticks == 0 || config_.stop_ticks == 0)
        throw std::invalid_argument("invalid feedback watchdog configuration");
}

FeedbackSafetyDecision FeedbackSafetyMonitor::evaluate(const FeedbackSnapshot& feedback,
                                                       ServoClock::time_point now) {
    FeedbackSafetyDecision result;
    if (!config_.enabled) return result;
    if (!first_tick_) first_tick_ = now;
    for (std::size_t i=0; i<kServoCount; ++i) {
        if (!feedback.valid[i] || feedback.timestamp[i] > now ||
            !std::isfinite(feedback.position[i]) ||
            !std::isfinite(feedback.velocity[i])) {
            ++result.invalid_joints;
            continue;
        }
        const auto age = now - feedback.timestamp[i];
        if (age > config_.hard_timeout) ++result.stale_joints;
        else if (age > config_.warning_age) ++result.warning_joints;
    }
    const bool critical = result.invalid_joints > 0 || result.stale_joints > 0;
    if (!critical) ever_healthy_ = true;

    // 初次启动允许通信线程填满 16 路，但期间 ControlLoop 自己拒绝不完整反馈。
    if (!ever_healthy_ && critical && now >= *first_tick_ &&
        now - *first_tick_ < config_.startup_grace) {
        result.startup_waiting = true;
        result.state = state_.state();
        return result;
    }

    if (critical) {
        ++consecutive_critical_;
        consecutive_warnings_ = 0;
        motion_inhibited_ = true;
        state_.degrade();
        if (consecutive_critical_ >= config_.stop_ticks) state_.requestStop();
    } else {
        consecutive_critical_ = 0;
        if (result.warning_joints > 0) {
            ++consecutive_warnings_;
            if (consecutive_warnings_ >= config_.warning_ticks) state_.degrade();
        } else {
            consecutive_warnings_ = 0;
        }
    }
    result.state = state_.state();
    result.consecutive_critical_ticks = consecutive_critical_;
    result.inhibit_motion = motion_inhibited_;
    result.request_safe_stop = result.state == SafetyState::SAFE_STOP ||
                               result.state == SafetyState::FAULT;
    return result;
}
} // namespace robot
