#pragma once
#include "robot/runtime/FeedbackBuffer.hpp"
#include "robot/safety/SafetyStateMachine.hpp"
#include <chrono>
#include <cstddef>
#include <optional>

// S5-06：仅由控制线程调用；用单调时钟判断每路反馈是否过期。
// DEGRADED 状态不自动恢复；首次硬超时立即锁存运动门禁，持续超时进入 SAFE_STOP。
namespace robot {
struct FeedbackSafetyConfig {
    bool enabled = true;
    std::chrono::milliseconds startup_grace{100};
    std::chrono::milliseconds warning_age{20};
    std::chrono::milliseconds hard_timeout{30};
    std::size_t warning_ticks = 2;
    std::size_t stop_ticks = 3;
};

struct FeedbackSafetyDecision {
    SafetyState state = SafetyState::NORMAL;
    std::size_t invalid_joints = 0;
    std::size_t stale_joints = 0;
    std::size_t warning_joints = 0;
    std::size_t consecutive_critical_ticks = 0;
    bool startup_waiting = false;
    bool inhibit_motion = false; // 首次硬异常立即锁存，不能自动恢复。
    bool request_safe_stop = false;
};

class FeedbackSafetyMonitor {
public:
    explicit FeedbackSafetyMonitor(FeedbackSafetyConfig config = {});
    FeedbackSafetyDecision evaluate(const FeedbackSnapshot& feedback,
                                    ServoClock::time_point now);
private:
    FeedbackSafetyConfig config_;
    SafetyStateMachine state_;
    std::optional<ServoClock::time_point> first_tick_;
    bool ever_healthy_ = false;
    bool motion_inhibited_ = false;
    std::size_t consecutive_warnings_ = 0;
    std::size_t consecutive_critical_ = 0;
};
} // namespace robot
