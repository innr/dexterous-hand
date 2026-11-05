#pragma once
#include "robot/control/AccelerationLimiter.hpp"
#include "robot/control/JointTrajectoryPlanner.hpp"
#include "robot/control/PDController.hpp"
#include "robot/runtime/FeedbackBuffer.hpp"
#include <array>
#include <chrono>
#include <memory>
#include <optional>

namespace robot {
// 外环的 PD 输出为位置偏置（deg），不是电流/转矩；总线只接收位置目标。
struct MotionJointConfig {
    float min_deg = -90.0f;
    float max_deg = 90.0f;
    float max_velocity_deg_s = 60.0f;
    float max_acceleration_deg_s2 = 180.0f;
    float kp = 1.0f;       // 位置误差 -> 位置偏置的无量纲增益
    float kd_seconds = 0.05f; // 速度误差 -> 位置偏置的秒数增益
};
struct MotionControlConfig {
    std::array<MotionJointConfig,kServoCount> joints{};
    std::chrono::milliseconds nominal_period{10};
    std::chrono::milliseconds max_step{50};
};

struct MotionStep {
    std::array<float,kServoCount> command_deg{};
    std::array<float,kServoCount> reference_deg{};
    std::array<float,kServoCount> reference_velocity_deg_s{};
    std::array<float,kServoCount> command_velocity_deg_s{};
};

class MotionControlCore {
public:
    explicit MotionControlCore(MotionControlConfig config);
    // 单控制线程调用。反馈必须已通过有效性/年龄检查。
    // false 表示不允许发布指令（超长 dt、超限反馈、非有限值等）。
    bool step(const std::array<float,kServoCount>& targets,
              const std::array<float,kServoCount>& measured,
              const std::array<float,kServoCount>& measured_velocity,
              ServoClock::time_point now, MotionStep& result);
private:
    MotionControlConfig config_;
    std::array<std::unique_ptr<JointTrajectoryPlanner>,kServoCount> planners_{};
    std::array<std::unique_ptr<AccelerationLimiter>,kServoCount> output_limiters_{};
    std::array<std::unique_ptr<PDController>,kServoCount> controllers_{};
    std::array<float,kServoCount> last_output_{};
    std::optional<ServoClock::time_point> last_tick_;
};
} // namespace robot
