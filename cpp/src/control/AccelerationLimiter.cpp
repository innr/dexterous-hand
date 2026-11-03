#include "robot/control/AccelerationLimiter.hpp"

#include "robot/control/TrajectoryMath.hpp"

#include <algorithm>

namespace robot {

AccelerationLimiter::AccelerationLimiter(float max_acceleration)
    : max_acceleration_(max_acceleration) {
    // 构造时校验最大加速度合法（有限正值）。
    validateMotion(max_acceleration_, 1.0f);
}

float AccelerationLimiter::update(float desired_velocity, float dt) {
    validateMotion(max_acceleration_, dt);

    // 单步允许的最大速度变化 = 加速度 * dt，把变化量裁剪到该上限。
    const float max_step = max_acceleration_ * dt;
    velocity_ += std::clamp(desired_velocity - velocity_, -max_step, max_step);
    return velocity_;
}

float AccelerationLimiter::velocity() const { return velocity_; }

} // namespace robot
