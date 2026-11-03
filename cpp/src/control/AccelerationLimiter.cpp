// S3 | Limit change of velocity per update; state is retained between calls.
#include "robot/control/AccelerationLimiter.hpp"
#include "robot/control/TrajectoryMath.hpp"
#include <algorithm>
namespace robot {
AccelerationLimiter::AccelerationLimiter(float max_acceleration)
    : max_acceleration_(max_acceleration) { validateMotion(max_acceleration_, 1.0f); }
float AccelerationLimiter::update(float desired_velocity, float dt) {
    validateMotion(max_acceleration_, dt);
    const float max_step = max_acceleration_ * dt;
    velocity_ += std::clamp(desired_velocity - velocity_, -max_step, max_step);
    return velocity_;
}
float AccelerationLimiter::velocity() const { return velocity_; }
} // namespace robot
