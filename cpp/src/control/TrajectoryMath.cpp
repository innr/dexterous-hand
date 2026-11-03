// S3 | Pure kinematic helpers; angles in degrees and time in seconds.
#include "robot/control/TrajectoryMath.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace robot {
void validateMotion(float limit, float dt) {
    if (!std::isfinite(limit) || limit <= 0.0f || !std::isfinite(dt) || dt <= 0.0f)
        throw std::invalid_argument("motion limit/dt must be positive and finite");
}
float calculateStoppingDistance(float velocity, float max_deceleration) {
    validateMotion(max_deceleration, 1.0f);
    // Constant-deceleration stopping distance: v^2/(2a).
    return velocity * velocity / (2.0f * max_deceleration);
}
float calculateRequiredBrakingDistance(float velocity, float max_deceleration, float dt) {
    validateMotion(max_deceleration, dt);
    // Add a one-step reaction-distance margin to ideal stopping distance.
    return calculateStoppingDistance(velocity, max_deceleration) + std::abs(velocity) * dt;
}
float calculateAllowedSpeed(float remaining_distance, float max_velocity, float max_acceleration) {
    validateMotion(max_acceleration, 1.0f);
    validateMotion(max_velocity, 1.0f);
    // v^2 <= 2ad gives the speed cap near the target.
    return std::min(max_velocity, std::sqrt(2.0f * max_acceleration * std::abs(remaining_distance)));
}
float calculateDesiredVelocityV2(float current_position, float target_position,
                                 float max_velocity, float max_acceleration) {
    const float delta = target_position - current_position;
    if (delta == 0.0f) return 0.0f;
    return std::copysign(calculateAllowedSpeed(delta, max_velocity, max_acceleration), delta);
}
TrajectoryState predictNextState(const TrajectoryState& current, float next_velocity, float dt) {
    validateMotion(1.0f, dt);
    // Trapezoidal integration assumes linear velocity change over this step.
    return {current.position_deg + 0.5f * (current.velocity_deg_s + next_velocity) * dt,
            next_velocity};
}
bool isBrakingFeasible(const TrajectoryState& state, float target_position, float max_deceleration) {
    validateMotion(max_deceleration, 1.0f);
    const float delta = target_position - state.position_deg;
    if (delta == 0.0f) return state.velocity_deg_s == 0.0f;
    const bool approaching = (delta > 0.0f && state.velocity_deg_s > 0.0f) ||
                             (delta < 0.0f && state.velocity_deg_s < 0.0f);
    // Moving away from target passes this narrow *approach braking* predicate.
    return !approaching || calculateStoppingDistance(state.velocity_deg_s, max_deceleration) <= std::abs(delta);
}
bool isNextStateBrakingFeasible(const TrajectoryState& current, float next_velocity,
                                float target_position, float max_deceleration, float dt) {
    return isBrakingFeasible(predictNextState(current, next_velocity, dt), target_position, max_deceleration);
}
bool isAccelerationFeasible(float current_velocity, float next_velocity,
                            float max_acceleration, float dt) {
    validateMotion(max_acceleration, dt);
    return std::abs(next_velocity - current_velocity) <= max_acceleration * dt + 1e-5f;
}
bool isCandidateVelocityFeasible(const TrajectoryState& current, float next_velocity,
                                 float target_position, float max_acceleration, float dt) {
    return isAccelerationFeasible(current.velocity_deg_s, next_velocity, max_acceleration, dt) &&
           isNextStateBrakingFeasible(current, next_velocity, target_position, max_acceleration, dt);
}
} // namespace robot
