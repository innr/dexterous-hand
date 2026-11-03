#include "robot/control/TrajectoryMath.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot {

void validateMotion(float limit, float dt) {
    // 限制值与步长都必须为有限正值。
    if (!std::isfinite(limit) || limit <= 0.0f || !std::isfinite(dt) || dt <= 0.0f)
        throw std::invalid_argument("motion limit/dt must be positive and finite");
}

float calculateStoppingDistance(float velocity, float max_deceleration) {
    validateMotion(max_deceleration, 1.0f);
    // 匀减速刹车距离：v^2 / (2a)。
    return velocity * velocity / (2.0f * max_deceleration);
}

float calculateRequiredBrakingDistance(float velocity, float max_deceleration, float dt) {
    validateMotion(max_deceleration, dt);
    // 刹车距离 + 一个控制周期内的位移。
    return calculateStoppingDistance(velocity, max_deceleration) + std::abs(velocity) * dt;
}

float calculateAllowedSpeed(float remaining_distance, float max_velocity, float max_acceleration) {
    validateMotion(max_acceleration, 1.0f);
    validateMotion(max_velocity, 1.0f);
    // 由 v^2 = 2a*d 反解，并限制不超过最大速度。
    return std::min(max_velocity, std::sqrt(2.0f * max_acceleration * std::abs(remaining_distance)));
}

float calculateDesiredVelocityV2(float current_position, float target_position,
                                 float max_velocity, float max_acceleration) {
    const float delta = target_position - current_position;
    if (delta == 0.0f) return 0.0f;
    // 速度大小由剩余距离决定，方向与位置误差一致。
    return std::copysign(calculateAllowedSpeed(delta, max_velocity, max_acceleration), delta);
}

TrajectoryState predictNextState(const TrajectoryState& current, float next_velocity, float dt) {
    validateMotion(1.0f, dt);
    // 梯形积分：位置增量 = 平均速度 * dt。
    return {current.position_deg + 0.5f * (current.velocity_deg_s + next_velocity) * dt,
            next_velocity};
}

bool isBrakingFeasible(const TrajectoryState& state, float target_position, float max_deceleration) {
    validateMotion(max_deceleration, 1.0f);
    const float delta = target_position - state.position_deg;
    if (delta == 0.0f) return state.velocity_deg_s == 0.0f;

    // 只有“正朝目标方向运动”时才存在刹不住的风险。
    const bool approaching = (delta > 0.0f && state.velocity_deg_s > 0.0f) ||
                             (delta < 0.0f && state.velocity_deg_s < 0.0f);
    return !approaching || calculateStoppingDistance(state.velocity_deg_s, max_deceleration) <= std::abs(delta);
}

bool isNextStateBrakingFeasible(const TrajectoryState& current, float next_velocity,
                                float target_position, float max_deceleration, float dt) {
    // 先预测走一步后的状态，再判断该状态能否刹停。
    return isBrakingFeasible(predictNextState(current, next_velocity, dt), target_position, max_deceleration);
}

bool isAccelerationFeasible(float current_velocity, float next_velocity,
                            float max_acceleration, float dt) {
    validateMotion(max_acceleration, dt);
    // 速度变化不得超过 a*dt（含 1e-5 浮点容差）。
    return std::abs(next_velocity - current_velocity) <= max_acceleration * dt + 1e-5f;
}

bool isCandidateVelocityFeasible(const TrajectoryState& current, float next_velocity,
                                 float target_position, float max_acceleration, float dt) {
    // 候选速度需同时满足加速度约束与一步后刹车约束。
    return isAccelerationFeasible(current.velocity_deg_s, next_velocity, max_acceleration, dt) &&
           isNextStateBrakingFeasible(current, next_velocity, target_position, max_acceleration, dt);
}

} // namespace robot
