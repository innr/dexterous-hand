#pragma once

// 轨迹数学工具：停车距离、可达速度、状态预测与可行性判定等纯函数。
namespace robot {

// 轨迹状态：位置（度）与速度（度/秒）。
struct TrajectoryState {
    float position_deg = 0.0f;
    float velocity_deg_s = 0.0f;
};

// 校验运动限制 dt/limit 为正且有限，否则抛 std::invalid_argument。
void validateMotion(float limit, float dt);

// 由当前速度与最大减速度估算刹车距离：v^2 / (2a)。
float calculateStoppingDistance(float velocity, float max_deceleration);

// 在刹车距离基础上加一个控制周期 dt 内继续前进的距离。
float calculateRequiredBrakingDistance(float velocity, float max_deceleration, float dt);

// 给定剩余距离，计算不超过 max_velocity 的安全速度上限：min(max_v, sqrt(2a*|d|))。
float calculateAllowedSpeed(float remaining_distance, float max_velocity, float max_acceleration);

// 由当前位置与目标位置计算带方向的期望速度。
float calculateDesiredVelocityV2(float current_position, float target_position,
                                 float max_velocity, float max_acceleration);

// 用梯形积分由当前状态和下一速度预测下一状态。
TrajectoryState predictNextState(const TrajectoryState& current, float next_velocity, float dt);

// 判断按当前速度能否在到达目标前刹停。
bool isBrakingFeasible(const TrajectoryState& state, float target_position, float max_deceleration);

// 判断以 next_velocity 走一步后的状态能否刹停。
bool isNextStateBrakingFeasible(const TrajectoryState& current, float next_velocity,
                                float target_position, float max_deceleration, float dt);

// 判断速度变化是否满足加速度上限（含 1e-5 浮点容差）。
bool isAccelerationFeasible(float current_velocity, float next_velocity,
                            float max_acceleration, float dt);

// 候选速度的完整可行性：同时满足加速度约束与一步后刹车约束。
bool isCandidateVelocityFeasible(const TrajectoryState& current, float next_velocity,
                                 float target_position, float max_acceleration, float dt);

} // namespace robot
