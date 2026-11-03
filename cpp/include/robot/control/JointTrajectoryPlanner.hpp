#pragma once

#include "robot/control/AccelerationLimiter.hpp"
#include "robot/control/TrajectoryMath.hpp"

// 关节轨迹规划器：把阶跃目标平滑成受最大速度/加速度约束的轨迹。
namespace robot {

class JointTrajectoryPlanner {
public:
    // initial_position 为起始角度；max_velocity/max_acceleration 必须为有限正值。
    JointTrajectoryPlanner(float initial_position, float max_velocity, float max_acceleration);

    // 以 dt 为步长更新一步，返回新的轨迹状态（位置、速度）。
    TrajectoryState update(float target_position, float dt);

    TrajectoryState state() const;

private:
    float max_velocity_;
    float max_acceleration_;
    AccelerationLimiter limiter_;
    TrajectoryState state_;
};

} // namespace robot
