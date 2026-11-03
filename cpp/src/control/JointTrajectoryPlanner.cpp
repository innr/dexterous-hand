#include "robot/control/JointTrajectoryPlanner.hpp"

#include <cmath>
#include <stdexcept>

namespace robot {

JointTrajectoryPlanner::JointTrajectoryPlanner(float initial_position, float max_velocity,
                                               float max_acceleration)
    : max_velocity_(max_velocity), max_acceleration_(max_acceleration),
      limiter_(max_acceleration), state_{initial_position, 0.0f} {
    // 校验速度/加速度限制，以及初始位置是否有限。
    validateMotion(max_velocity_, 1.0f);
    validateMotion(max_acceleration_, 1.0f);
    if (!std::isfinite(initial_position)) throw std::invalid_argument("invalid initial position");
}

TrajectoryState JointTrajectoryPlanner::update(float target_position, float dt) {
    validateMotion(max_acceleration_, dt);
    if (!std::isfinite(target_position)) throw std::invalid_argument("invalid target");

    // 1) 由剩余距离算出期望速度；2) 经加速度限制器平滑；3) 预测下一状态。
    const float desired = calculateDesiredVelocityV2(state_.position_deg, target_position,
                                                     max_velocity_, max_acceleration_);
    const float next_velocity = limiter_.update(desired, dt);
    state_ = predictNextState(state_, next_velocity, dt);
    return state_;
}

TrajectoryState JointTrajectoryPlanner::state() const { return state_; }

} // namespace robot
