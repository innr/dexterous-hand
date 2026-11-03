#pragma once
// S3 | Stateful single-joint position planner with bounded velocity changes.
// update() advances one discrete dt step; target can change between calls.
// Current planner is educational: no proven overshoot/settling guarantees.
#include "robot/control/AccelerationLimiter.hpp"
#include "robot/control/TrajectoryMath.hpp"
namespace robot {
class JointTrajectoryPlanner {
public:
    JointTrajectoryPlanner(float initial_position, float max_velocity, float max_acceleration);
    TrajectoryState update(float target_position, float dt);
    TrajectoryState state() const;
private:
    float max_velocity_, max_acceleration_;
    AccelerationLimiter limiter_;
    TrajectoryState state_;
};
} // namespace robot
