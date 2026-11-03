#pragma once
// Compatibility alias for the original S3 exercise test.
// New code should use robot/control/JointTrajectoryPlanner.hpp directly.
// Legacy lesson-39/40 include; use robot/control/JointTrajectoryPlanner.hpp in new code.
#include "robot/control/JointTrajectoryPlanner.hpp"
using TrajectoryState = robot::TrajectoryState;
using JointTrajectoryPlanner = robot::JointTrajectoryPlanner;
