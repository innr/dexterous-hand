#pragma once
// Compatibility umbrella: includes public modules but contains no implementations.
// New code should include the narrow robot/<layer>/<Class>.hpp header.
// Compatibility facade. Prefer importing only the module header you need.
#include "robot/core/RobotJoint.hpp"
#include "robot/core/RobotHand.hpp"
#include "robot/bus/IBus.hpp"
#include "robot/bus/MockBus.hpp"
#include "robot/control/RobotController.hpp"
#include "robot/control/PDController.hpp"
#include "robot/control/TrajectoryMath.hpp"
#include "robot/control/AccelerationLimiter.hpp"
#include "robot/control/JointTrajectoryPlanner.hpp"
#include "robot/safety/SafetyStateMachine.hpp"
