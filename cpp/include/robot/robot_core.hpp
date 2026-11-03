#pragma once

// 兼容聚合头：一次性引入全部模块。新代码建议只包含所需的具体模块头。
#include "robot/bus/IBus.hpp"
#include "robot/bus/MockBus.hpp"
#include "robot/control/AccelerationLimiter.hpp"
#include "robot/control/JointTrajectoryPlanner.hpp"
#include "robot/control/PDController.hpp"
#include "robot/control/RobotController.hpp"
#include "robot/control/TrajectoryMath.hpp"
#include "robot/core/RobotHand.hpp"
#include "robot/core/RobotJoint.hpp"
#include "robot/safety/SafetyStateMachine.hpp"
