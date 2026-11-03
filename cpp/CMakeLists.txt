cmake_minimum_required(VERSION 3.16)
project(dexterous_hand_cpp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_library(robot_core STATIC
    src/core/RobotJoint.cpp
    src/core/RobotHand.cpp
    src/bus/MockBus.cpp
    src/control/RobotController.cpp
    src/control/PDController.cpp
    src/control/TrajectoryMath.cpp
    src/control/AccelerationLimiter.cpp
    src/control/JointTrajectoryPlanner.cpp
    src/safety/SafetyStateMachine.cpp
)
target_include_directories(robot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(MSVC)
    target_compile_options(robot_core PRIVATE /W4)
else()
    target_compile_options(robot_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
add_executable(robot_control_demo examples/robot_control_demo.cpp)
target_link_libraries(robot_control_demo PRIVATE robot_core)
enable_testing()
add_executable(test_robot_core tests/test_robot_core.cpp)
target_link_libraries(test_robot_core PRIVATE robot_core)
add_test(NAME RobotCoreTest COMMAND test_robot_core)
add_executable(test_s3_trajectory_planner ../tests/cpp/test_s3_trajectory_planner.cpp)
target_link_libraries(test_s3_trajectory_planner PRIVATE robot_core)
add_test(NAME S3TrajectoryPlannerTest COMMAND test_s3_trajectory_planner)
