# S1–S4 C++17 分层控制核心的 CMake 构建脚本。
cmake_minimum_required(VERSION 3.16)
project(dexterous_hand_cpp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Threads REQUIRED)
add_library(robot_core STATIC
    src/core/RobotJoint.cpp
    src/core/RobotHand.cpp
    src/bus/MockBus.cpp
    src/bus/VirtualServoBus.cpp
    src/control/RobotController.cpp
    src/control/PDController.cpp
    src/control/TrajectoryMath.cpp
    src/control/AccelerationLimiter.cpp
    src/control/JointTrajectoryPlanner.cpp
    src/safety/SafetyStateMachine.cpp
    src/safety/StopCoordinator.cpp
    src/runtime/PeriodicScheduler.cpp
    src/runtime/DeadlineMonitor.cpp
    src/runtime/FeedbackBuffer.cpp
    src/runtime/CommandMailbox.cpp
    src/runtime/S4Pipeline.cpp
)
target_include_directories(robot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robot_core PUBLIC Threads::Threads)
if(MSVC)
    target_compile_options(robot_core PRIVATE /W4 /utf-8)
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
add_executable(test_s4_runtime tests/test_s4_runtime.cpp)
target_link_libraries(test_s4_runtime PRIVATE robot_core)
add_test(NAME S4RuntimeTest COMMAND test_s4_runtime)
add_executable(test_s4_integration tests/test_s4_integration.cpp)
target_link_libraries(test_s4_integration PRIVATE robot_core)
add_test(NAME S4IntegrationTest COMMAND test_s4_integration)
