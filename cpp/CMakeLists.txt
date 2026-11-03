# S1–S3 C++17 分层控制核心的 CMake 构建脚本。
cmake_minimum_required(VERSION 3.16)
project(dexterous_hand_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 静态库：核心控制逻辑（分区在 core / bus / control / safety）。
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

# 严格告警：MSVC 启用 /W4，其他编译器启用 -Wall -Wextra -Wpedantic -Werror。
if(MSVC)
    # /utf-8：源码含中文注释，显式按 UTF-8 解析，避免代码页告警。
    target_compile_options(robot_core PRIVATE /W4 /utf-8)
else()
    target_compile_options(robot_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# 演示程序
add_executable(robot_control_demo examples/robot_control_demo.cpp)
target_link_libraries(robot_control_demo PRIVATE robot_core)

# 回归测试
enable_testing()

add_executable(test_robot_core tests/test_robot_core.cpp)
target_link_libraries(test_robot_core PRIVATE robot_core)
add_test(NAME RobotCoreTest COMMAND test_robot_core)

add_executable(test_s3_trajectory_planner ../tests/cpp/test_s3_trajectory_planner.cpp)
target_link_libraries(test_s3_trajectory_planner PRIVATE robot_core)
add_test(NAME S3TrajectoryPlannerTest COMMAND test_s3_trajectory_planner)
