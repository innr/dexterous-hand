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
    src/bus/Sts3215Protocol.cpp
    src/bus/Sts3215Transport.cpp
    src/bus/PosixHalfDuplexPort.cpp
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
    src/runtime/RealtimeExecutor.cpp
    src/runtime/ReadOnlyBench.cpp
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

# S5-01：停机与 TX 启动线性化点、多线程竞态回归。
add_executable(test_s5_stop tests/test_s5_stop.cpp)
target_link_libraries(test_s5_stop PRIVATE robot_core)
add_test(NAME S5StopCoordinatorTest COMMAND test_s5_stop)

# S5-02：真实线程生命周期、周期指标与慢总线隔离回归。
add_executable(test_s5_runtime tests/test_s5_runtime.cpp)
target_link_libraries(test_s5_runtime PRIVATE robot_core)
add_test(NAME S5RealtimeExecutorTest COMMAND test_s5_runtime)

# S5-03：STS3215 数据包、串口协议与故障注入测试。
add_executable(test_s5_sts3215 tests/test_s5_sts3215.cpp)
target_link_libraries(test_s5_sts3215 PRIVATE robot_core)
add_test(NAME S5Sts3215ProtocolTest COMMAND test_s5_sts3215)

# 台架只读工具：必须手动传入串口与 16 路 ID；不会使能扭矩。
add_executable(sts3215_readonly_probe examples/sts3215_readonly_probe.cpp)
target_link_libraries(sts3215_readonly_probe PRIVATE robot_core)

# S5-04：只读 16 路轮询和独立控制观察线程的时序回归。
add_executable(test_s5_bench tests/test_s5_bench.cpp)
target_link_libraries(test_s5_bench PRIVATE robot_core)
add_test(NAME S5ReadOnlyBenchTest COMMAND test_s5_bench)

add_executable(sts3215_bench examples/sts3215_bench.cpp)
target_link_libraries(sts3215_bench PRIVATE robot_core)
