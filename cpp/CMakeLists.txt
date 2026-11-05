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
    src/control/MotionControlCore.cpp
    src/control/ResponseMetrics.cpp
    src/control/VirtualTuningBench.cpp
    src/safety/SafetyStateMachine.cpp
    src/safety/StopCoordinator.cpp
    src/safety/FeedbackSafetyMonitor.cpp
    src/runtime/PeriodicScheduler.cpp
    src/runtime/DeadlineMonitor.cpp
    src/runtime/FeedbackBuffer.cpp
    src/runtime/CommandMailbox.cpp
    src/runtime/S4Pipeline.cpp
    src/runtime/RealtimeExecutor.cpp
    src/runtime/ReadOnlyBench.cpp
    src/runtime/ControlTelemetry.cpp
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

# S5-05：分片轮询、逐路发布、故障隔离与轮转公平性。
add_executable(test_s5_feedback_slicing tests/test_s5_feedback_slicing.cpp)
target_link_libraries(test_s5_feedback_slicing PRIVATE robot_core)
add_test(NAME S5FeedbackSlicingTest COMMAND test_s5_feedback_slicing)

# S5-06：反馈超时分级、运动发送门禁与 SAFE_STOP 锁存。
add_executable(test_s5_feedback_safety tests/test_s5_feedback_safety.cpp)
target_link_libraries(test_s5_feedback_safety PRIVATE robot_core)
add_test(NAME S5FeedbackSafetyTest COMMAND test_s5_feedback_safety)

# S5-07：16路位置外环PD、轨迹与最终输出限速/限加速度。
add_executable(test_s5_motion_control tests/test_s5_motion_control.cpp)
target_link_libraries(test_s5_motion_control PRIVATE robot_core)
add_test(NAME S5MotionControlTest COMMAND test_s5_motion_control)

# S5-08：离线阶跃响应分析与确定性虚拟模型 PD 参数对比。
add_executable(test_s5_performance tests/test_s5_performance.cpp)
target_link_libraries(test_s5_performance PRIVATE robot_core)
add_test(NAME S5PerformanceMetricsTest COMMAND test_s5_performance)
add_executable(s5_response_analyzer examples/s5_response_analyzer.cpp)
target_link_libraries(s5_response_analyzer PRIVATE robot_core)
add_executable(s5_virtual_tuning examples/s5_virtual_tuning.cpp)
target_link_libraries(s5_virtual_tuning PRIVATE robot_core)

# S5-08 CLI 端到端：仿真导出 CSV -> 同一 CSV 由分析器成功解析。
set(S5_TUNING_CSV "${CMAKE_CURRENT_BINARY_DIR}/s5_tuning_best.csv")
add_test(NAME S5VirtualTuningCliTest COMMAND s5_virtual_tuning "${S5_TUNING_CSV}")
set_tests_properties(S5VirtualTuningCliTest PROPERTIES FIXTURES_SETUP S5_TUNING_CSV_READY)
add_test(NAME S5ResponseAnalyzerCliTest COMMAND s5_response_analyzer "${S5_TUNING_CSV}")
set_tests_properties(S5ResponseAnalyzerCliTest PROPERTIES FIXTURES_REQUIRED S5_TUNING_CSV_READY)

# S5-09：非阻塞100Hz环形遥测，虚拟总线CSV端到端。
add_executable(test_s5_telemetry tests/test_s5_telemetry.cpp)
target_link_libraries(test_s5_telemetry PRIVATE robot_core)
add_test(NAME S5TelemetryTest COMMAND test_s5_telemetry)
add_executable(s5_telemetry_demo examples/s5_telemetry_demo.cpp)
target_link_libraries(s5_telemetry_demo PRIVATE robot_core)
set(S5_TELEMETRY_CSV "${CMAKE_CURRENT_BINARY_DIR}/s5_telemetry.csv")
add_test(NAME S5TelemetryDemoTest COMMAND s5_telemetry_demo "${S5_TELEMETRY_CSV}")
set_tests_properties(S5TelemetryDemoTest PROPERTIES FIXTURES_SETUP S5_TELEMETRY_CSV_READY)
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_test(NAME S5TelemetryPlotTest
             COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/plot_s5_telemetry.py"
                     "${S5_TELEMETRY_CSV}" --joint 0
                     --output "${CMAKE_CURRENT_BINARY_DIR}/s5_telemetry.html")
    set_tests_properties(S5TelemetryPlotTest PROPERTIES FIXTURES_REQUIRED S5_TELEMETRY_CSV_READY)
endif()

# S5 遗留：整批 TX 中途过期后，逐关节 TTL 截断。
add_executable(test_s5_command_ttl tests/test_s5_command_ttl.cpp)
target_link_libraries(test_s5_command_ttl PRIVATE robot_core)
add_test(NAME S5CommandTtlTest COMMAND test_s5_command_ttl)

# S6-01：16 路 MuJoCo 与现有 C++17 控制核心的进程协议适配。
add_executable(s6_control_bridge examples/s6_control_bridge.cpp)
target_link_libraries(s6_control_bridge PRIVATE robot_core)
if(Python3_Interpreter_FOUND)
    add_test(NAME S6BridgeProtocolTest
             COMMAND "${Python3_EXECUTABLE}"
                     "${CMAKE_CURRENT_SOURCE_DIR}/../tests/test_s6_bridge_protocol.py"
                     "$<TARGET_FILE:s6_control_bridge>")
    execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import mujoco, numpy"
                    RESULT_VARIABLE S6_MUJOCO_IMPORT_RESULT
                    OUTPUT_QUIET ERROR_QUIET)
    if(S6_MUJOCO_IMPORT_RESULT EQUAL 0)
        add_test(NAME S6MuJoCoSmokeTest
                 COMMAND "${Python3_EXECUTABLE}" -m simulation.s6_mujoco_bridge
                         --bridge "$<TARGET_FILE:s6_control_bridge>"
                         --seconds 0.6 --assert-motion)
        set_tests_properties(S6MuJoCoSmokeTest PROPERTIES
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/..")
    endif()
endif()

# S6-02：官方 Menagerie LEAP MJCF 仿真安全会话回归（需先下载 pinned assets）。
if(Python3_Interpreter_FOUND AND S6_MUJOCO_IMPORT_RESULT EQUAL 0)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/mujoco_menagerie/leap_hand/right_hand.xml")
        add_test(NAME S6OfficialModelTest
                 COMMAND "${Python3_EXECUTABLE}"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../tests/test_s6_official_bridge.py"
                         "$<TARGET_FILE:s6_control_bridge>")
        set_tests_properties(S6OfficialModelTest PROPERTIES
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/..")
    endif()
endif()
