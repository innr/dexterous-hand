# S1–S3 C++17 layered control core

The C++ teaching code now uses **separate public headers and .cpp implementations** instead of a monolithic header.

## Layout

```text
cpp/
├── include/robot/
│   ├── core/      RobotJoint.hpp, RobotHand.hpp
│   ├── bus/       IBus.hpp, MockBus.hpp
│   ├── control/   RobotController.hpp, PDController.hpp,
│   │              TrajectoryMath.hpp, AccelerationLimiter.hpp,
│   │              JointTrajectoryPlanner.hpp
│   └── safety/    SafetyStateMachine.hpp
├── src/
│   ├── core/      RobotJoint.cpp, RobotHand.cpp
│   ├── bus/       MockBus.cpp
│   ├── control/   RobotController.cpp, PDController.cpp,
│   │              TrajectoryMath.cpp, AccelerationLimiter.cpp,
│   │              JointTrajectoryPlanner.cpp
│   └── safety/    SafetyStateMachine.cpp
├── examples/robot_control_demo.cpp
├── tests/test_robot_core.cpp
└── CMakeLists.txt
tests/cpp/test_s3_trajectory_planner.cpp
```

`robot/robot_core.hpp` and `JointTrajectoryPlanner.h` are **compatibility include shims**, not implementations. Prefer module-specific headers in new code.

## Build and test

```sh
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Debug
cmake --build build/cpp --parallel
ctest --test-dir build/cpp --output-on-failure
./build/cpp/robot_control_demo
```

The library is a **STATIC** CMake target with PUBLIC includes. The demo and two existing regression test executables link against it.

## Scope and limitations

This is a **behavior-preserving organization refactor** of the S1–S3 educational core. The mock bus remains synchronous. The trajectory planner is nominal and is not a verified safe motion planner; braking predicates do not guarantee no overshoot. No STS3215 serial driver, 100 Hz asynchronous scheduler, hardware emergency stop, ROS2 adapter or MuJoCo adapter has been implemented here. Existing Python simulation and hardware code remain unchanged.
