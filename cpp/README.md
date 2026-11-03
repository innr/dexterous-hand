# C++17 robot control learning core (S1–S3)

Standalone educational baseline reconstructed from S1–S3 exercises. It does not replace the existing Python simulation or reproduce unprovided historical local files verbatim.

## Architecture

```text
cpp/
  CMakeLists.txt
  include/JointTrajectoryPlanner.h
  include/robot/robot_core.hpp
  examples/robot_control_demo.cpp
  tests/test_robot_core.cpp
tests/cpp/test_s3_trajectory_planner.cpp
docs/architecture/cpp_control_architecture.md
```

## Build & test

```bash
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Debug
cmake --build build/cpp
ctest --test-dir build/cpp --output-on-failure
./build/cpp/robot_control_demo
```

The C++ core is header-only for educational clarity. The synchronous IBus is a mock teaching interface, NOT a real-time 100 Hz hardware bus design. Tests validate finite values, speed and acceleration bounds, joint limits, mock failures, PD math, braking checks, and basic state transitions. Overshoot is measured but not acceptance-tested.
