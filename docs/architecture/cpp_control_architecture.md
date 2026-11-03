# LEAP dexterous hand — C++ S1–S3 layered architecture

## Dependency flow

```text
Application / examples
         |
         v
control/JointTrajectoryPlanner ---> control/AccelerationLimiter
         |                                   |
         +----------------> control/TrajectoryMath
         |
         v
control/PDController     safety/SafetyStateMachine
         |                          |
         +-----------+--------------+
                     |
                     v
           control/RobotController
                |           |
                v           v
          core/RobotHand   bus/IBus (interface)
                |           |
                v           v
          core/RobotJoint  bus/MockBus (test only)
```

The C++ source tree follows `include/robot/{core,control,safety,bus}` and `src/{core,control,safety,bus}`. Every implementation lives in its own `.cpp` translation unit. The umbrella `robot_core.hpp` is a compatibility header only.

## Responsibilities

| Layer | Responsibilities | Does not own |
|---|---|---|
| core | 16 joint IDs, commanded position, limits | bus I/O |
| control | PD, speed/acceleration constraints, trajectory, controller orchestration | serial protocol |
| safety | NORMAL/DEGRADED/SAFE_STOP/FAULT demonstration | hardware power cutoff |
| bus | abstract send interface, mock adapter | motion planning |
| examples/tests | wiring, unit/regression checks | production scheduling |

## S4 planned, NOT implemented

```text
ROS2 / teleoperation command
         |
         v
100 Hz control worker --> safety gate --> latest-command mailbox
         ^                                   |
         |                                   v
timestamped feedback snapshot <-- STS3215 communication worker
```

16-servo polling (~11.2 ms) can exceed the 10 ms control period. S4 must avoid blocking the control loop on full feedback, validate feedback age, and define stop-priority/in-flight TX semantics.

## Validation

Build with `cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Debug`, then `cmake --build build/cpp` and `ctest --test-dir build/cpp --output-on-failure`.

The S3 overshoot metric is diagnostic; it is **not** proof of safety or target convergence. The synchronous `RobotController` tracks commands, not physical feedback. The actual STS3215 hardware interface is not present.
