# S3 trajectory planner regression checks

The C++ implementation now exists in `cpp/src/control/` and is linked through the `robot_core` STATIC library.

```sh
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Debug
cmake --build build/cpp --parallel
ctest --test-dir build/cpp --output-on-failure
```

`test_s3_trajectory_planner.cpp` checks 500 updates at 10 ms, finite state, max velocity and per-step acceleration, and prints overshoot/final error. The tests use `assert`; build with assertions enabled (Debug).

Overshoot and final position error remain diagnostics, not acceptance thresholds. More coverage is needed for target changes, reverse motion, time jitter, actuator feedback lag, and emergency-stop behavior.
