# S3 C++17 trajectory planner tests (lessons 39–40)

This folder preserves the latest exercises from the S3 robotics control curriculum in executable-test form.

## Contents

- `test_s3_trajectory_planner.cpp`: 500 updates at 10 ms from 20° toward 60°; checks finite outputs, speed bound (`20°/s`), and per-step acceleration bound (`40°/s²`); records maximum overshoot and final position error.
- The missing acceleration assertion from the last lesson has been added.
- Overshoot and final position error are **diagnostics**, not acceptance thresholds. Additional requirements are needed before declaring motion safe.

## Integration status

The current repository is primarily Python and **does not contain** the `JointTrajectoryPlanner.h` C++ implementation used in the lessons. Consequently this test is **not wired into CI or CTest**, and it has **not been compiled or executed against the actual planner**.

Once the C++17 planner is added, expose the interface documented at the top of the test file, then build with a command like:

```sh
c++ -std=c++17 -Wall -Wextra -Werror -I<planner-include-dir> \
    tests/cpp/test_s3_trajectory_planner.cpp \
    <planner-source-files> -o test_s3_trajectory_planner
./test_s3_trajectory_planner
```

Replace the placeholders with the real include directory and implementation sources. Keep `assert` enabled (do not define `NDEBUG`), or migrate to a test framework such as GoogleTest.

## Known gaps

- No hard overshoot/settling acceptance criteria yet.
- No reverse motion, target change, joint limits, communication lag, or emergency-stop tests yet.
- Constraint predicates alone do not guarantee that a feasible next command exists or that the actual servo follows the planned state.
