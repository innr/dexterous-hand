// S3 lessons 39-40: trajectory planner regression checks.
//
// Integration prerequisite:
//   Provide a C++17 JointTrajectoryPlanner.h exposing:
//     struct TrajectoryState { float position_deg; float velocity_deg_s; };
//     JointTrajectoryPlanner(float initial_position,
//                            float max_velocity,
//                            float max_acceleration);
//     TrajectoryState update(float target_position, float dt);
//
// This repository does not yet have that C++ header/implementation.
// The test is intentionally NOT registered in CMake or claimed to pass.
// Once the implementation is added, compile in a Debug/test build (assert enabled).
#include "JointTrajectoryPlanner.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

namespace {
constexpr float kTolerance = 1e-4f;

void testAccelerationLimit()
{
    constexpr float dt = 0.01f;
    constexpr float max_acceleration = 40.0f;
    constexpr float max_velocity = 20.0f;

    JointTrajectoryPlanner planner(20.0f, max_velocity, max_acceleration);
    float previous_velocity = 0.0f;

    for (int i = 0; i < 500; ++i) {
        const TrajectoryState state = planner.update(60.0f, dt);
        assert(std::isfinite(state.position_deg));
        assert(std::isfinite(state.velocity_deg_s));

        const float delta_velocity =
            state.velocity_deg_s - previous_velocity;
        const float max_delta = max_acceleration * dt;

        assert(std::abs(delta_velocity) <= max_delta + kTolerance);
        previous_velocity = state.velocity_deg_s;
    }
}

void testTrajectoryPlanner()
{
    constexpr float dt = 0.01f;
    constexpr float max_velocity = 20.0f;
    constexpr float max_acceleration = 40.0f;
    constexpr float initial_position = 20.0f;
    constexpr float target_position = 60.0f;

    JointTrajectoryPlanner planner(
        initial_position, max_velocity, max_acceleration);

    float previous_velocity = 0.0f;
    float max_position = initial_position;
    TrajectoryState last_state{initial_position, 0.0f};

    for (int i = 0; i < 500; ++i) {
        const TrajectoryState state =
            planner.update(target_position, dt);

        assert(std::isfinite(state.position_deg));
        assert(std::isfinite(state.velocity_deg_s));

        // TODO 1: speed bound
        assert(std::abs(state.velocity_deg_s) <=
               max_velocity + kTolerance);

        // TODO 2: acceleration bound (missing in original submission)
        const float delta_velocity =
            state.velocity_deg_s - previous_velocity;
        assert(std::abs(delta_velocity) <=
               max_acceleration * dt + kTolerance);

        // TODO 3: track maximum position for overshoot measurement
        max_position = std::max(max_position, state.position_deg);

        // TODO 4: retain previous velocity for the next iteration
        previous_velocity = state.velocity_deg_s;
        last_state = state;
    }

    // TODO 5: calculate overshoot (diagnostic, not yet a pass/fail limit)
    const float overshoot =
        std::max(0.0f, max_position - target_position);

    // TODO 6: report maximum overshoot and terminal tracking error
    std::cout << "Max overshoot: " << overshoot << " degrees\n";
    std::cout << "Final position error: "
              << std::abs(last_state.position_deg - target_position)
              << " degrees\n";
}
} // namespace

int main()
{
    testAccelerationLimit();
    testTrajectoryPlanner();
    std::cout << "S3 trajectory constraint checks passed\n";
    return 0;
}
