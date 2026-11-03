// S3 lessons 39-40: trajectory planner regression checks.
// Builds against cpp/include/JointTrajectoryPlanner.h.
// Build with CMake in Debug mode so standard assert() remains enabled.
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
        const float delta_velocity = state.velocity_deg_s - previous_velocity;
        assert(std::abs(delta_velocity) <= max_acceleration * dt + kTolerance);
        previous_velocity = state.velocity_deg_s;
    }
}

void testTrajectoryPlanner()
{
    constexpr float dt = 0.01f;
    constexpr float max_velocity = 20.0f;
    constexpr float max_acceleration = 40.0f;
    constexpr float target_position = 60.0f;
    JointTrajectoryPlanner planner(20.0f, max_velocity, max_acceleration);
    float previous_velocity = 0.0f;
    float max_position = 20.0f;
    TrajectoryState last_state{20.0f, 0.0f};
    for (int i = 0; i < 500; ++i) {
        const TrajectoryState state = planner.update(target_position, dt);
        assert(std::isfinite(state.position_deg));
        assert(std::isfinite(state.velocity_deg_s));
        assert(std::abs(state.velocity_deg_s) <= max_velocity + kTolerance);
        const float delta_velocity = state.velocity_deg_s - previous_velocity;
        assert(std::abs(delta_velocity) <= max_acceleration * dt + kTolerance);
        max_position = std::max(max_position, state.position_deg);
        previous_velocity = state.velocity_deg_s;
        last_state = state;
    }
    const float overshoot = std::max(0.0f, max_position - target_position);
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
}
