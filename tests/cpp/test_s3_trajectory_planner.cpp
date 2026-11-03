// S3 lesson 39-40：轨迹规划器回归检查。
// 依赖 cpp/include/JointTrajectoryPlanner.h 构建。
// 请以 Debug 模式经 CMake 构建，以便标准 assert() 生效。
#include "JointTrajectoryPlanner.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

namespace {
constexpr float kTolerance = 1e-4f;

// 检查每步速度变化均不超过加速度上限。
void testAccelerationLimit() {
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

// 检查长程规划的速度上限，并打印超调量与最终误差（作为诊断信息）。
void testTrajectoryPlanner() {
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

int main() {
    testAccelerationLimit();
    testTrajectoryPlanner();
    std::cout << "S3 trajectory constraint checks passed\n";
}
