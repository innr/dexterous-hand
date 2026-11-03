#pragma once
// S3 | Pure trajectory math and feasibility predicates (units: deg, deg/s, deg/s^2).
// Braking predicates are local checks, NOT a guarantee of safe arrival.
// All time arguments are in seconds; use dt=0.01f for a nominal 100 Hz step.
namespace robot {
struct TrajectoryState {
    float position_deg = 0.0f;
    float velocity_deg_s = 0.0f;
};
void validateMotion(float limit, float dt);
float calculateStoppingDistance(float velocity, float max_deceleration);
float calculateRequiredBrakingDistance(float velocity, float max_deceleration, float dt);
float calculateAllowedSpeed(float remaining_distance, float max_velocity, float max_acceleration);
float calculateDesiredVelocityV2(float current_position, float target_position,
                                 float max_velocity, float max_acceleration);
TrajectoryState predictNextState(const TrajectoryState& current, float next_velocity, float dt);
bool isBrakingFeasible(const TrajectoryState& state, float target_position, float max_deceleration);
bool isNextStateBrakingFeasible(const TrajectoryState& current, float next_velocity,
                                float target_position, float max_deceleration, float dt);
bool isAccelerationFeasible(float current_velocity, float next_velocity,
                            float max_acceleration, float dt);
bool isCandidateVelocityFeasible(const TrajectoryState& current, float next_velocity,
                                 float target_position, float max_acceleration, float dt);
} // namespace robot
