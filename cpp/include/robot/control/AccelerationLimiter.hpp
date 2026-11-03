#pragma once
// S3 | Rate-limits changes in velocity: |v_next-v_prev| <= a_max*dt.
// This limits planned acceleration; it does not measure actual servo acceleration.
namespace robot {
class AccelerationLimiter {
public:
    explicit AccelerationLimiter(float max_acceleration);
    float update(float desired_velocity, float dt);
    float velocity() const;
private:
    float max_acceleration_;
    float velocity_ = 0.0f;
};
} // namespace robot
