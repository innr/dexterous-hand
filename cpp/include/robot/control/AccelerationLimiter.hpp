#pragma once
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
