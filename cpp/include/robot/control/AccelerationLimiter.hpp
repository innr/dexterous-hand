#pragma once

// 加速度限制器：把速度指令的变化率约束在 max_acceleration 以内。
namespace robot {

class AccelerationLimiter {
public:
    // max_acceleration 必须为有限正值。
    explicit AccelerationLimiter(float max_acceleration);

    // 以 dt 为步长向 desired_velocity 逼近，返回限加速度后的新速度。
    float update(float desired_velocity, float dt);

    float velocity() const;

private:
    float max_acceleration_;
    float velocity_ = 0.0f; // 当前速度（内部状态）
};

} // namespace robot
