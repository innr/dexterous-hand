#pragma once
namespace robot {
class PDController {
public:
    PDController(float kp, float kd);
    float update(float target_pos, float measured_pos, float measured_vel) const;
private:
    float kp_, kd_;
};
} // namespace robot
