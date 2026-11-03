#pragma once
// S3 | Discrete PD effort: kp * position_error - kd * measured_velocity.
// The returned value is a control effort, not a position or a serial command.

namespace robot {
class PDController {
public:
    PDController(float kp, float kd);
    float update(float target_pos, float measured_pos, float measured_vel) const;
private:
    float kp_, kd_;
};
} // namespace robot
