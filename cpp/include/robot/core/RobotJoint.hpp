#pragma once
// S1 | Single-joint model: validates ID/limits and clamps commanded position.
// getPosition() returns the last commanded value, not encoder feedback.

namespace robot {
inline constexpr int kJointCount = 16;
class RobotJoint {
public:
    RobotJoint(int id, float min_deg, float max_deg);
    int getId() const;
    float getPosition() const;
    float clamp(float value) const;
    void setPosition(float value);
private:
    int id_;
    float min_, max_, position_;
};
} // namespace robot
