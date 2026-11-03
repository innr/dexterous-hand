#pragma once
#include "robot/core/RobotJoint.hpp"
#include <cstddef>
#include <vector>
namespace robot {
class RobotHand {
public:
    RobotHand(float min_deg = -90.0f, float max_deg = 90.0f);
    void setPosition(std::size_t id, float pos);
    float getPosition(std::size_t id) const;
    float clamp(std::size_t id, float pos) const;
private:
    std::vector<RobotJoint> joints_;
};
} // namespace robot
