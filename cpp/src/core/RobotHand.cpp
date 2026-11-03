// S1 | 16-joint aggregate: storage and checked access, no communication.
#include "robot/core/RobotHand.hpp"
namespace robot {
RobotHand::RobotHand(float min_deg, float max_deg) {
    // Reserve all slots up front to avoid reallocations while constructing joints.
    joints_.reserve(kJointCount);
    for (int i = 0; i < kJointCount; ++i)
        joints_.emplace_back(i, min_deg, max_deg);
}
void RobotHand::setPosition(std::size_t id, float pos) { joints_.at(id).setPosition(pos); }
float RobotHand::getPosition(std::size_t id) const { return joints_.at(id).getPosition(); }
float RobotHand::clamp(std::size_t id, float pos) const { return joints_.at(id).clamp(pos); }
} // namespace robot
