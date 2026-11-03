#include "robot/core/RobotHand.hpp"

namespace robot {

RobotHand::RobotHand(float min_deg, float max_deg) {
    // 预分配并依次构造 16 个关节，全部使用同一组限位。
    joints_.reserve(kJointCount);
    for (int i = 0; i < kJointCount; ++i)
        joints_.emplace_back(i, min_deg, max_deg);
}

void RobotHand::setPosition(std::size_t id, float pos) { joints_.at(id).setPosition(pos); }

float RobotHand::getPosition(std::size_t id) const { return joints_.at(id).getPosition(); }

float RobotHand::clamp(std::size_t id, float pos) const { return joints_.at(id).clamp(pos); }

} // namespace robot
