#include "robot/core/RobotJoint.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot {

RobotJoint::RobotJoint(int id, float min_deg, float max_deg)
    : id_(id), min_(min_deg), max_(max_deg), position_(0.0f) {
    // 校验 ID 与限位合法性：ID 在 [0, kJointCount)，限位须有限且 min <= max。
    if (id < 0 || id >= kJointCount || !std::isfinite(min_deg) ||
        !std::isfinite(max_deg) || min_deg > max_deg)
        throw std::invalid_argument("invalid joint limits/id");
    position_ = std::clamp(0.0f, min_, max_);
}

int RobotJoint::getId() const { return id_; }

float RobotJoint::getPosition() const { return position_; }

float RobotJoint::clamp(float value) const { return std::clamp(value, min_, max_); }

void RobotJoint::setPosition(float value) {
    // 拒绝 NaN/Inf，其余值一律裁剪到限位范围，保证状态始终在范围内。
    if (!std::isfinite(value)) throw std::invalid_argument("nonfinite joint command");
    position_ = clamp(value);
}

} // namespace robot
