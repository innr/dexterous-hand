#include "robot/control/PDController.hpp"
#include <cmath>
#include <stdexcept>
namespace robot {
PDController::PDController(float kp, float kd) : kp_(kp), kd_(kd) {
    if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0 || kd < 0)
        throw std::invalid_argument("invalid gain");
}
float PDController::update(float target_pos, float measured_pos, float measured_vel) const {
    return kp_ * (target_pos - measured_pos) - kd_ * measured_vel;
}
} // namespace robot
