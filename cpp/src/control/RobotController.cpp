#include "robot/control/RobotController.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace robot {

RobotController::RobotController(std::unique_ptr<IBus> bus) : bus_(std::move(bus)) {
    if (!bus_) throw std::invalid_argument("bus is null");
}

bool RobotController::setJointPosition(int id, float pos) {
    // 先校验 ID 范围与角度有限性。
    if (id < 0 || id >= kJointCount || !std::isfinite(pos)) return false;

    const auto index = static_cast<std::size_t>(id);
    const float limited = hand_.clamp(index, pos);
    if (!bus_->send(id, limited)) return false; // 总线下发失败
    hand_.setPosition(index, limited);
    return true;
}

float RobotController::commandedPosition(std::size_t id) const { return hand_.getPosition(id); }

} // namespace robot
