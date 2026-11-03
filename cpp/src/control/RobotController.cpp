// S2 | Orchestrates a bounded joint command and an abstract bus send.
#include "robot/control/RobotController.hpp"
#include <cmath>
#include <stdexcept>
#include <utility>
namespace robot {
RobotController::RobotController(std::unique_ptr<IBus> bus) : bus_(std::move(bus)) {
    // Enforce the ownership invariant: controller always has a transport.
    if (!bus_) throw std::invalid_argument("bus is null");
}
bool RobotController::setJointPosition(int id, float pos) {
    if (id < 0 || id >= kJointCount || !std::isfinite(pos)) return false;
    const auto index = static_cast<std::size_t>(id);
    // Clamp before I/O so the bus never receives an out-of-range target.
    const float limited = hand_.clamp(index, pos);
    // Update the command cache only after a successful send attempt.
    // Success here is not an encoder acknowledgment.
    if (!bus_->send(id, limited)) return false;
    hand_.setPosition(index, limited);
    return true;
}
float RobotController::commandedPosition(std::size_t id) const { return hand_.getPosition(id); }
} // namespace robot
