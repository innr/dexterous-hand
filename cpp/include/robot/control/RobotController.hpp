#pragma once
#include "robot/bus/IBus.hpp"
#include "robot/core/RobotHand.hpp"
#include <cstddef>
#include <memory>
namespace robot {
// Educational synchronous controller; S4 must decouple I/O from 100 Hz control.
class RobotController {
public:
    explicit RobotController(std::unique_ptr<IBus> bus);
    bool setJointPosition(int id, float pos);
    float commandedPosition(std::size_t id) const;
private:
    RobotHand hand_;
    std::unique_ptr<IBus> bus_;
};
} // namespace robot
