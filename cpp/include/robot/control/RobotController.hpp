#pragma once
// S2 | Owns IBus via unique_ptr: the controller is responsible for its lifetime.
// Synchronous send is for learning/testing; S4 must move bus I/O off the control thread.

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
