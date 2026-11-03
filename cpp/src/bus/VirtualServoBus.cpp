#include "robot/bus/VirtualServoBus.hpp"
#include <cmath>
#include <stdexcept>
namespace robot {
bool VirtualServoBus::send(std::size_t id, float position_deg) {
    if (id >= kServoCount || !std::isfinite(position_deg)) return false;
    position_[id] = position_deg;
    ++sends_;
    return true;
}
bool VirtualServoBus::read(std::size_t id, float& p, float& v) {
    if (id >= kServoCount || fail_[id]) return false;
    p = position_[id];
    v = 0.0f;
    return true;
}
void VirtualServoBus::setReadFailure(std::size_t id, bool fail) {
    if (id >= kServoCount) throw std::out_of_range("invalid servo id");
    fail_[id] = fail;
}
} // namespace robot
