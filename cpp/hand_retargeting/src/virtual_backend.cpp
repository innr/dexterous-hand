#include "dexterous_hand/backend.hpp"

#include <cmath>
#include <stdexcept>

namespace dexterous_hand {

void VirtualSts3215Backend::send_position(
    const std::array<double, kJointCount>& position_rad) {
  for (double position : position_rad) {
    if (!std::isfinite(position)) {
      throw std::invalid_argument("virtual backend received non-finite position");
    }
  }
  state_.position_rad = position_rad;
  state_.velocity_rad_s.fill(0.0);
}

JointState VirtualSts3215Backend::read_state() const { return state_; }

void VirtualSts3215Backend::disable_torque() {
  state_.torque_enabled = false;
  state_.velocity_rad_s.fill(0.0);
}

}  // namespace dexterous_hand
