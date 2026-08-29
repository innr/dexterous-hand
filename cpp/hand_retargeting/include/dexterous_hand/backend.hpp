#pragma once

#include "dexterous_hand/retargeter.hpp"

#include <array>

namespace dexterous_hand {

struct JointState {
  std::array<double, kJointCount> position_rad{};
  std::array<double, kJointCount> velocity_rad_s{};
  bool torque_enabled{true};
};

class JointBackend {
 public:
  virtual ~JointBackend() = default;

  virtual void send_position(
      const std::array<double, kJointCount>& position_rad) = 0;
  virtual JointState read_state() const = 0;
  virtual void disable_torque() = 0;
};

// Deterministic backend used until the physical STS3215 chain is available.
// It intentionally exposes the same safety boundary as a future serial backend.
class VirtualSts3215Backend final : public JointBackend {
 public:
  void send_position(
      const std::array<double, kJointCount>& position_rad) override;
  JointState read_state() const override;
  void disable_torque() override;

 private:
  JointState state_{};
};

}  // namespace dexterous_hand
