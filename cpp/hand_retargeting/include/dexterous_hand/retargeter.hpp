#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace dexterous_hand {

constexpr std::size_t kLandmarkCount = 21;
constexpr std::size_t kJointCount = 16;

struct Landmark {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

enum class Handedness {
  Unknown,
  Left,
  Right,
};

enum class RetargetReason {
  Ok,
  LowConfidence,
  InvalidLandmarks,
  InvalidTimestamp,
  FrameGap,
};

struct RetargetInput {
  std::array<Landmark, kLandmarkCount> landmarks{};
  double confidence{0.0};
  double timestamp{0.0};
  Handedness handedness{Handedness::Right};
};

struct RetargetOutput {
  std::array<double, kJointCount> position_rad{};
  bool valid{false};
  RetargetReason reason{RetargetReason::InvalidLandmarks};
  double timestamp{0.0};
};

// Language-neutral name used by the ROS 2 and future hardware adapters.
using JointCommand = RetargetOutput;

struct RetargetConfig {
  std::array<double, kJointCount> lower_limits{};
  std::array<double, kJointCount> upper_limits{};
  std::array<double, kJointCount> home_pose{};
  double confidence_threshold{0.6};
  double smoothing_alpha{0.35};
  double max_velocity_rad_s{3.0};
  double max_frame_gap_s{0.5};
};

class HandRetargeter {
 public:
  explicit HandRetargeter(RetargetConfig config);

  RetargetOutput update(const RetargetInput& input);

  const std::array<double, kJointCount>& last_command() const noexcept {
    return last_command_;
  }

  static RetargetConfig safe_default_config();

 private:
  RetargetOutput hold(RetargetReason reason, double timestamp) const;

  RetargetConfig config_;
  std::array<double, kJointCount> last_command_{};
  bool have_valid_sample_{false};
  bool have_timestamp_{false};
  double last_timestamp_{0.0};
};

std::string to_string(RetargetReason reason);

}  // namespace dexterous_hand
