#include "dexterous_hand/retargeter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace dexterous_hand {
namespace {

struct Vec3 {
  double x;
  double y;
  double z;
};

Vec3 operator-(const Landmark& a, const Landmark& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator-(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3& v, double scale) {
  return {v.x * scale, v.y * scale, v.z * scale};
}

double dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y,
          a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

double norm(const Vec3& v) {
  return std::sqrt(dot(v, v));
}

Vec3 normalized(const Vec3& v) {
  const double length = norm(v);
  if (length < 1e-9) {
    throw std::invalid_argument("landmark vectors are degenerate");
  }
  return v * (1.0 / length);
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

double bend_angle(const Vec3& first, const Vec3& second) {
  const Vec3 a = normalized(first);
  const Vec3 b = normalized(second);
  return std::acos(clamp(dot(a, b), -1.0, 1.0));
}

bool finite(const Landmark& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

struct PalmFrame {
  Vec3 lateral;
  Vec3 forward;
  Vec3 normal;
};

PalmFrame make_palm_frame(const std::array<Landmark, kLandmarkCount>& points,
                          Handedness handedness) {
  // MediaPipe indices: wrist=0, index MCP=5, middle MCP=9, pinky MCP=17.
  const Vec3 forward = normalized(points[9] - points[0]);
  // Remove the component along the forward axis before normalizing. This
  // makes the lateral axis stable when the two MCP points are at different
  // distances from the wrist and makes left/right mirroring deterministic.
  const Vec3 raw_lateral = points[17] - points[5];
  Vec3 lateral = normalized(raw_lateral - forward * dot(raw_lateral, forward));
  Vec3 normal = normalized(cross(lateral, forward));
  // The frame is anatomical (index-to-pinky and wrist-to-middle), so a
  // physically mirrored hand already produces the same signed local angles.
  // Keep the handedness argument in the API for future camera adapters that
  // may need an explicit image-space reflection.
  (void)handedness;
  return {lateral, forward, normal};
}

std::array<double, kJointCount> raw_angles(
    const std::array<Landmark, kLandmarkCount>& points,
    const PalmFrame& frame) {
  // Each entry is {MCP, PIP, DIP, TIP}; the thumb uses CMC, MCP, IP, TIP.
  constexpr std::array<std::array<std::size_t, 4>, 4> chains = {{
      {{5, 6, 7, 8}},
      {{9, 10, 11, 12}},
      {{13, 14, 15, 16}},
      {{1, 2, 3, 4}},
  }};

  std::array<double, kJointCount> result{};
  for (std::size_t finger = 0; finger < chains.size(); ++finger) {
    const auto& chain = chains[finger];
    const Vec3 first_bone = points[chain[1]] - points[chain[0]];
    const Vec3 direction = normalized(first_bone);

    // Signed MCP angles in the canonical palm frame.
    const double side = std::atan2(dot(direction, frame.lateral),
                                   dot(direction, frame.forward));
    const double forward = std::atan2(dot(direction, frame.normal),
                                      dot(direction, frame.forward));

    const double first_bend = bend_angle(
        points[chain[1]] - points[chain[0]],
        points[chain[2]] - points[chain[1]]);
    const double second_bend = bend_angle(
        points[chain[2]] - points[chain[1]],
        points[chain[3]] - points[chain[2]]);

    const std::size_t base = finger * 4;
    result[base] = side;
    result[base + 1] = forward;
    result[base + 2] = first_bend;
    result[base + 3] = second_bend;
  }
  return result;
}

}  // namespace

RetargetConfig HandRetargeter::safe_default_config() {
  RetargetConfig config;
  config.lower_limits.fill(-3.14159265358979323846);
  config.upper_limits.fill(3.14159265358979323846);
  config.home_pose.fill(0.0);
  return config;
}

HandRetargeter::HandRetargeter(RetargetConfig config)
    : config_(std::move(config)), last_command_(config_.home_pose) {
  if (!(config_.confidence_threshold >= 0.0 &&
        config_.confidence_threshold <= 1.0)) {
    throw std::invalid_argument("confidence threshold must be in [0, 1]");
  }
  if (!(config_.smoothing_alpha > 0.0 && config_.smoothing_alpha <= 1.0)) {
    throw std::invalid_argument("smoothing alpha must be in (0, 1]");
  }
  if (!(config_.max_velocity_rad_s > 0.0) ||
      !(config_.max_frame_gap_s > 0.0)) {
    throw std::invalid_argument("velocity and frame gap must be positive");
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(config_.lower_limits[i]) ||
        !std::isfinite(config_.upper_limits[i]) ||
        config_.lower_limits[i] > config_.upper_limits[i] ||
        !std::isfinite(config_.home_pose[i])) {
      throw std::invalid_argument("invalid joint limit or home pose");
    }
    last_command_[i] = clamp(config_.home_pose[i], config_.lower_limits[i],
                             config_.upper_limits[i]);
  }
}

RetargetOutput HandRetargeter::hold(RetargetReason reason,
                                    double timestamp) const {
  return {last_command_, false, reason, timestamp};
}

RetargetOutput HandRetargeter::update(const RetargetInput& input) {
  if (!(std::isfinite(input.confidence)) ||
      input.confidence < config_.confidence_threshold) {
    return hold(RetargetReason::LowConfidence, input.timestamp);
  }

  for (const Landmark& point : input.landmarks) {
    if (!finite(point)) {
      return hold(RetargetReason::InvalidLandmarks, input.timestamp);
    }
  }
  if (!std::isfinite(input.timestamp)) {
    return hold(RetargetReason::InvalidTimestamp, input.timestamp);
  }
  if (have_timestamp_ && input.timestamp <= last_timestamp_) {
    return hold(RetargetReason::InvalidTimestamp, input.timestamp);
  }

  std::array<double, kJointCount> target{};
  try {
    const PalmFrame frame = make_palm_frame(input.landmarks, input.handedness);
    target = raw_angles(input.landmarks, frame);
  } catch (const std::invalid_argument&) {
    return hold(RetargetReason::InvalidLandmarks, input.timestamp);
  }

  for (std::size_t i = 0; i < kJointCount; ++i) {
    target[i] = clamp(target[i], config_.lower_limits[i],
                      config_.upper_limits[i]);
  }

  if (!have_valid_sample_) {
    last_command_ = target;
    have_valid_sample_ = true;
    have_timestamp_ = true;
    last_timestamp_ = input.timestamp;
    return {last_command_, true, RetargetReason::Ok, input.timestamp};
  }

  const double dt = input.timestamp - last_timestamp_;
  last_timestamp_ = input.timestamp;
  if (dt > config_.max_frame_gap_s) {
    return hold(RetargetReason::FrameGap, input.timestamp);
  }

  const double alpha = config_.smoothing_alpha;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double smoothed =
        alpha * target[i] + (1.0 - alpha) * last_command_[i];
    const double max_delta = config_.max_velocity_rad_s * dt;
    const double delta = clamp(smoothed - last_command_[i], -max_delta,
                               max_delta);
    last_command_[i] = clamp(last_command_[i] + delta,
                             config_.lower_limits[i],
                             config_.upper_limits[i]);
  }
  return {last_command_, true, RetargetReason::Ok, input.timestamp};
}

std::string to_string(RetargetReason reason) {
  switch (reason) {
    case RetargetReason::Ok:
      return "ok";
    case RetargetReason::LowConfidence:
      return "low_confidence";
    case RetargetReason::InvalidLandmarks:
      return "invalid_landmarks";
    case RetargetReason::InvalidTimestamp:
      return "invalid_timestamp";
    case RetargetReason::FrameGap:
      return "frame_gap";
  }
  return "unknown";
}

}  // namespace dexterous_hand
