#include "dexterous_hand/backend.hpp"
#include "dexterous_hand/mapping.hpp"
#include "dexterous_hand/retargeter.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

using dexterous_hand::Handedness;
using dexterous_hand::HandRetargeter;
using dexterous_hand::Landmark;
using dexterous_hand::RetargetInput;
using dexterous_hand::RetargetReason;

std::array<Landmark, dexterous_hand::kLandmarkCount> make_hand(
    bool bent_index = false) {
  std::array<Landmark, dexterous_hand::kLandmarkCount> points{};
  points[0] = {0.0, 0.0, 0.0};
  points[5] = {-0.18, 0.25, 0.0};
  points[9] = {0.0, 0.28, 0.0};
  points[13] = {0.18, 0.25, 0.0};
  points[17] = {0.32, 0.18, 0.0};

  points[1] = {-0.22, 0.05, 0.0};
  points[2] = {-0.30, 0.15, 0.0};
  points[3] = {-0.36, 0.23, 0.0};
  points[4] = {-0.40, 0.30, 0.0};

  const std::array<std::size_t, 3> mcp = {5, 9, 13};
  const std::array<std::size_t, 3> pip = {6, 10, 14};
  const std::array<std::size_t, 3> dip = {7, 11, 15};
  const std::array<std::size_t, 3> tip = {8, 12, 16};
  const std::array<double, 3> x = {-0.18, 0.0, 0.18};
  for (std::size_t i = 0; i < 3; ++i) {
    points[mcp[i]] = {x[i], 0.25, 0.0};
    points[pip[i]] = {x[i], 0.42, 0.0};
    points[dip[i]] = {x[i], 0.59, 0.0};
    points[tip[i]] = {x[i], 0.76, 0.0};
  }
  points[17] = {0.30, 0.18, 0.0};
  points[18] = {0.30, 0.34, 0.0};
  points[19] = {0.30, 0.50, 0.0};
  points[20] = {0.30, 0.66, 0.0};

  if (bent_index) {
    points[6] = {-0.18, 0.34, 0.0};
    points[7] = {-0.10, 0.42, 0.0};
    points[8] = {-0.02, 0.35, 0.0};
  }
  return points;
}

dexterous_hand::RetargetConfig test_config() {
  auto config = HandRetargeter::safe_default_config();
  config.lower_limits.fill(-2.5);
  config.upper_limits.fill(2.5);
  config.max_velocity_rad_s = 1.0;
  config.max_frame_gap_s = 0.5;
  return config;
}

RetargetInput input_for(
    const std::array<Landmark, dexterous_hand::kLandmarkCount>& points,
    double timestamp) {
  RetargetInput input;
  input.landmarks = points;
  input.confidence = 0.99;
  input.timestamp = timestamp;
  input.handedness = Handedness::Right;
  return input;
}

void test_shape_and_backend() {
  HandRetargeter retargeter(test_config());
  dexterous_hand::VirtualSts3215Backend backend;
  const auto output = retargeter.update(input_for(make_hand(), 0.0));
  assert(output.valid);
  backend.send_position(output.position_rad);
  const auto state = backend.read_state();
  assert(state.torque_enabled);
  assert(state.position_rad == output.position_rad);
  backend.disable_torque();
  assert(!backend.read_state().torque_enabled);
}

void test_bending_and_limits() {
  HandRetargeter retargeter(test_config());
  const auto open = retargeter.update(input_for(make_hand(), 0.0));
  const auto bent = retargeter.update(input_for(make_hand(true), 0.1));
  assert(open.valid && bent.valid);
  assert(bent.position_rad[2] > open.position_rad[2]);
  assert(bent.position_rad[3] > open.position_rad[3]);
  for (double value : bent.position_rad) {
    assert(value >= -2.5 && value <= 2.5);
  }
}

void test_left_right_mirror() {
  HandRetargeter right_retargeter(test_config());
  HandRetargeter left_retargeter(test_config());
  auto right = make_hand(true);
  auto left = right;
  for (Landmark& point : left) {
    point.x = -point.x;
  }
  auto right_input = input_for(right, 0.0);
  auto left_input = input_for(left, 0.0);
  right_input.handedness = Handedness::Right;
  left_input.handedness = Handedness::Left;
  const auto right_output = right_retargeter.update(right_input);
  const auto left_output = left_retargeter.update(left_input);
  for (std::size_t i = 0; i < dexterous_hand::kJointCount; ++i) {
    assert(std::abs(right_output.position_rad[i] -
                    left_output.position_rad[i]) < 1e-9);
  }
}

void test_invalid_input_holds() {
  HandRetargeter retargeter(test_config());
  const auto valid = retargeter.update(input_for(make_hand(), 0.0));
  auto bad = input_for(make_hand(), 0.1);
  bad.confidence = 0.1;
  const auto low = retargeter.update(bad);
  assert(!low.valid && low.reason == RetargetReason::LowConfidence);
  assert(low.position_rad == valid.position_rad);

  bad = input_for(make_hand(), 0.2);
  bad.landmarks[8].x = NAN;
  const auto nan = retargeter.update(bad);
  assert(!nan.valid && nan.reason == RetargetReason::InvalidLandmarks);
  assert(nan.position_rad == valid.position_rad);
}

void test_velocity_and_frame_gap() {
  HandRetargeter retargeter(test_config());
  const auto open = retargeter.update(input_for(make_hand(), 0.0));
  const auto bent = retargeter.update(input_for(make_hand(true), 0.01));
  for (std::size_t i = 0; i < dexterous_hand::kJointCount; ++i) {
    assert(std::abs(bent.position_rad[i] - open.position_rad[i]) <= 0.0100001);
  }
  const auto gap = retargeter.update(input_for(make_hand(true), 1.0));
  assert(!gap.valid && gap.reason == RetargetReason::FrameGap);
}

void test_mapping_loader() {
  const auto path = std::filesystem::temp_directory_path() /
                    "dexterous_hand_test_mapping.json";
  {
    std::ofstream output(path);
    for (int i = 0; i < 16; ++i) {
      output << "{\"limit_rad\":[-" << (i + 1) << "," << (i + 2)
             << "]}\n";
    }
  }
  const auto config = dexterous_hand::load_retarget_config_from_mapping(
      path.string());
  assert(config.lower_limits[0] == -1.0);
  assert(config.upper_limits[15] == 17.0);
  std::filesystem::remove(path);
}

}  // namespace

int main() {
  test_shape_and_backend();
  test_bending_and_limits();
  test_left_right_mirror();
  test_invalid_input_holds();
  test_velocity_and_frame_gap();
  test_mapping_loader();
  std::cout << "all C++ retargeting tests passed\n";
  return 0;
}
