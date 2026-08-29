#include "dexterous_hand/backend.hpp"
#include "dexterous_hand/mapping.hpp"
#include "dexterous_hand/retargeter.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

using dexterous_hand::Handedness;
using dexterous_hand::Landmark;
using dexterous_hand::RetargetInput;

std::array<Landmark, dexterous_hand::kLandmarkCount> make_open_hand() {
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
  return points;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    dexterous_hand::RetargetConfig config;
    if (argc > 1) {
      config = dexterous_hand::load_retarget_config_from_mapping(argv[1]);
    } else {
      config = dexterous_hand::HandRetargeter::safe_default_config();
    }
    dexterous_hand::HandRetargeter retargeter(config);
    dexterous_hand::VirtualSts3215Backend backend;

    RetargetInput input;
    input.landmarks = make_open_hand();
    input.confidence = 0.99;
    input.timestamp = 0.0;
    input.handedness = Handedness::Right;
    const auto output = retargeter.update(input);
    backend.send_position(output.position_rad);

    std::cout << "valid=" << std::boolalpha << output.valid
              << " reason=" << dexterous_hand::to_string(output.reason)
              << "\npositions_rad=";
    for (double position : backend.read_state().position_rad) {
      std::cout << ' ' << std::fixed << std::setprecision(3) << position;
    }
    std::cout << "\n";
    backend.disable_torque();
    std::cout << "torque_enabled=" << backend.read_state().torque_enabled
              << "\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "retarget_demo: " << error.what() << '\n';
    return 1;
  }
}
