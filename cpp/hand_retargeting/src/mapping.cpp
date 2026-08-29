#include "dexterous_hand/mapping.hpp"

#include <fstream>
#include <iterator>
#include <regex>
#include <stdexcept>

namespace dexterous_hand {

RetargetConfig load_retarget_config_from_mapping(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open joint mapping: " + path);
  }

  const std::string contents((std::istreambuf_iterator<char>(input)),
                             std::istreambuf_iterator<char>());
  const std::regex limit_pattern(
      R"("limit_rad"\s*:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\])");

  RetargetConfig config = HandRetargeter::safe_default_config();
  std::sregex_iterator current(contents.begin(), contents.end(),
                               limit_pattern);
  const std::sregex_iterator end;
  std::size_t count = 0;
  for (; current != end && count < kJointCount; ++current, ++count) {
    config.lower_limits[count] = std::stod((*current)[1].str());
    config.upper_limits[count] = std::stod((*current)[2].str());
    config.home_pose[count] = 0.0;
  }

  if (count != kJointCount || current != end) {
    throw std::runtime_error("joint mapping must contain exactly 16 limit_rad pairs");
  }
  return config;
}

}  // namespace dexterous_hand
