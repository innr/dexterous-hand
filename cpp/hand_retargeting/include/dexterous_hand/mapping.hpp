#pragma once

#include "dexterous_hand/retargeter.hpp"

#include <string>

namespace dexterous_hand {

// Reads the 16 limit_rad pairs from config/joint_mapping.json without adding
// a third-party JSON dependency to the C++ core.
RetargetConfig load_retarget_config_from_mapping(const std::string& path);

}  // namespace dexterous_hand
