#pragma once

#include "robot/bus/Sts3215Transport.hpp"
#include "robot/contract/CanonicalCommand.hpp"
#include "robot/control/MotionControlCore.hpp"

#include <array>
#include <string>

// Identity table from config/joint_mapping.json. Load at process start only.
namespace robot {
namespace mapping {

enum class CalibrationStatus { NotCalibrated, Calibrated, Invalid };

struct JointRecord {
    int canonical_id = -1;
    char canonical[32]{};
    char urdf_joint[32]{};
    int hardware_id = -1;
    int direction = 1;
    double zero_offset_rad = 0.0;
    double limit_min_rad = 0.0;
    double limit_max_rad = 0.0;
};

struct JointTable {
    int schema_version = 0;
    CalibrationStatus calibration = CalibrationStatus::Invalid;
    std::array<JointRecord, contract::kCanonicalDof> joints{};
};

bool resolveIdentityPath(std::string& path, std::string& error);
bool loadJointTable(const char* path, JointTable& out, std::string& error);
MotionControlConfig motionConfigFromTable(const JointTable& table);
Sts3215Transport::Config transportConfigFromTable(const JointTable& table,
                                                  bool operator_allow_motion);

} // namespace mapping
} // namespace robot
