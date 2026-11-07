#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif
#include "robot/mapping/JointTable.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

namespace {
int failures = 0;

void check(bool cond, const char* what) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

std::string writeTemp(const std::string& name, const std::string& body) {
    const char* dir = std::getenv("TEMP");
    if (dir == nullptr) {
        dir = "/tmp";
    }
    std::string path = std::string(dir) + "/" + name;
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << body;
    return path;
}

std::string joint(int id, const char* name, double z = 0.0) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "{\"hardware_id\":%d,\"name\":\"%s\",\"mujoco_joint\":\"%d\","
                  "\"direction\":1,\"zero_offset_rad\":%.3f,\"limit_rad\":[-0.5,0.5]}",
                  id, name, id, z);
    return buf;
}

std::string validJson(double zero = 0.0, const char* status = "not_calibrated",
                      int count = 16) {
    const char* names[16] = {
        "index_mcp_side", "index_mcp_forward", "index_pip", "index_dip",
        "middle_mcp_side", "middle_mcp_forward", "middle_pip", "middle_dip",
        "ring_mcp_side", "ring_mcp_forward", "ring_pip", "ring_dip",
        "thumb_cmc_side", "thumb_cmc_forward", "thumb_mcp", "thumb_ip",
    };
    std::string body = "{\"schema_version\":1,\"hardware\":{\"position_unit\":"
                       "\"radian_at_interface\"},\"calibration\":{\"status\":\"";
    body += status;
    body += "\"},\"joints\":[";
    for (int i = 0; i < count; ++i) {
        if (i) {
            body += ",";
        }
        body += joint(i, names[i], i == 0 ? zero : 0.0);
    }
    body += "]}";
    return body;
}
} // namespace

int main() {
    using robot::mapping::CalibrationStatus;
    using robot::mapping::JointTable;
    using robot::mapping::loadJointTable;
    using robot::mapping::motionConfigFromTable;
    using robot::mapping::resolveIdentityPath;
    using robot::mapping::transportConfigFromTable;

    std::string path;
    std::string error;
    JointTable table;
    check(resolveIdentityPath(path, error), "resolve from env");
    check(loadJointTable(path.c_str(), table, error), error.c_str());
    check(table.schema_version == 1, "schema 1");
    check(table.calibration == CalibrationStatus::NotCalibrated, "repo not calibrated");
    auto motion = motionConfigFromTable(table);
    check(motion.joints[0].min_deg < -50.0f && motion.joints[0].max_deg > 50.0f,
          "side limits from JSON");
    check(motion.joints[0].kp == 1.0f, "PD defaults retained");
    auto transport = transportConfigFromTable(table, true);
    check(!transport.allow_motion, "uncalibrated blocks motion even if operator true");
    check(transport.joints[0].center_ticks == 2048, "center_ticks frozen at 2048");

    check(!loadJointTable("", table, error), "empty path");
    const std::string truncated = writeTemp("leap_trunc.json", "{\"schema_version\":1");
    check(!loadJointTable(truncated.c_str(), table, error), "truncated");
    const std::string comma = writeTemp(
        "leap_comma.json",
        "{\"schema_version\":1,\"hardware\":{\"position_unit\":\"radian_at_interface\"},"
        "\"calibration\":{\"status\":\"not_calibrated\"},\"joints\":[],}");
    check(!loadJointTable(comma.c_str(), table, error), "trailing comma");
    auto bad_schema = validJson();
    bad_schema.replace(bad_schema.find("1"), 1, "2");
    const std::string schema = writeTemp("leap_schema.json", bad_schema);
    check(!loadJointTable(schema.c_str(), table, error), "schema_version != 1");
    const std::string fifteen = writeTemp("leap_15.json", validJson(0.0, "not_calibrated", 15));
    check(!loadJointTable(fifteen.c_str(), table, error), "15 joints");

    std::string dup = validJson();
    const std::string needle = "\"hardware_id\":1";
    const auto pos = dup.find(needle);
    dup.replace(pos, needle.size(), "\"hardware_id\":0");
    const std::string dup_path = writeTemp("leap_dup.json", dup);
    check(!loadJointTable(dup_path.c_str(), table, error), "duplicate hardware_id");

    const std::string offset_path = writeTemp("leap_offset.json", validJson(0.1));
    check(loadJointTable(offset_path.c_str(), table, error), "nonzero offset loads");
    auto offset_transport = transportConfigFromTable(table, true);
    check(!offset_transport.allow_motion, "nonzero offset does not enable motion");
    check(offset_transport.joints[0].center_ticks == 2048,
          "nonzero offset does not rewrite center_ticks");
    check(table.joints[0].zero_offset_rad > 0.09, "offset stored on JointRecord");

    if (failures != 0) {
        std::fprintf(stderr, "%d checks failed\n", failures);
        return 1;
    }
    std::printf("S7JointTableLoadTest passed\n");
    return 0;
}
