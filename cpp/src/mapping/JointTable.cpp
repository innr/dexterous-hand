#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif
#include "robot/mapping/JointTable.hpp"

#include "robot/contract/CanonicalCommand.hpp"
#include "robot/runtime/FeedbackBuffer.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

namespace robot {
namespace mapping {
namespace {

static_assert(contract::kCanonicalDof == kServoCount, "canonical DOF must match servo count");

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

struct Json {
    enum Type { NUL, BOOL, NUM, STR, ARR, OBJ } type = NUL;
    bool b = false;
    double n = 0.0;
    std::string s;
    std::vector<Json> a;
    std::map<std::string, Json> o;
};

class Parser {
public:
    explicit Parser(std::string text) : text_(std::move(text)) {}

    bool parse(Json& out, std::string& error) {
        skip();
        if (!parseValue(out, error)) {
            return false;
        }
        skip();
        if (i_ != text_.size()) {
            error = "trailing content after JSON value";
            return false;
        }
        return true;
    }

private:
    std::string text_;
    std::size_t i_ = 0;

    void skip() {
        while (i_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[i_]))) {
            ++i_;
        }
    }

    bool parseValue(Json& out, std::string& error) {
        skip();
        if (i_ >= text_.size()) {
            error = "truncated JSON";
            return false;
        }
        const char c = text_[i_];
        if (c == '{') {
            return parseObject(out, error);
        }
        if (c == '[') {
            return parseArray(out, error);
        }
        if (c == '"') {
            return parseString(out, error);
        }
        if (c == 't' || c == 'f') {
            return parseBool(out, error);
        }
        if (c == 'n') {
            return parseNull(out, error);
        }
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
            return parseNumber(out, error);
        }
        error = "unexpected token";
        return false;
    }

    bool parseObject(Json& out, std::string& error) {
        ++i_;
        out.type = Json::OBJ;
        skip();
        if (i_ < text_.size() && text_[i_] == '}') {
            ++i_;
            return true;
        }
        while (true) {
            skip();
            Json key;
            if (!parseString(key, error)) {
                return false;
            }
            skip();
            if (i_ >= text_.size() || text_[i_] != ':') {
                error = "expected ':'";
                return false;
            }
            ++i_;
            Json value;
            if (!parseValue(value, error)) {
                return false;
            }
            out.o.emplace(key.s, std::move(value));
            skip();
            if (i_ >= text_.size()) {
                error = "truncated object";
                return false;
            }
            if (text_[i_] == ',') {
                ++i_;
                skip();
                if (i_ < text_.size() && text_[i_] == '}') {
                    error = "trailing comma";
                    return false;
                }
                continue;
            }
            if (text_[i_] == '}') {
                ++i_;
                return true;
            }
            error = "expected ',' or '}'";
            return false;
        }
    }

    bool parseArray(Json& out, std::string& error) {
        ++i_;
        out.type = Json::ARR;
        skip();
        if (i_ < text_.size() && text_[i_] == ']') {
            ++i_;
            return true;
        }
        while (true) {
            Json value;
            if (!parseValue(value, error)) {
                return false;
            }
            out.a.push_back(std::move(value));
            skip();
            if (i_ >= text_.size()) {
                error = "truncated array";
                return false;
            }
            if (text_[i_] == ',') {
                ++i_;
                skip();
                if (i_ < text_.size() && text_[i_] == ']') {
                    error = "trailing comma";
                    return false;
                }
                continue;
            }
            if (text_[i_] == ']') {
                ++i_;
                return true;
            }
            error = "expected ',' or ']'";
            return false;
        }
    }

    bool parseString(Json& out, std::string& error) {
        if (i_ >= text_.size() || text_[i_] != '"') {
            error = "expected string";
            return false;
        }
        ++i_;
        out.type = Json::STR;
        while (i_ < text_.size()) {
            const char c = text_[i_++];
            if (c == '"') {
                return true;
            }
            if (c == '\\') {
                if (i_ >= text_.size()) {
                    error = "truncated escape";
                    return false;
                }
                const char e = text_[i_++];
                switch (e) {
                case '"':
                case '\\':
                case '/':
                    out.s.push_back(e);
                    break;
                case 'n':
                    out.s.push_back('\n');
                    break;
                case 't':
                    out.s.push_back('\t');
                    break;
                default:
                    error = "unsupported escape";
                    return false;
                }
            } else {
                out.s.push_back(c);
            }
        }
        error = "truncated string";
        return false;
    }

    bool parseNumber(Json& out, std::string& error) {
        const std::size_t start = i_;
        if (text_[i_] == '-') {
            ++i_;
        }
        if (i_ >= text_.size() || !std::isdigit(static_cast<unsigned char>(text_[i_]))) {
            error = "invalid number";
            return false;
        }
        if (text_[i_] == '0') {
            ++i_;
        } else {
            while (i_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[i_]))) {
                ++i_;
            }
        }
        if (i_ < text_.size() && text_[i_] == '.') {
            ++i_;
            if (i_ >= text_.size() || !std::isdigit(static_cast<unsigned char>(text_[i_]))) {
                error = "invalid number";
                return false;
            }
            while (i_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[i_]))) {
                ++i_;
            }
        }
        out.type = Json::NUM;
        try {
            out.n = std::stod(text_.substr(start, i_ - start));
        } catch (...) {
            error = "invalid number";
            return false;
        }
        return true;
    }

    bool parseBool(Json& out, std::string& error) {
        if (text_.compare(i_, 4, "true") == 0) {
            i_ += 4;
            out.type = Json::BOOL;
            out.b = true;
            return true;
        }
        if (text_.compare(i_, 5, "false") == 0) {
            i_ += 5;
            out.type = Json::BOOL;
            out.b = false;
            return true;
        }
        error = "invalid boolean";
        return false;
    }

    bool parseNull(Json& out, std::string& error) {
        if (text_.compare(i_, 4, "null") == 0) {
            i_ += 4;
            out.type = Json::NUL;
            return true;
        }
        error = "invalid null";
        return false;
    }
};

bool copyName(const std::string& src, char dest[32], std::string& error) {
    if (src.empty() || src.size() > contract::kCanonicalNameMax) {
        error = "name empty or longer than 31 characters";
        return false;
    }
    std::memset(dest, 0, 32);
    std::memcpy(dest, src.data(), src.size());
    return true;
}

const Json* field(const Json& obj, const char* key) {
    if (obj.type != Json::OBJ) {
        return nullptr;
    }
    const auto it = obj.o.find(key);
    if (it == obj.o.end()) {
        return nullptr;
    }
    return &it->second;
}

bool asInt(const Json& v, int& out) {
    if (v.type != Json::NUM) {
        return false;
    }
    if (!std::isfinite(v.n) || v.n != std::floor(v.n)) {
        return false;
    }
    out = static_cast<int>(v.n);
    return true;
}

} // namespace

bool resolveIdentityPath(std::string& path, std::string& error) {
    const char* mapping = std::getenv("LEAP_JOINT_MAPPING");
    if (mapping != nullptr) {
        if (mapping[0] == '\0') {
            error = "LEAP_JOINT_MAPPING is empty";
            return false;
        }
        path = mapping;
        return true;
    }
    const char* root = std::getenv("LEAP_PROJECT_ROOT");
    if (root != nullptr && root[0] != '\0') {
        path = std::string(root) + "/config/joint_mapping.json";
        return true;
    }
    error = "LEAP_JOINT_MAPPING or LEAP_PROJECT_ROOT is required";
    return false;
}

bool loadJointTable(const char* path, JointTable& out, std::string& error) {
    out = JointTable{};
    if (path == nullptr || path[0] == '\0') {
        error = "mapping path is empty";
        return false;
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = std::string("cannot read mapping file: ") + path;
        return false;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    Parser parser(buffer.str());
    Json root;
    if (!parser.parse(root, error)) {
        return false;
    }
    const Json* schema = field(root, "schema_version");
    int version = 0;
    if (schema == nullptr || !asInt(*schema, version) || version != 1) {
        error = "schema_version must be 1";
        return false;
    }
    const Json* hardware = field(root, "hardware");
    const Json* unit = hardware != nullptr ? field(*hardware, "position_unit") : nullptr;
    if (unit == nullptr || unit->type != Json::STR || unit->s != "radian_at_interface") {
        error = "hardware.position_unit must be radian_at_interface";
        return false;
    }
    const Json* calibration = field(root, "calibration");
    const Json* status = calibration != nullptr ? field(*calibration, "status") : nullptr;
    if (status == nullptr || status->type != Json::STR) {
        error = "calibration.status missing";
        return false;
    }
    if (status->s == "not_calibrated") {
        out.calibration = CalibrationStatus::NotCalibrated;
    } else if (status->s == "calibrated") {
        out.calibration = CalibrationStatus::Calibrated;
    } else {
        error = "invalid calibration.status";
        return false;
    }
    const Json* joints = field(root, "joints");
    if (joints == nullptr || joints->type != Json::ARR ||
        joints->a.size() != contract::kCanonicalDof) {
        error = "joints must contain exactly 16 entries";
        return false;
    }
    std::array<bool, contract::kCanonicalDof> seen{};
    for (const Json& item : joints->a) {
        const Json* idv = field(item, "hardware_id");
        const Json* namev = field(item, "name");
        const Json* urdfv = field(item, "mujoco_joint");
        const Json* dirv = field(item, "direction");
        const Json* zerov = field(item, "zero_offset_rad");
        const Json* limv = field(item, "limit_rad");
        int hardware_id = -1;
        int direction = 0;
        if (idv == nullptr || !asInt(*idv, hardware_id) ||
            hardware_id < 0 ||
            hardware_id >= static_cast<int>(contract::kCanonicalDof)) {
            error = "invalid hardware_id";
            return false;
        }
        if (seen[static_cast<std::size_t>(hardware_id)]) {
            error = "duplicate hardware_id";
            return false;
        }
        seen[static_cast<std::size_t>(hardware_id)] = true;
        if (namev == nullptr || namev->type != Json::STR ||
            urdfv == nullptr || urdfv->type != Json::STR ||
            dirv == nullptr || !asInt(*dirv, direction) ||
            (direction != 1 && direction != -1) ||
            zerov == nullptr || zerov->type != Json::NUM || !std::isfinite(zerov->n) ||
            limv == nullptr || limv->type != Json::ARR || limv->a.size() != 2 ||
            limv->a[0].type != Json::NUM || limv->a[1].type != Json::NUM ||
            !std::isfinite(limv->a[0].n) || !std::isfinite(limv->a[1].n) ||
            !(limv->a[0].n < 0.0 && 0.0 < limv->a[1].n)) {
            error = "invalid joint record";
            return false;
        }
        if (namev->s != contract::kCanonicalNames[static_cast<std::size_t>(hardware_id)]) {
            error = "canonical name does not match frozen table";
            return false;
        }
        JointRecord rec;
        rec.canonical_id = hardware_id;
        rec.hardware_id = hardware_id;
        rec.direction = direction;
        rec.zero_offset_rad = zerov->n;
        rec.limit_min_rad = limv->a[0].n;
        rec.limit_max_rad = limv->a[1].n;
        if (!copyName(namev->s, rec.canonical, error) ||
            !copyName(urdfv->s, rec.urdf_joint, error)) {
            return false;
        }
        out.joints[static_cast<std::size_t>(hardware_id)] = rec;
    }
    for (bool flag : seen) {
        if (!flag) {
            error = "missing hardware_id in 0..15";
            return false;
        }
    }
    out.schema_version = version;
    return true;
}

MotionControlConfig motionConfigFromTable(const JointTable& table) {
    MotionControlConfig config;
    for (std::size_t i = 0; i < contract::kCanonicalDof; ++i) {
        config.joints[i].min_deg =
            static_cast<float>(table.joints[i].limit_min_rad * kRadToDeg);
        config.joints[i].max_deg =
            static_cast<float>(table.joints[i].limit_max_rad * kRadToDeg);
    }
    return config;
}

Sts3215Transport::Config transportConfigFromTable(const JointTable& table,
                                                  bool operator_allow_motion) {
    Sts3215Transport::Config config;
    config.allow_motion =
        operator_allow_motion && table.calibration == CalibrationStatus::Calibrated;
    for (std::size_t i = 0; i < contract::kCanonicalDof; ++i) {
        auto& joint = config.joints[i];
        joint.servo_id = static_cast<std::uint8_t>(table.joints[i].hardware_id);
        joint.center_ticks = 2048;
        joint.ticks_per_degree = 4096.0f / 360.0f;
        joint.direction = table.joints[i].direction;
        joint.min_degree = static_cast<float>(table.joints[i].limit_min_rad * kRadToDeg);
        joint.max_degree = static_cast<float>(table.joints[i].limit_max_rad * kRadToDeg);
        (void)table.joints[i].zero_offset_rad;
    }
    return config;
}

} // namespace mapping
} // namespace robot
