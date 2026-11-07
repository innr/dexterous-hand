#include "robot/contract/CanonicalCommand.hpp"

#include <array>
#include <cstdio>
#include <cstring>
#include <limits>

namespace {
int failures = 0;

void check(bool cond, const char* what) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

void fillName(char dest[32], const char* src) {
    std::memset(dest, 0, 32);
    for (std::size_t i = 0; i < 31 && src[i] != '\0'; ++i) {
        dest[i] = src[i];
    }
}

robot::contract::CanonicalCommand shuffledCommand() {
    using robot::contract::kCanonicalDof;
    using robot::contract::kCanonicalNames;
    robot::contract::CanonicalCommand cmd;
    cmd.session_id = 1;
    for (std::size_t i = 0; i < kCanonicalDof; ++i) {
        const std::size_t src = kCanonicalDof - 1 - i;
        fillName(cmd.names[i], kCanonicalNames[src]);
        cmd.position_rad[i] = static_cast<double>(src);
    }
    return cmd;
}
} // namespace

int main() {
    using robot::contract::kCanonicalDof;
    using robot::contract::kCanonicalNameMax;
    using robot::contract::kCanonicalNames;
    using robot::contract::CanonicalCommand;
    using robot::contract::CanonicalState;
    using robot::contract::reorderToCanonical;

    check(kCanonicalDof == 16, "kCanonicalDof is 16");
    check(kCanonicalNameMax == 31, "kCanonicalNameMax is 31");
    check(std::strlen("middle_mcp_forward") == 18, "longest name is 18 chars");
    check(std::strlen("middle_mcp_forward") <= kCanonicalNameMax, "longest name fits");

    const char* expected[16] = {
        "index_mcp_side", "index_mcp_forward", "index_pip", "index_dip",
        "middle_mcp_side", "middle_mcp_forward", "middle_pip", "middle_dip",
        "ring_mcp_side", "ring_mcp_forward", "ring_pip", "ring_dip",
        "thumb_cmc_side", "thumb_cmc_forward", "thumb_mcp", "thumb_ip",
    };
    for (std::size_t i = 0; i < kCanonicalDof; ++i) {
        check(std::strcmp(kCanonicalNames[i], expected[i]) == 0, "frozen name table");
        check(std::strlen(kCanonicalNames[i]) <= kCanonicalNameMax, "name length");
        check(kCanonicalNames[i][31] == '\0' || std::strlen(kCanonicalNames[i]) < 32,
              "stored name has room for NUL");
    }

    CanonicalCommand def{};
    check(def.session_id == 0, "default session_id is the invalid 0");
    CanonicalState state{};
    check(state.velocity_rad_s.size() == kCanonicalDof, "state has 16 velocities");

    CanonicalCommand cmd = shuffledCommand();
    std::array<double, kCanonicalDof> out{};
    char err[64] = {};
    check(reorderToCanonical(cmd, out, err, sizeof(err)), "reorder shuffled names");
    for (std::size_t i = 0; i < kCanonicalDof; ++i) {
        check(out[i] == static_cast<double>(i), "reordered rad by canonical id");
    }

    CanonicalCommand menagerie = cmd;
    fillName(menagerie.names[0], "if_rot");
    check(!reorderToCanonical(menagerie, out, err, sizeof(err)), "reject Menagerie names");
    check(std::strlen(err) > 0, "error buffer written");

    CanonicalCommand dup = shuffledCommand();
    fillName(dup.names[1], kCanonicalNames[15]);
    check(!reorderToCanonical(dup, out, err, sizeof(err)), "reject duplicate name");

    CanonicalCommand nan_cmd = shuffledCommand();
    nan_cmd.position_rad[3] = std::numeric_limits<double>::quiet_NaN();
    check(!reorderToCanonical(nan_cmd, out, err, sizeof(err)), "reject NaN");
    check(std::strcmp(err, "non_finite") == 0, "NaN error token");

    CanonicalCommand unterminated = shuffledCommand();
    std::memset(unterminated.names[2], 'a', 32);
    check(!reorderToCanonical(unterminated, out, err, sizeof(err)), "reject missing NUL");

    // JointTable must not be required to compile this translation unit.

    if (failures != 0) {
        std::fprintf(stderr, "%d checks failed\n", failures);
        return 1;
    }
    std::printf("S7CanonicalLayoutTest passed\n");
    return 0;
}
