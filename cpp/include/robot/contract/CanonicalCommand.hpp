#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

// S7-00 application-facing 16-DOF layout (radians). Header-only.
// Do not include mapping / JointTable. Do not call from ControlLoop::tick.
namespace robot {
namespace contract {

constexpr std::size_t kCanonicalDof = 16;
constexpr std::size_t kCanonicalNameMax = 31; // + NUL in char[32]

constexpr char kCanonicalNames[kCanonicalDof][32] = {
    "index_mcp_side",
    "index_mcp_forward",
    "index_pip",
    "index_dip",
    "middle_mcp_side",
    "middle_mcp_forward",
    "middle_pip",
    "middle_dip",
    "ring_mcp_side",
    "ring_mcp_forward",
    "ring_pip",
    "ring_dip",
    "thumb_cmc_side",
    "thumb_cmc_forward",
    "thumb_mcp",
    "thumb_ip",
};

struct CanonicalCommand {
    std::uint64_t stamp_us = 0;
    std::uint64_t session_id = 0; // 0 is invalid
    std::uint64_t epoch = 0;
    std::uint64_t sequence = 0;
    char names[kCanonicalDof][32]{}; // must be NUL-terminated; max 31 chars
    std::array<double, kCanonicalDof> position_rad{};
};

struct CanonicalState {
    std::uint64_t stamp_us = 0;
    std::uint64_t session_id = 0;
    std::uint64_t epoch = 0;
    std::uint64_t sequence = 0;
    char names[kCanonicalDof][32]{};
    std::array<double, kCanonicalDof> position_rad{};
    std::array<double, kCanonicalDof> velocity_rad_s{};
};

inline void writeContractError(char* error, std::size_t error_cap, const char* message) {
    if (error == nullptr || error_cap == 0) {
        return;
    }
    std::size_t n = 0;
    while (message[n] != '\0' && n + 1 < error_cap) {
        error[n] = message[n];
        ++n;
    }
    error[n] = '\0';
}

inline bool nameTerminated(const char name[32]) {
    for (std::size_t i = 0; i < 32; ++i) {
        if (name[i] == '\0') {
            return true;
        }
    }
    return false;
}

inline bool namesEqual(const char a[32], const char b[32]) {
    return nameTerminated(a) && nameTerminated(b) && std::strncmp(a, b, 32) == 0;
}

// Reorder in.position_rad into canonical id 0..15 using expected_names.
// Session-boundary only. Returns false and writes error on failure. No throw.
inline bool reorderToCanonical(const CanonicalCommand& in,
                               const char expected_names[kCanonicalDof][32],
                               std::array<double, kCanonicalDof>& out_rad,
                               char* error, std::size_t error_cap) {
    if (expected_names == nullptr) {
        writeContractError(error, error_cap, "expected_names is null");
        return false;
    }
    for (std::size_t i = 0; i < kCanonicalDof; ++i) {
        if (!nameTerminated(expected_names[i]) || expected_names[i][0] == '\0') {
            writeContractError(error, error_cap, "expected name missing NUL");
            return false;
        }
        if (!nameTerminated(in.names[i]) || in.names[i][0] == '\0') {
            writeContractError(error, error_cap, "command name missing NUL");
            return false;
        }
        if (!std::isfinite(in.position_rad[i])) {
            writeContractError(error, error_cap, "non_finite");
            return false;
        }
    }
    std::array<bool, kCanonicalDof> used{};
    for (std::size_t want = 0; want < kCanonicalDof; ++want) {
        int found = -1;
        for (std::size_t have = 0; have < kCanonicalDof; ++have) {
            if (!namesEqual(in.names[have], expected_names[want])) {
                continue;
            }
            if (found >= 0 || used[static_cast<std::size_t>(have)]) {
                writeContractError(error, error_cap, "duplicate name");
                return false;
            }
            found = static_cast<int>(have);
        }
        if (found < 0) {
            writeContractError(error, error_cap, "missing canonical name");
            return false;
        }
        const std::size_t src = static_cast<std::size_t>(found);
        used[src] = true;
        out_rad[want] = in.position_rad[src];
    }
    return true;
}

inline bool reorderToCanonical(const CanonicalCommand& in,
                               std::array<double, kCanonicalDof>& out_rad,
                               char* error, std::size_t error_cap) {
    return reorderToCanonical(in, kCanonicalNames, out_rad, error, error_cap);
}

} // namespace contract
} // namespace robot
