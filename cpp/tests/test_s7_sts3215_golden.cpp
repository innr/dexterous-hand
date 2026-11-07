#include "robot/bus/Sts3215Protocol.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
std::string hexOf(const robot::sts3215::Packet& packet) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    out.resize(packet.size * 2);
    for (std::size_t i = 0; i < packet.size; ++i) {
        out[2 * i] = digits[packet.bytes[i] >> 4];
        out[2 * i + 1] = digits[packet.bytes[i] & 0x0f];
    }
    return out;
}

std::string extract(const std::string& json, const std::string& key) {
    const std::string needle = "\"" + key + "\"";
    const auto pos = json.find(needle);
    if (pos == std::string::npos) {
        return {};
    }
    const auto colon = json.find(':', pos + needle.size());
    const auto quote = json.find('"', colon);
    const auto end = json.find('"', quote + 1);
    if (colon == std::string::npos || quote == std::string::npos || end == std::string::npos) {
        return {};
    }
    return json.substr(quote + 1, end - quote - 1);
}

std::vector<std::uint8_t> fromHex(const std::string& hex) {
    std::vector<std::uint8_t> out;
    for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
        auto nybble = [](char c) {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        const int hi = nybble(hex[i]);
        const int lo = nybble(hex[i + 1]);
        if (hi < 0 || lo < 0) {
            return {};
        }
        out.push_back(static_cast<std::uint8_t>((hi << 4) | lo));
    }
    return out;
}
} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: test_s7_sts3215_golden packets.json\n";
        return 1;
    }
    std::ifstream in(argv[1], std::ios::binary);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    const std::string json = buffer.str();
    robot::sts3215::Packet packet;
    assert(robot::sts3215::encode(1, 1, nullptr, 0, packet));
    assert(hexOf(packet) == extract(json, "ping_id1"));
    const std::uint8_t read_params[] = {56, 2};
    assert(robot::sts3215::encode(7, robot::sts3215::kRead, read_params, 2, packet));
    assert(hexOf(packet) == extract(json, "read_id7_present_position"));
    const auto status = fromHex(extract(json, "status_id1_params_3412"));
    robot::sts3215::Status decoded;
    assert(robot::sts3215::decodeStatus(status.data(), status.size(), 1, decoded));
    assert(decoded.params[0] == 0x34 && decoded.params[1] == 0x12);
    auto bad = fromHex(extract(json, "status_bad_checksum"));
    assert(!robot::sts3215::decodeStatus(bad.data(), bad.size(), 1, decoded));
    std::printf("S7Sts3215GoldenTest passed\n");
    return 0;
}
