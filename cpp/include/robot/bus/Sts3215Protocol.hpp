#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
// 与 hardware/sts3215.py 对齐的 STS3215 数据包编解码。
namespace robot { namespace sts3215 {
constexpr std::uint8_t kRead = 0x02, kWrite = 0x03;
constexpr std::uint8_t kGoalPosition = 42, kPresentPosition = 56;
constexpr std::size_t kMaxPacket = 32;
struct Packet { std::array<std::uint8_t,kMaxPacket> bytes{}; std::size_t size = 0; };
struct Status { std::uint8_t id = 0, error = 0; std::array<std::uint8_t,kMaxPacket> params{}; std::size_t param_size = 0; };
bool encode(std::uint8_t id, std::uint8_t instruction, const std::uint8_t* params,
            std::size_t count, Packet& output) noexcept;
bool decodeStatus(const std::uint8_t* bytes, std::size_t size,
                  std::uint8_t expected_id, Status& output) noexcept;
} }
