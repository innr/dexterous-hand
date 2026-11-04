#pragma once
#include "robot/bus/IHalfDuplexPort.hpp"
#include "robot/bus/Sts3215Protocol.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
// Linux 1Mbps 8N1；需要外部自动方向切换且支持 TEMT 查询的串口设备。
namespace robot {
class PosixHalfDuplexPort final : public IHalfDuplexPort {
public:
    explicit PosixHalfDuplexPort(const std::string& device, unsigned baud=1000000);
    ~PosixHalfDuplexPort() override;
    PosixHalfDuplexPort(const PosixHalfDuplexPort&) = delete;
    PosixHalfDuplexPort& operator=(const PosixHalfDuplexPort&) = delete;
    bool startWrite(const std::uint8_t* data, std::size_t size) noexcept override;
    bool finishWrite(std::chrono::milliseconds timeout) noexcept override;
    bool readExact(std::uint8_t* data, std::size_t size,
                   std::chrono::milliseconds timeout) noexcept override;
    bool discardInput() noexcept override;
private:
    int fd_=-1;
    bool active_=false, faulted_=false;
    std::array<std::uint8_t,sts3215::kMaxPacket> pending_{};
    std::size_t size_=0, offset_=0;
};
}
