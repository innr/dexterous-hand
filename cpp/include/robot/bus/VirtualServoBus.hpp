#pragma once
#include "robot/bus/IServoTransport.hpp"
#include "robot/runtime/FeedbackBuffer.hpp"
#include <array>

// PC 虚拟总线：模拟立即启动/完成；不模拟 UART、DMA 或真实舵机。
namespace robot {
class VirtualServoBus final : public IServoTransport {
public:
    bool startSend(std::size_t id, float position_deg) noexcept override;
    bool finishSend() noexcept override;
    bool read(std::size_t id, float& position_deg, float& velocity_deg_s) override;
    void setReadFailure(std::size_t id, bool fail);
    std::size_t sendCount() const noexcept { return sends_; }
private:
    std::array<float, kServoCount> position_{};
    std::array<bool, kServoCount> fail_{};
    std::size_t sends_ = 0;
    bool active_ = false;
};
} // namespace robot
