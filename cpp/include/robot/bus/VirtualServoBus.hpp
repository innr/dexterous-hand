#pragma once
#include "robot/bus/IServoTransport.hpp"
#include "robot/runtime/FeedbackBuffer.hpp"
#include <array>

// PC 虚拟总线：仅用于接口集成，不代表真实 STS3215 动力学。
namespace robot {
class VirtualServoBus final : public IServoTransport {
public:
    bool send(std::size_t id, float position_deg) override;
    bool read(std::size_t id, float& position_deg, float& velocity_deg_s) override;
    void setReadFailure(std::size_t id, bool fail);
    std::size_t sendCount() const noexcept { return sends_; }
private:
    std::array<float, kServoCount> position_{};
    std::array<bool, kServoCount> fail_{};
    std::size_t sends_ = 0;
};
} // namespace robot
