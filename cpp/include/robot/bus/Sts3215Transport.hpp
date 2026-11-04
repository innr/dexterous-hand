#pragma once
#include "robot/bus/IHalfDuplexPort.hpp"
#include "robot/bus/IServoTransport.hpp"
#include "robot/bus/Sts3215Protocol.hpp"
#include "robot/runtime/FeedbackBuffer.hpp"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

// STS3215 16 路总线适配：默认只读；不修改扭矩使能和 EEPROM。
namespace robot {
class Sts3215Transport final : public IServoTransport {
public:
    struct JointConfig {
        std::uint8_t servo_id = 0;
        int center_ticks = 2048;
        float ticks_per_degree = 4096.0f / 360.0f;
        int direction = 1;
        float min_degree = -90.0f, max_degree = 90.0f;
    };
    struct Config {
        std::array<JointConfig,kServoCount> joints{};
        std::chrono::milliseconds io_timeout{20};
        bool allow_motion = false; // 必须显式开启运动写入。
        bool expect_write_status = false; // 取决于舵机状态返回等级。
    };
    Sts3215Transport(IHalfDuplexPort& port, Config config);
    bool startSend(std::size_t joint, float degree) noexcept override;
    bool finishSend() noexcept override;
    bool read(std::size_t joint, float& position, float& velocity) override;
    // 只读台架使用：返回未经零位/限位换算的编码器原始 tick。
    // 首次读取即可判定协议是否成功，不需要虚构速度。
    bool readPositionTicks(std::size_t joint, std::uint16_t& ticks);
    bool faulted() const noexcept { return faulted_; }
private:
    bool readStatus(std::uint8_t servo_id, std::size_t count, sts3215::Status& out) noexcept;
    bool sendReadRequest(std::uint8_t servo_id) noexcept;
    IHalfDuplexPort& port_;
    Config config_;
    bool sending_=false, faulted_=false;
    std::uint8_t active_id_=0;
    std::array<bool,kServoCount> have_last_{};
    std::array<float,kServoCount> last_position_{};
    std::array<ServoClock::time_point,kServoCount> last_time_{};
};
}
