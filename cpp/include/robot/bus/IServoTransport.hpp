#pragma once
#include <cstddef>

// 通信接口：TX 启动与完成分离，以便安全门禁只覆盖短暂启动动作。
namespace robot {
class IServoTransport {
public:
    virtual ~IServoTransport() = default;

    // 必须非阻塞、有界、无异常，并在返回 true 前真正启动 TX（不能仅排队）。
    // 调用方保证同一时刻仅有一个未完成的发送事务。
    virtual bool startSend(std::size_t id, float position_deg) noexcept = 0;

    // 等待 UART 物理传输完成（包括移位寄存器 TC），或确认已安全取消。
    // 允许阻塞；不得在门禁互斥锁内调用。
    virtual bool finishSend() noexcept = 0;

    virtual bool read(std::size_t id, float& position_deg, float& velocity_deg_s) = 0;
};
} // namespace robot
