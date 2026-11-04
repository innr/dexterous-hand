#pragma once
#include <chrono>
#include <cstddef>
#include <cstdint>
// 串口后端：单通信线程调用；负责 TX 完成确认及硬件方向切换。
namespace robot {
class IHalfDuplexPort {
public:
    virtual ~IHalfDuplexPort() = default;
    // 非阻塞、有界：true 表示至少一个字节已提交到物理 TX，剩余部分由 finishWrite 完成。
    virtual bool startWrite(const std::uint8_t* data, std::size_t size) noexcept = 0;
    // 有界等待所有字节完成（UART TC），不允许用 DMA 完成代替。
    virtual bool finishWrite(std::chrono::milliseconds timeout) noexcept = 0;
    virtual bool readExact(std::uint8_t* data, std::size_t size,
                           std::chrono::milliseconds timeout) noexcept = 0;
    virtual bool discardInput() noexcept = 0;
};
}
