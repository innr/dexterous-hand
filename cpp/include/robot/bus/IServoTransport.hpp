#pragma once
#include <cstddef>

// 同步总线接口；异步 DMA 必须另行定义完成事件及许可生命周期。
namespace robot {
class IServoTransport {
public:
    virtual ~IServoTransport() = default;
    virtual bool send(std::size_t id, float position_deg) = 0;
    virtual bool read(std::size_t id, float& position_deg, float& velocity_deg_s) = 0;
};
} // namespace robot
