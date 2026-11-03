#pragma once

#include "robot/bus/IBus.hpp"

// 测试用总线：记录每次下发，并可通过 fail_count 模拟连续失败。
namespace robot {

class MockBus final : public IBus {
public:
    bool send_result = true;    // send() 的默认返回值
    int fail_count = 0;         // 还需模拟失败的次数
    int send_count = 0;         // 已收到的 send() 调用次数
    int last_id = -1;           // 最近一次下发的关节 ID
    float last_position = 0.0f; // 最近一次下发的角度

    bool send(int id, float position_deg) override;
};

} // namespace robot
