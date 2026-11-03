#include "robot/bus/MockBus.hpp"

namespace robot {

bool MockBus::send(int id, float position_deg) {
    // 记录调用信息，便于测试断言。
    ++send_count;
    last_id = id;
    last_position = position_deg;

    // fail_count 大于 0 时模拟一次失败并递减。
    if (fail_count > 0) {
        --fail_count;
        return false;
    }
    return send_result;
}

} // namespace robot
