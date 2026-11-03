#include "robot/bus/MockBus.hpp"
namespace robot {
bool MockBus::send(int id, float position_deg) {
    ++send_count;
    last_id = id;
    last_position = position_deg;
    if (fail_count > 0) { --fail_count; return false; }
    return send_result;
}
} // namespace robot
