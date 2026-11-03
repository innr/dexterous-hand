// S2 | Mock transport for failure injection and send-count assertions.
#include "robot/bus/MockBus.hpp"
namespace robot {
bool MockBus::send(int id, float position_deg) {
    // Record each attempted send, including failed attempts.
    ++send_count;
    last_id = id;
    last_position = position_deg;
    // Fail exactly the next fail_count sends, then use send_result.
    if (fail_count > 0) { --fail_count; return false; }
    return send_result;
}
} // namespace robot
