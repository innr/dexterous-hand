#pragma once
// S2 | Deterministic fake bus for unit tests; no serial port is opened.
// fail_count simulates consecutive send failures before normal behavior resumes.

#include "robot/bus/IBus.hpp"
namespace robot {
class MockBus final : public IBus {
public:
    bool send_result = true;
    int fail_count = 0;
    int send_count = 0;
    int last_id = -1;
    float last_position = 0.0f;
    bool send(int id, float position_deg) override;
};
} // namespace robot
