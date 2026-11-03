#pragma once
namespace robot {
class IBus {
public:
    virtual ~IBus() = default;
    virtual bool send(int id, float position_deg) = 0;
};
} // namespace robot
