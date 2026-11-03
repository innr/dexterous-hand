#pragma once
// S2 | Hardware-independent command interface (dependency inversion).
// send() reports transport success only; it does not prove physical motion.

namespace robot {
class IBus {
public:
    virtual ~IBus() = default;
    virtual bool send(int id, float position_deg) = 0;
};
} // namespace robot
