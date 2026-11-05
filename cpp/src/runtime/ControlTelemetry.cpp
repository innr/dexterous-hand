#include "robot/runtime/ControlTelemetry.hpp"
#include <iomanip>
#include <ostream>
#include <stdexcept>

namespace robot {
ControlTelemetryRing::ControlTelemetryRing(std::size_t capacity) : slots_(capacity) {
    if (capacity==0 || capacity>1000000)
        throw std::invalid_argument("telemetry capacity must be 1..1000000");
}
bool ControlTelemetryRing::tryPush(const ControlTelemetryFrame& frame) noexcept {
    const auto w=write_.load(std::memory_order_relaxed);
    const auto r=read_.load(std::memory_order_acquire);
    if (w-r>=slots_.size()) {
        dropped_.fetch_add(1,std::memory_order_relaxed);
        return false;
    }
    slots_[static_cast<std::size_t>(w%slots_.size())]=frame;
    write_.store(w+1,std::memory_order_release);
    return true;
}
bool ControlTelemetryRing::tryPop(ControlTelemetryFrame& frame) noexcept {
    const auto r=read_.load(std::memory_order_relaxed);
    const auto w=write_.load(std::memory_order_acquire);
    if (r==w) return false;
    frame=slots_[static_cast<std::size_t>(r%slots_.size())];
    read_.store(r+1,std::memory_order_release);
    return true;
}
void writeTelemetryCsvHeader(std::ostream& out) {
    out<<"tick,time_us,joint,status,safety_state,published,target_deg,reference_deg,"
          "command_deg,measured_deg,measured_velocity_deg_s,feedback_age_us,"
          "feedback_valid,feedback_sequence,start_lateness_us,control_duration_us\n";
}
void writeTelemetryCsvFrame(std::ostream& out,const ControlTelemetryFrame& f) {
    out<<std::setprecision(9);
    for (std::size_t j=0;j<kServoCount;++j) {
        out<<f.tick<<','<<f.time_us<<','<<j<<','
           <<static_cast<int>(f.status)<<','<<static_cast<int>(f.safety_state)<<','
           <<(f.status==ControlTickStatus::PUBLISHED?1:0)<<','
           <<f.target_deg[j]<<','<<f.reference_deg[j]<<','<<f.command_deg[j]<<','
           <<f.measured_deg[j]<<','<<f.measured_velocity_deg_s[j]<<','
           <<f.feedback_age_us[j]<<','<<(f.feedback_valid[j]?1:0)<<','
           <<f.feedback_sequence<<','<<f.start_lateness_us<<','
           <<f.control_duration_us<<'\n';
    }
}
} // namespace robot
