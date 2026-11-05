#include "robot/control/ResponseMetrics.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot {
namespace {
double crossing(const std::vector<ResponseSample>& s,double level,double direction) {
    if (direction*s.front().measured_deg>=direction*level) return s.front().time_s;
    for (std::size_t i=1;i<s.size();++i) {
        const double a=direction*s[i-1].measured_deg;
        const double b=direction*s[i].measured_deg;
        if (a<direction*level && b>=direction*level) {
            const double fraction=(direction*level-a)/(b-a);
            return s[i-1].time_s+fraction*(s[i].time_s-s[i-1].time_s);
        }
    }
    return -1.0;
}
}
ResponseMetrics analyzeStepResponse(const std::vector<ResponseSample>& s,
                                    ResponseMetricConfig cfg) {
    if (s.size()<3 || !std::isfinite(cfg.settling_band_fraction) ||
        cfg.settling_band_fraction<=0 || !std::isfinite(cfg.settling_band_min_deg) ||
        cfg.settling_band_min_deg<=0 || !std::isfinite(cfg.settling_hold_s) ||
        cfg.settling_hold_s<0 || !std::isfinite(cfg.minimum_step_deg) ||
        cfg.minimum_step_deg<=0) throw std::invalid_argument("invalid response metric configuration");
    const double target=s.front().target_deg;
    const double initial=s.front().measured_deg;
    const double delta=target-initial;
    if (!std::isfinite(delta) || std::abs(delta)<cfg.minimum_step_deg)
        throw std::invalid_argument("step amplitude too small");
    for (std::size_t i=0;i<s.size();++i) {
        const auto& x=s[i];
        if (!std::isfinite(x.time_s) || !std::isfinite(x.target_deg) ||
            !std::isfinite(x.reference_deg) || !std::isfinite(x.command_deg) ||
            !std::isfinite(x.measured_deg) || !std::isfinite(x.feedback_age_ms) ||
            x.feedback_age_ms<0 || x.time_s<0 ||
            std::abs(x.target_deg-target)>1e-5 ||
            (i>0 && x.time_s<=s[i-1].time_s))
            throw std::invalid_argument("nonfinite, unsorted, or non-step response samples");
    }
    const double duration=s.back().time_s-s.front().time_s;
    if (!(duration>0)) throw std::invalid_argument("zero response duration");
    ResponseMetrics m;
    m.samples=s.size();
    m.duration_s=duration;
    m.step_deg=delta;
    const double direction=std::copysign(1.0,delta);
    const double band=std::max(cfg.settling_band_min_deg,
                               cfg.settling_band_fraction*std::abs(delta));
    std::vector<double> ages;
    ages.reserve(s.size());
    std::size_t last_outside=0;
    bool outside=false;
    double tail_sum=0;
    std::size_t tail_count=0;
    for (std::size_t i=0;i<s.size();++i) {
        const auto& x=s[i];
        ages.push_back(x.feedback_age_ms);
        m.max_feedback_age_ms=std::max(m.max_feedback_age_ms,x.feedback_age_ms);
        m.overshoot_deg=std::max(m.overshoot_deg,direction*(x.measured_deg-target));
        if (std::abs(x.measured_deg-target)>band) {
            last_outside=i;
            outside=true;
        }
        if (x.time_s>=s.front().time_s+0.8*duration) {
            tail_sum+=target-x.measured_deg;
            ++tail_count;
        }
        if (i==0) continue;
        const auto& prev=s[i-1];
        const double dt=x.time_s-prev.time_s;
        const double e0=target-prev.measured_deg,e1=target-x.measured_deg;
        const double r0=prev.reference_deg-prev.measured_deg;
        const double r1=x.reference_deg-x.measured_deg;
        m.rms_target_error_deg+=0.5*dt*(e0*e0+e1*e1);
        m.rms_reference_error_deg+=0.5*dt*(r0*r0+r1*r1);
        m.iae_target_deg_s+=0.5*dt*(std::abs(e0)+std::abs(e1));
    }
    m.rms_target_error_deg=std::sqrt(m.rms_target_error_deg/duration);
    m.rms_reference_error_deg=std::sqrt(m.rms_reference_error_deg/duration);
    m.overshoot_percent=100.0*m.overshoot_deg/std::abs(delta);
    m.steady_state_error_deg=tail_sum/static_cast<double>(tail_count);
    std::sort(ages.begin(),ages.end());
    const auto rank=static_cast<std::size_t>(std::ceil(0.99*ages.size()));
    m.p99_feedback_age_ms=ages[rank-1];
    const double t10=crossing(s,initial+0.1*delta,direction);
    const double t90=crossing(s,initial+0.9*delta,direction);
    if (t10>=0 && t90>=t10) m.rise_time_10_90_s=t90-t10;
    if (!outside && duration>=cfg.settling_hold_s)
        m.settling_time_s=0.0;
    else if (outside && last_outside+1<s.size() &&
             s.back().time_s-s[last_outside+1].time_s>=cfg.settling_hold_s)
        m.settling_time_s=s[last_outside+1].time_s-s.front().time_s;
    return m;
}
} // namespace robot
