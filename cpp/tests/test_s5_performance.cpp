// S5-08：阶跃响应指标边界、无稳定结果、非法数据及离线整定可重复性。
#include "robot/control/ResponseMetrics.hpp"
#include "robot/control/VirtualTuningBench.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
bool near(double a,double b,double eps=1e-6) { return std::abs(a-b)<eps; }
}
int main() {
    using namespace robot;
    std::vector<ResponseSample> samples{
        {0.0,20,0,0,0,0},
        {0.1,20,10,10,10,1},
        {0.2,20,20,20,20,2},
        {0.3,20,20,20,20,3},
        {0.4,20,20,20,20,4},
        {0.5,20,20,20,20,5}
    };
    auto m=analyzeStepResponse(samples);
    assert(m.samples==6 && near(m.duration_s,0.5));
    assert(near(m.overshoot_deg,0) && near(m.overshoot_percent,0));
    assert(m.rise_time_10_90_s && near(*m.rise_time_10_90_s,0.16));
    assert(m.settling_time_s && near(*m.settling_time_s,0.2));
    assert(near(m.steady_state_error_deg,0));
    assert(near(m.iae_target_deg_s,2.0));
    assert(near(m.p99_feedback_age_ms,5.0));
    assert(m.rms_reference_error_deg==0);

    samples[2].measured_deg=23;
    m=analyzeStepResponse(samples);
    assert(near(m.overshoot_deg,3));
    assert(near(m.overshoot_percent,15));
    assert(m.settling_time_s && near(*m.settling_time_s,0.3));
    samples[5].measured_deg=18;
    m=analyzeStepResponse(samples);
    assert(!m.settling_time_s);
    samples[5].measured_deg=20;

    bool rejected=false;
    samples[3].time_s=samples[2].time_s;
    try { (void)analyzeStepResponse(samples); }
    catch (const std::invalid_argument&) { rejected=true; }
    assert(rejected);
    samples[3].time_s=0.3;
    rejected=false;
    samples[3].target_deg=25;
    try { (void)analyzeStepResponse(samples); }
    catch (const std::invalid_argument&) { rejected=true; }
    assert(rejected);
    samples[3].target_deg=20;
    rejected=false;
    samples[3].measured_deg=std::numeric_limits<double>::quiet_NaN();
    try { (void)analyzeStepResponse(samples); }
    catch (const std::invalid_argument&) { rejected=true; }
    assert(rejected);

    const auto baseline=simulateTuningCandidate({1.0f,0.05f});
    const auto repeated=simulateTuningCandidate({1.0f,0.05f});
    assert(baseline.trace.size()>300);
    assert(near(baseline.score,repeated.score,1e-12));
    assert(near(baseline.metrics.rms_reference_error_deg,
                repeated.metrics.rms_reference_error_deg,1e-12));
    assert(std::isfinite(baseline.score));
    assert(baseline.metrics.max_feedback_age_ms>0);
    VirtualPlantConfig delayed;
    delayed.command_delay_ticks=5;
    delayed.feedback_delay_ticks=5;
    const auto high_delay=simulateTuningCandidate({1.0f,0.05f},delayed);
    assert(high_delay.metrics.max_feedback_age_ms>
           baseline.metrics.max_feedback_age_ms);
    assert(!near(high_delay.metrics.rms_target_error_deg,
                 baseline.metrics.rms_target_error_deg));
    VirtualPlantConfig no_command_delay;
    no_command_delay.command_delay_ticks=0;
    const auto no_delay=simulateTuningCandidate({1.0f,0.05f},no_command_delay);
    assert(no_delay.trace.size()==baseline.trace.size());

    const auto ranked=rankVirtualCandidates({
        {0.3f,0.0f},{1.0f,0.05f},{1.5f,0.15f}});
    assert(ranked.size()==3);
    assert(ranked[0].score<=ranked[1].score && ranked[1].score<=ranked[2].score);
    rejected=false;
    try { (void)rankVirtualCandidates({}); }
    catch (const std::invalid_argument&) { rejected=true; }
    assert(rejected);
    rejected=false;
    try { (void)simulateTuningCandidate({1.0f,0.05f},{0.0}); }
    catch (const std::invalid_argument&) { rejected=true; }
    assert(rejected);
}
