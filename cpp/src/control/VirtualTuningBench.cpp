#include "robot/control/VirtualTuningBench.hpp"
#include "robot/control/MotionControlCore.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <stdexcept>

namespace robot {
TuningResult simulateTuningCandidate(TuningCandidate candidate,VirtualPlantConfig plant) {
    if (!std::isfinite(plant.period_s) || !std::isfinite(plant.duration_s) ||
        !std::isfinite(plant.step_deg) || !std::isfinite(plant.servo_time_constant_s) ||
        !std::isfinite(plant.max_plant_velocity_deg_s) ||
        plant.period_s<=0 || plant.duration_s<plant.period_s*30 ||
        std::abs(plant.step_deg)<0.5 || std::abs(plant.step_deg)>70 ||
        plant.servo_time_constant_s<=0 || plant.max_plant_velocity_deg_s<=0 ||
        plant.command_delay_ticks>1000 || plant.feedback_delay_ticks>1000 ||
        plant.period_s*plant.command_delay_ticks>plant.duration_s ||
        plant.period_s*plant.feedback_delay_ticks>plant.duration_s)
        throw std::invalid_argument("invalid virtual plant configuration");
    const auto ns=std::chrono::duration_cast<ServoClock::duration>(
        std::chrono::duration<double>(plant.period_s));
    if (ns<=ServoClock::duration::zero()) throw std::invalid_argument("period too small");
    MotionControlConfig control_cfg;
    control_cfg.nominal_period=std::chrono::milliseconds(10);
    control_cfg.max_step=std::chrono::milliseconds(100);
    for (auto& j:control_cfg.joints) {
        j.kp=candidate.kp;
        j.kd_seconds=candidate.kd_seconds;
        j.max_velocity_deg_s=60.0f;
        j.max_acceleration_deg_s2=180.0f;
    }
    MotionControlCore control(control_cfg); // 增益合法性由原控制器校验。
    std::array<float,kServoCount> target{},measured{},velocity{};
    target.fill(static_cast<float>(plant.step_deg));
    std::deque<double> commands(plant.command_delay_ticks,0.0);
    std::deque<double> history(std::max<std::size_t>(1,plant.feedback_delay_ticks),0.0);
    double actual=0.0,prev_measured=0.0;
    TuningResult result;
    result.candidate=candidate;
    const auto count=static_cast<std::size_t>(std::floor(plant.duration_s/plant.period_s));
    result.trace.reserve(count+1);
    // t=0 表示施加目标前的反馈，便于计算上升时间。
    result.trace.push_back({0.0,plant.step_deg,0.0,0.0,0.0,
                            std::max<std::size_t>(1,plant.feedback_delay_ticks)*plant.period_s*1000.0});
    const ServoClock::time_point epoch{};
    for (std::size_t n=1;n<=count;++n) {
        const double measured_position=history.front();
        history.pop_front();
        const double measured_speed=(measured_position-prev_measured)/plant.period_s;
        prev_measured=measured_position;
        measured.fill(static_cast<float>(measured_position));
        velocity.fill(static_cast<float>(measured_speed));
        MotionStep step;
        if (!control.step(target,measured,velocity,epoch+ns*static_cast<ServoClock::duration::rep>(n),step))
            throw std::runtime_error("virtual motion controller rejected sample");
        commands.push_back(step.command_deg[0]);
        const double applied=commands.front();
        commands.pop_front();
        const double speed=std::clamp((applied-actual)/plant.servo_time_constant_s,
                                      -plant.max_plant_velocity_deg_s,
                                      plant.max_plant_velocity_deg_s);
        actual+=speed*plant.period_s;
        history.push_back(actual);
        result.trace.push_back({n*plant.period_s,plant.step_deg,
                                step.reference_deg[0],step.command_deg[0],
                                actual,plant.feedback_delay_ticks*plant.period_s*1000.0});
    }
    result.metrics=analyzeStepResponse(result.trace);
    // 惩罚响应慢、超调和末端偏差；不把这个分数视为硬件稳定性证明。
    result.score=result.metrics.iae_target_deg_s+
                 5.0*result.metrics.overshoot_deg+
                 3.0*std::abs(result.metrics.steady_state_error_deg);
    if (!result.metrics.settling_time_s)
        result.score+=10.0;
    return result;
}

std::vector<TuningResult> rankVirtualCandidates(const std::vector<TuningCandidate>& candidates,
                                                VirtualPlantConfig plant) {
    if (candidates.empty() || candidates.size()>100)
        throw std::invalid_argument("candidate count must be 1..100");
    std::vector<TuningResult> results;
    results.reserve(candidates.size());
    for (const auto c:candidates) results.push_back(simulateTuningCandidate(c,plant));
    std::stable_sort(results.begin(),results.end(),[](const auto& a,const auto& b) {
        return a.score<b.score;
    });
    return results;
}
} // namespace robot
