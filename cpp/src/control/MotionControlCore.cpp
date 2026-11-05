#include "robot/control/MotionControlCore.hpp"
#include "robot/control/TrajectoryMath.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot {
MotionControlCore::MotionControlCore(MotionControlConfig config) : config_(config) {
    if (config_.nominal_period<=std::chrono::milliseconds::zero() ||
        config_.max_step<config_.nominal_period)
        throw std::invalid_argument("invalid motion period");
    for (std::size_t i=0;i<kServoCount;++i) {
        const auto& c=config_.joints[i];
        if (!std::isfinite(c.min_deg) || !std::isfinite(c.max_deg) ||
            c.min_deg>=c.max_deg || !std::isfinite(c.max_velocity_deg_s) ||
            c.max_velocity_deg_s<=0 || !std::isfinite(c.max_acceleration_deg_s2) ||
            c.max_acceleration_deg_s2<=0 || !std::isfinite(c.kp) || c.kp<0 ||
            !std::isfinite(c.kd_seconds) || c.kd_seconds<0)
            throw std::invalid_argument("invalid joint motion config");
        controllers_[i]=std::make_unique<PDController>(c.kp,c.kd_seconds);
    }
}

bool MotionControlCore::step(const std::array<float,kServoCount>& targets,
                             const std::array<float,kServoCount>& measured,
                             const std::array<float,kServoCount>& measured_velocity,
                             ServoClock::time_point now, MotionStep& result) {
    const auto dt_duration = last_tick_ ? now-*last_tick_ : config_.nominal_period;
    if (dt_duration<=ServoClock::duration::zero() ||
        dt_duration>config_.max_step) return false;
    const float dt=std::chrono::duration<float>(dt_duration).count();
    // 先检查所有关节，再修改任意一个规划器状态，避免部分更新。
    for (std::size_t i=0;i<kServoCount;++i) {
        const auto& c=config_.joints[i];
        if (!std::isfinite(targets[i]) || !std::isfinite(measured[i]) ||
            !std::isfinite(measured_velocity[i]) ||
            measured[i]<c.min_deg || measured[i]>c.max_deg) return false;
    }
    MotionStep candidate;
    for (std::size_t i=0;i<kServoCount;++i) {
        const auto& c=config_.joints[i];
        if (!planners_[i]) {
            // 首次从实测位置起步，绝不从默认 0° 突然追赶。
            planners_[i]=std::make_unique<JointTrajectoryPlanner>(
                measured[i],c.max_velocity_deg_s,c.max_acceleration_deg_s2);
            output_limiters_[i]=std::make_unique<AccelerationLimiter>(
                c.max_acceleration_deg_s2);
            last_output_[i]=measured[i];
        }
        const auto target=std::clamp(targets[i],c.min_deg,c.max_deg);
        const auto ref=planners_[i]->update(target,dt);
        if (!std::isfinite(ref.position_deg) || !std::isfinite(ref.velocity_deg_s))
            return false;
        // PDController::update(target, actual, actual_velocity) 的 D 项是 -kd*v。
        // 传入 (actual_velocity-reference_velocity) 得到 kd*(v_ref-v_actual)。
        const float correction=controllers_[i]->update(
            ref.position_deg,measured[i],measured_velocity[i]-ref.velocity_deg_s);
        const float desired=std::clamp(measured[i]+correction,c.min_deg,c.max_deg);
        if (!std::isfinite(desired)) return false;
        // 最终位置目标再经过独立的限速/限加速度器，防止 PD 修正绕过轨迹限制。
        const float desired_velocity=calculateDesiredVelocityV2(
            last_output_[i],desired,c.max_velocity_deg_s,c.max_acceleration_deg_s2);
        const float old_velocity=output_limiters_[i]->velocity();
        const float next_velocity=output_limiters_[i]->update(desired_velocity,dt);
        const float next_position=std::clamp(
            last_output_[i]+0.5f*(old_velocity+next_velocity)*dt,
            c.min_deg,c.max_deg);
        if (!std::isfinite(next_position) || !std::isfinite(next_velocity))
            return false;
        candidate.reference_deg[i]=ref.position_deg;
        candidate.reference_velocity_deg_s[i]=ref.velocity_deg_s;
        candidate.command_deg[i]=next_position;
        candidate.command_velocity_deg_s[i]=next_velocity;
        last_output_[i]=next_position;
        // 触及机械软限位时强制停止输出速度；软限位优先于加速度约束。
        if ((next_position==c.min_deg && next_velocity<0) ||
            (next_position==c.max_deg && next_velocity>0))
            output_limiters_[i]=std::make_unique<AccelerationLimiter>(
                c.max_acceleration_deg_s2);
    }
    last_tick_=now;
    result=candidate;
    return true;
}
} // namespace robot
