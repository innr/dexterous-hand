#pragma once
#include "robot/control/ResponseMetrics.hpp"
#include <cstddef>
#include <vector>

namespace robot {
// 仅供离线比较的单关节一阶位置执行器模型，不等于 STS3215 的物理模型。
struct VirtualPlantConfig {
    double period_s = 0.01;
    double duration_s = 5.0;
    double step_deg = 20.0;
    double servo_time_constant_s = 0.12;
    std::size_t command_delay_ticks = 2;
    std::size_t feedback_delay_ticks = 2;
    double max_plant_velocity_deg_s = 180.0;
};
struct TuningCandidate {
    float kp = 1.0f;
    float kd_seconds = 0.05f;
};
struct TuningResult {
    TuningCandidate candidate;
    ResponseMetrics metrics;
    double score = 0.0;
    std::vector<ResponseSample> trace;
};
TuningResult simulateTuningCandidate(TuningCandidate candidate,
                                     VirtualPlantConfig plant = {});
// 候选排序仅适用于相同虚拟模型；不输出或下发真实硬件增益。
std::vector<TuningResult> rankVirtualCandidates(const std::vector<TuningCandidate>& candidates,
                                                VirtualPlantConfig plant = {});
} // namespace robot
