#pragma once
#include <cstddef>
#include <optional>
#include <vector>

namespace robot {
// 单关节、单次恒定阶跃试验；全部时间相对阶跃时刻，单位秒。
// 输入可来自离线仿真或未来的只读台架日志，绝不触发运动。
struct ResponseSample {
    double time_s = 0.0;
    double target_deg = 0.0;
    double reference_deg = 0.0;
    double command_deg = 0.0;
    double measured_deg = 0.0;
    double feedback_age_ms = 0.0;
};

struct ResponseMetrics {
    std::size_t samples = 0;
    double duration_s = 0.0;
    double step_deg = 0.0;
    double rms_reference_error_deg = 0.0;
    double rms_target_error_deg = 0.0;
    double iae_target_deg_s = 0.0;
    double overshoot_deg = 0.0;
    double overshoot_percent = 0.0;
    double steady_state_error_deg = 0.0; // 最后20%时间窗的 signed (target-measured) 平均值
    double max_feedback_age_ms = 0.0;
    double p99_feedback_age_ms = 0.0; // nearest-rank
    std::optional<double> rise_time_10_90_s;
    std::optional<double> settling_time_s;
};

struct ResponseMetricConfig {
    double settling_band_fraction = 0.02; // 目标阶跃幅度的2%
    double settling_band_min_deg = 0.2;
    double settling_hold_s = 0.2; // 末尾至少保持该时长
    double minimum_step_deg = 0.5;
};

// 拒绝时间不严格递增、非有限数据、非恒定目标及无阶跃的试验。
// 对不满足上升/稳定条件的试验，optional 保持空值（不是 0 秒）。
ResponseMetrics analyzeStepResponse(const std::vector<ResponseSample>& samples,
                                    ResponseMetricConfig config = {});
} // namespace robot
