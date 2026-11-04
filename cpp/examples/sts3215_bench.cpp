// S5-04：真实 16 路 STS3215 只读台架测试，导出 CSV，不写目标或使能扭矩。
#include "robot/bus/PosixHalfDuplexPort.hpp"
#include "robot/bus/Sts3215Transport.hpp"
#include "robot/runtime/ReadOnlyBench.hpp"

#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
std::size_t positive(const std::string& value) {
    std::size_t consumed=0;
    const auto number=std::stoull(value,&consumed);
    if (consumed!=value.size() || number==0 || number>10000)
        throw std::invalid_argument("expected integer in [1,10000]");
    return static_cast<std::size_t>(number);
}
void parseIds(const std::string& value, robot::Sts3215Transport::Config& config) {
    std::istringstream input(value);
    std::string token;
    for (std::size_t i=0;i<robot::kServoCount;++i) {
        if (!std::getline(input,token,','))
            throw std::invalid_argument("exactly 16 servo IDs required");
        std::size_t consumed=0;
        const int id=std::stoi(token,&consumed);
        if (consumed!=token.size() || id<0 || id>253)
            throw std::invalid_argument("servo ID must be in [0,253]");
        config.joints[i].servo_id=static_cast<std::uint8_t>(id);
    }
    if (std::getline(input,token,','))
        throw std::invalid_argument("too many servo IDs");
}
void usage() {
    std::cerr << "用法: sts3215_bench --port /dev/ttyUSB0 --ids 1,2,...,16 "
                 "[--rounds 100] [--period-ms 10] [--age-ms 20] "
                 "[--csv s5_04_bench.csv]\n"
                 "仅读取原始位置 tick；不会使能扭矩或下发运动命令。\n";
}
}
int main(int argc,char** argv) {
    std::string port_name,ids,csv="s5_04_bench.csv";
    robot::BenchConfig bench;
    try {
        for (int i=1;i<argc;++i) {
            const std::string key=argv[i];
            if (i+1>=argc) { usage(); return 2; }
            const std::string value=argv[++i];
            if (key=="--port") port_name=value;
            else if (key=="--ids") ids=value;
            else if (key=="--csv") csv=value;
            else if (key=="--rounds") bench.rounds=positive(value);
            else if (key=="--period-ms")
                bench.control_period=std::chrono::milliseconds(positive(value));
            else if (key=="--age-ms")
                bench.max_feedback_age=std::chrono::milliseconds(positive(value));
            else { usage(); return 2; }
        }
        if (port_name.empty() || ids.empty()) { usage(); return 2; }
        robot::Sts3215Transport::Config config;
        config.allow_motion=false; // 只读门禁，任何情况下均不启动运动写入。
        parseIds(ids,config);
        robot::PosixHalfDuplexPort port(port_name);
        robot::Sts3215Transport transport(port,config);
        const auto report=robot::ReadOnlyBench::run(
            [&](std::size_t joint,std::uint16_t& ticks) {
                return transport.readPositionTicks(joint,ticks);
            },bench);
        std::ofstream output(csv);
        if (!output) throw std::runtime_error("cannot create CSV");
        robot::ReadOnlyBench::writeCsv(output,report);
        if (!output) throw std::runtime_error("CSV write failed");
        std::cout << "CSV: " << csv << "\n"
                  << "轮询: " << report.round_us.size() << " 轮\n"
                  << "读取成功/失败: " << report.successful_reads << "/"
                  << report.failed_reads << "\n"
                  << "整轮耗时 us (min/mean/p95/max): "
                  << report.round_latency.min_us << "/"
                  << report.round_latency.mean_us << "/"
                  << report.round_latency.p95_us << "/"
                  << report.round_latency.max_us << "\n"
                  << "控制观察 tick: " << report.ticks.size()
                  << ", Deadline miss: " << report.deadline_misses
                  << ", 跳过周期: " << report.skipped_control_cycles << "\n"
                  << "最大启动迟到 us: " << report.max_start_lateness_us
                  << ", 最大反馈年龄 us: " << report.max_feedback_age_us
                  << ", 过期关节观测次数: " << report.stale_observations << "\n";
        for (std::size_t i=0;i<robot::kServoCount;++i)
            std::cout << "关节 " << i << " / ID "
                      << static_cast<unsigned>(config.joints[i].servo_id)
                      << " 成功 " << report.successful_by_joint[i]
                      << " 失败 " << report.failed_by_joint[i] << "\n";
        if (transport.faulted()) {
            std::cerr << "串口 TX 完成确认失败，驱动锁存故障\n";
            return 1;
        }
        return report.failed_reads==0 ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "台架测试失败: " << error.what() << "\n";
        return 1;
    }
}
