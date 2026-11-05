// S5-08：仅离线确定性虚拟模型。输出排名及最佳候选的 CSV，禁止自动写入舵机。
#include "robot/control/VirtualTuningBench.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

int main(int argc,char** argv) {
    if (argc>2) {
        std::cerr<<"usage: s5_virtual_tuning [best_trace.csv]\n";
        return 2;
    }
    try {
        const std::vector<robot::TuningCandidate> candidates{
            {0.3f,0.00f},{0.3f,0.05f},{0.3f,0.15f},
            {0.7f,0.00f},{0.7f,0.05f},{0.7f,0.15f},
            {1.0f,0.00f},{1.0f,0.05f},{1.0f,0.15f},
            {1.5f,0.00f},{1.5f,0.05f},{1.5f,0.15f}
        };
        const auto ranked=robot::rankVirtualCandidates(candidates);
        std::cout<<std::fixed<<std::setprecision(4);
        std::cout<<"OFFLINE VIRTUAL PLANT ONLY -- NOT HARDWARE TUNING\n";
        std::cout<<"rank,kp,kd_seconds,score,rms_reference_deg,overshoot_deg,settling_s\n";
        for (std::size_t i=0;i<ranked.size();++i) {
            const auto& x=ranked[i];
            std::cout<<i+1<<','<<x.candidate.kp<<','<<x.candidate.kd_seconds<<','
                     <<x.score<<','<<x.metrics.rms_reference_error_deg<<','
                     <<x.metrics.overshoot_deg<<',';
            if (x.metrics.settling_time_s) std::cout<<*x.metrics.settling_time_s;
            else std::cout<<"not_reached";
            std::cout<<'\n';
        }
        if (argc==2) {
            std::ofstream out(argv[1]);
            if (!out) throw std::runtime_error("cannot open output CSV");
            out<<std::fixed<<std::setprecision(9);
            out<<"time_s,target_deg,reference_deg,command_deg,measured_deg,feedback_age_ms\n";
            for (const auto& x:ranked.front().trace)
                out<<x.time_s<<','<<x.target_deg<<','<<x.reference_deg<<','
                   <<x.command_deg<<','<<x.measured_deg<<','
                   <<x.feedback_age_ms<<'\n';
            if (!out) throw std::runtime_error("cannot write output CSV");
        }
    } catch (const std::exception& e) {
        std::cerr<<"virtual tuning failed: "<<e.what()<<"\n";
        return 1;
    }
}
