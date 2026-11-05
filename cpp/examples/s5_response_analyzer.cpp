// S5-08：单关节 CSV 离线响应分析。此工具不连接任何舵机，也不发送运动指令。
#include "robot/control/ResponseMetrics.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void stripCR(std::string& line) {
    if (!line.empty() && line.back()=='\r') line.pop_back();
}
double parse(const std::string& value) {
    std::size_t consumed=0;
    const auto n=std::stod(value,&consumed);
    if (consumed!=value.size()) throw std::invalid_argument("invalid CSV number");
    return n;
}
robot::ResponseSample parseRow(const std::string& row) {
    std::string cleaned=row;
    stripCR(cleaned);
    std::istringstream input(cleaned);
    std::string cell;
    std::vector<double> columns;
    while (std::getline(input,cell,',')) columns.push_back(parse(cell));
    if (columns.size()!=6 || (!cleaned.empty() && cleaned.back()==','))
        throw std::invalid_argument("expected 6 numeric CSV columns");
    return {columns[0],columns[1],columns[2],columns[3],columns[4],columns[5]};
}
void optional(std::ostream& out,const char* name,const std::optional<double>& value) {
    out<<name<<"=";
    if (value) out<<*value;
    else out<<"not_reached";
    out<<'\n';
}
}
int main(int argc,char** argv) {
    if (argc!=2) {
        std::cerr<<"usage: s5_response_analyzer <one_joint_step.csv>\n";
        return 2;
    }
    try {
        std::ifstream file(argv[1]);
        if (!file) throw std::runtime_error("cannot open input CSV");
        std::string line;
        if (!std::getline(file,line))
            throw std::invalid_argument("missing CSV header");
        stripCR(line);
        if (line!="time_s,target_deg,reference_deg,command_deg,measured_deg,feedback_age_ms")
            throw std::invalid_argument("unexpected CSV header");
        std::vector<robot::ResponseSample> samples;
        while (std::getline(file,line)) {
            if (line.empty()) continue;
            samples.push_back(parseRow(line));
        }
        const auto m=robot::analyzeStepResponse(samples);
        std::cout<<std::fixed<<std::setprecision(5);
        std::cout<<"samples="<<m.samples<<"\n"
                 <<"duration_s="<<m.duration_s<<"\n"
                 <<"rms_reference_error_deg="<<m.rms_reference_error_deg<<"\n"
                 <<"rms_target_error_deg="<<m.rms_target_error_deg<<"\n"
                 <<"iae_target_deg_s="<<m.iae_target_deg_s<<"\n"
                 <<"overshoot_deg="<<m.overshoot_deg<<"\n"
                 <<"overshoot_percent="<<m.overshoot_percent<<"\n"
                 <<"steady_state_error_deg="<<m.steady_state_error_deg<<"\n"
                 <<"p99_feedback_age_ms="<<m.p99_feedback_age_ms<<"\n"
                 <<"max_feedback_age_ms="<<m.max_feedback_age_ms<<"\n";
        optional(std::cout,"rise_time_10_90_s",m.rise_time_10_90_s);
        optional(std::cout,"settling_time_s",m.settling_time_s);
    } catch (const std::exception& e) {
        std::cerr<<"analysis failed: "<<e.what()<<"\n";
        return 1;
    }
}
