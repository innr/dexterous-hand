// STS3215 台架只读探测：不使能扭矩、不写目标位置，必须显式指定 16 路 ID。
#include "robot/bus/PosixHalfDuplexPort.hpp"
#include "robot/bus/Sts3215Transport.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    if (argc!=3) {
        std::cerr << "用法: sts3215_readonly_probe /dev/ttyUSB0 1,2,3,...,16\n"
                     "只读探测；要求自动方向切换并支持 UART TEMT 查询。\n";
        return 2;
    }
    try {
        robot::Sts3215Transport::Config config;
        config.allow_motion=false; // 即使调用 startSend 也不会驱动舵机。
        std::istringstream ids(argv[2]);
        std::string token;
        for (std::size_t i=0;i<robot::kServoCount;++i) {
            if (!std::getline(ids,token,',')) {
                std::cerr << "必须显式提供 16 个物理 ID\n";
                return 2;
            }
            const auto parsed=std::stoi(token);
            if (parsed<0 || parsed>253) {
                std::cerr << "ID 超出范围\n";
                return 2;
            }
            config.joints[i].servo_id=static_cast<std::uint8_t>(parsed);
        }
        if (std::getline(ids,token,',')) {
            std::cerr << "ID 数量不能超过 16\n";
            return 2;
        }
        robot::PosixHalfDuplexPort port(argv[1]);
        robot::Sts3215Transport transport(port,config);
        for (int round=0;round<2;++round) {
            for (std::size_t i=0;i<robot::kServoCount;++i) {
                float position=0.0f,velocity=0.0f;
                const bool valid=transport.read(i,position,velocity);
                if (round==1) {
                    std::cout << "关节 " << i << " / 舵机 "
                              << static_cast<unsigned>(config.joints[i].servo_id) << ": ";
                    if (valid) std::cout << position << " deg, " << velocity << " deg/s\n";
                    else std::cout << "无效（超时、协议错误、未标定或速度未就绪）\n";
                }
                if (transport.faulted()) {
                    std::cerr << "TX 完成确认失败，驱动已锁存故障\n";
                    return 1;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "只读探测失败: " << e.what() << "\n";
        return 1;
    }
}
