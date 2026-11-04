#include "robot/bus/Sts3215Transport.hpp"
#include <cmath>
#include <stdexcept>
#include <utility>

namespace robot {
Sts3215Transport::Sts3215Transport(IHalfDuplexPort& port, Config config)
    : port_(port), config_(std::move(config)) {
    if (config_.io_timeout<=std::chrono::milliseconds::zero())
        throw std::invalid_argument("invalid IO timeout");
    std::array<bool,254> used{};
    for (const auto& j:config_.joints) {
        if (j.servo_id>=254 || used[j.servo_id] || j.center_ticks<0 || j.center_ticks>4095 ||
            !std::isfinite(j.ticks_per_degree) || j.ticks_per_degree<=0 ||
            (j.direction!=1 && j.direction!=-1) ||
            !std::isfinite(j.min_degree) || !std::isfinite(j.max_degree) ||
            j.min_degree>=j.max_degree)
            throw std::invalid_argument("invalid servo mapping");
        used[j.servo_id]=true;
        const float a=j.center_ticks+j.direction*j.min_degree*j.ticks_per_degree;
        const float b=j.center_ticks+j.direction*j.max_degree*j.ticks_per_degree;
        if (a<0 || a>4095 || b<0 || b>4095)
            throw std::invalid_argument("joint limits outside encoder range");
    }
}
bool Sts3215Transport::startSend(std::size_t joint,float degree) noexcept {
    if (faulted_ || sending_ || !config_.allow_motion || joint>=kServoCount ||
        !std::isfinite(degree)) return false;
    const auto& j=config_.joints[joint];
    if (degree<j.min_degree || degree>j.max_degree) return false;
    const float raw=j.center_ticks+j.direction*degree*j.ticks_per_degree;
    if (!std::isfinite(raw) || raw<0 || raw>4095) return false;
    const auto ticks=static_cast<std::uint16_t>(std::lround(raw));
    const std::uint8_t params[]={
        sts3215::kGoalPosition,
        static_cast<std::uint8_t>(ticks&0xff),
        static_cast<std::uint8_t>(ticks>>8)
    };
    sts3215::Packet packet;
    if (!sts3215::encode(j.servo_id,sts3215::kWrite,params,3,packet) ||
        !port_.discardInput() || !port_.startWrite(packet.bytes.data(),packet.size))
        return false;
    active_id_=j.servo_id;
    sending_=true;
    return true;
}
bool Sts3215Transport::finishSend() noexcept {
    if (!sending_ || faulted_) return false;
    sending_=false;
    if (!port_.finishWrite(config_.io_timeout)) {
        faulted_=true; // 无法证明最后停止位已发出，不允许继续运动。
        return false;
    }
    if (config_.expect_write_status) {
        sts3215::Status status;
        if (!readStatus(active_id_,0,status) || status.error!=0) {
            faulted_=true;
            return false;
        }
    }
    return true;
}
bool Sts3215Transport::readStatus(std::uint8_t servo_id,std::size_t count,
                                  sts3215::Status& status) noexcept {
    std::array<std::uint8_t,sts3215::kMaxPacket> data{};
    if (!port_.readExact(data.data(),4,config_.io_timeout)) return false;
    const auto remaining=static_cast<std::size_t>(data[3]);
    if (remaining<2 || remaining>data.size()-4) return false;
    if (!port_.readExact(data.data()+4,remaining,config_.io_timeout)) return false;
    return sts3215::decodeStatus(data.data(),remaining+4,servo_id,status) &&
           status.param_size==count;
}
bool Sts3215Transport::sendReadRequest(std::uint8_t servo_id) noexcept {
    const std::uint8_t params[]={sts3215::kPresentPosition,2};
    sts3215::Packet packet;
    if (!sts3215::encode(servo_id,sts3215::kRead,params,2,packet) ||
        !port_.discardInput() || !port_.startWrite(packet.bytes.data(),packet.size))
        return false;
    if (!port_.finishWrite(config_.io_timeout)) {
        faulted_=true;
        return false;
    }
    return true;
}
bool Sts3215Transport::readPositionTicks(std::size_t joint,std::uint16_t& ticks) {
    if (faulted_ || sending_ || joint>=kServoCount) return false;
    const auto servo_id=config_.joints[joint].servo_id;
    if (!sendReadRequest(servo_id)) return false;
    sts3215::Status status;
    if (!readStatus(servo_id,2,status) || status.error!=0) return false;
    const auto raw=static_cast<unsigned>(status.params[0]) |
                   (static_cast<unsigned>(status.params[1])<<8);
    if (raw>4095) return false;
    ticks=static_cast<std::uint16_t>(raw);
    return true;
}
bool Sts3215Transport::read(std::size_t joint,float& position,float& velocity) {
    std::uint16_t ticks=0;
    if (!readPositionTicks(joint,ticks)) return false;
    const auto& j=config_.joints[joint];
    const float angle=(static_cast<float>(ticks)-j.center_ticks) /
                      (j.direction*j.ticks_per_degree);
    if (!std::isfinite(angle) || angle<j.min_degree || angle>j.max_degree) return false;
    const auto now=ServoClock::now();
    if (!have_last_[joint]) {
        have_last_[joint]=true;
        last_position_[joint]=angle;
        last_time_[joint]=now;
        return false; // 首次读取没有速度信息，不能伪造 0 速度。
    }
    const float dt=std::chrono::duration<float>(now-last_time_[joint]).count();
    const float estimated=dt>0 ? (angle-last_position_[joint])/dt : 0.0f;
    last_position_[joint]=angle;
    last_time_[joint]=now;
    if (dt<=0.0001f || dt>1.0f || !std::isfinite(estimated)) return false;
    position=angle;
    velocity=estimated;
    return true;
}
}
