// S5-03：协议黄金向量、映射、半双工事务、故障注入，不依赖硬件。
#include "robot/bus/Sts3215Protocol.hpp"
#include "robot/bus/Sts3215Transport.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
class FakePort final : public robot::IHalfDuplexPort {
public:
    bool startWrite(const std::uint8_t* bytes,std::size_t count) noexcept override {
        if (fail_start || active) return false;
        writes.emplace_back(bytes,bytes+count);
        active=true;
        return true;
    }
    bool finishWrite(std::chrono::milliseconds) noexcept override {
        if (!active || fail_finish) return false;
        active=false;
        if (!responses.empty()) {
            for (auto b:responses.front()) incoming.push_back(b);
            responses.pop_front();
        }
        return true;
    }
    bool readExact(std::uint8_t* data,std::size_t count,
                   std::chrono::milliseconds) noexcept override {
        if (incoming.size()<count) return false;
        for (std::size_t i=0;i<count;++i) {
            data[i]=incoming.front(); incoming.pop_front();
        }
        return true;
    }
    bool discardInput() noexcept override {
        if (active) return false;
        incoming.clear(); return true;
    }
    void respond(std::uint8_t id,std::uint8_t error,std::uint16_t ticks) {
        const std::uint8_t params[]={static_cast<std::uint8_t>(ticks&0xff),
                                      static_cast<std::uint8_t>(ticks>>8)};
        robot::sts3215::Packet packet;
        assert(robot::sts3215::encode(id,error,params,2,packet));
        responses.emplace_back(packet.bytes.begin(),packet.bytes.begin()+packet.size);
    }
    void ack(std::uint8_t id,std::uint8_t error=0) {
        robot::sts3215::Packet packet;
        assert(robot::sts3215::encode(id,error,nullptr,0,packet));
        responses.emplace_back(packet.bytes.begin(),packet.bytes.begin()+packet.size);
    }
    std::vector<std::vector<std::uint8_t>> writes;
    std::deque<std::vector<std::uint8_t>> responses;
    std::deque<std::uint8_t> incoming;
    bool active=false, fail_start=false, fail_finish=false;
};
robot::Sts3215Transport::Config config(bool motion=false) {
    robot::Sts3215Transport::Config c;
    c.allow_motion=motion;
    c.io_timeout=std::chrono::milliseconds(5);
    for (std::size_t i=0;i<robot::kServoCount;++i)
        c.joints[i].servo_id=static_cast<std::uint8_t>(i+1);
    return c;
}
}
int main() {
    using namespace robot;
    using namespace std::chrono;

    sts3215::Packet packet;
    assert(sts3215::encode(1,1,nullptr,0,packet));
    assert(packet.size==6);
    const std::array<std::uint8_t,6> ping{{0xff,0xff,1,2,1,0xfb}};
    for (std::size_t i=0;i<ping.size();++i) assert(packet.bytes[i]==ping[i]);
    const std::uint8_t read_params[]={56,2};
    assert(sts3215::encode(7,sts3215::kRead,read_params,2,packet));
    const std::array<std::uint8_t,8> expected_read{{0xff,0xff,7,4,2,56,2,0xb8}};
    for (std::size_t i=0;i<expected_read.size();++i)
        assert(packet.bytes[i]==expected_read[i]);

    sts3215::Status status;
    const std::uint8_t valid_status[]={0xff,0xff,1,4,0,0x34,0x12,0xb4};
    assert(sts3215::decodeStatus(valid_status,8,1,status));
    assert(status.param_size==2 && status.params[0]==0x34);
    std::array<std::uint8_t,8> corrupted{{0xff,0xff,1,4,0,0x34,0x12,0xb4}};
    corrupted[7]^=1;
    assert(!sts3215::decodeStatus(corrupted.data(),8,1,status));
    assert(!sts3215::decodeStatus(valid_status,8,2,status));

    FakePort port;
    Sts3215Transport safe(port,config());
    assert(!safe.startSend(0,0.0f)); // 默认不允许驱动舵机。
    assert(port.writes.empty());
    auto bad=config();
    bad.joints[1].servo_id=bad.joints[0].servo_id;
    bool rejected=false;
    try { Sts3215Transport invalid(port,bad); }
    catch (const std::invalid_argument&) { rejected=true; }
    assert(rejected);

    Sts3215Transport transport(port,config(true));
    assert(!transport.startSend(0,120.0f)); // 超机械限位。
    assert(!transport.startSend(16,0.0f));
    assert(transport.startSend(6,0.0f));
    assert(!transport.startSend(6,0.0f)); // 不允许同一时刻重入 TX。
    assert(transport.finishSend());
    assert(port.writes.size()==1);
    const std::array<std::uint8_t,9> write{{0xff,0xff,7,5,3,42,0,8,0xbe}};
    for (std::size_t i=0;i<write.size();++i) assert(port.writes[0][i]==write[i]);

    // 原始位置读取不依赖关节零位和速度估算，首次响应即有效。
    port.respond(7,0,2048);
    std::uint16_t raw_ticks=0;
    assert(transport.readPositionTicks(6,raw_ticks) && raw_ticks==2048);

    // 第一次读只获得位置；第二次读才能计算速度。
    port.respond(7,0,2048);
    float position=999,velocity=999;
    assert(!transport.read(6,position,velocity));
    assert(position==999 && velocity==999);
    std::this_thread::sleep_for(milliseconds(2));
    port.respond(7,0,2059);
    assert(transport.read(6,position,velocity));
    assert(position>0.0f && velocity>0.0f);

    // 错误字节、错误 ID、校验失败和超时都不能返回有效反馈。
    port.respond(7,4,2059);
    assert(!transport.read(6,position,velocity));
    port.respond(8,0,2059);
    assert(!transport.read(6,position,velocity));
    assert(!transport.read(6,position,velocity));

    // 配置为要求写 ACK 时，必须检查 ID 和错误码。
    FakePort ackport;
    auto ackcfg=config(true);
    ackcfg.expect_write_status=true;
    Sts3215Transport with_ack(ackport,ackcfg);
    ackport.ack(1);
    assert(with_ack.startSend(0,0.0f));
    assert(with_ack.finishSend());
    ackport.ack(1,4);
    assert(with_ack.startSend(0,0.0f));
    assert(!with_ack.finishSend());
    assert(with_ack.faulted());
    assert(!with_ack.startSend(0,0.0f));

    FakePort broken;
    Sts3215Transport fails(broken,config(true));
    assert(fails.startSend(0,0.0f));
    broken.fail_finish=true;
    assert(!fails.finishSend()); // UART TC 不确定：锁存故障。
    assert(fails.faulted() && !fails.startSend(0,0.0f));
}
