#include "robot/bus/Sts3215Protocol.hpp"
namespace robot { namespace sts3215 {
namespace {
std::uint8_t checksum(const std::uint8_t* data, std::size_t count) noexcept {
    unsigned sum = 0;
    for (std::size_t i=0;i<count;++i) sum += data[i];
    return static_cast<std::uint8_t>(~sum);
}
}
bool encode(std::uint8_t id, std::uint8_t instruction, const std::uint8_t* params,
            std::size_t count, Packet& output) noexcept {
    if (id>=254 || count>kMaxPacket-6 || (count && !params)) return false;
    output = {};
    output.size = count+6;
    output.bytes[0]=0xff; output.bytes[1]=0xff;
    output.bytes[2]=id; output.bytes[3]=static_cast<std::uint8_t>(count+2);
    output.bytes[4]=instruction;
    for (std::size_t i=0;i<count;++i) output.bytes[5+i]=params[i];
    output.bytes[output.size-1]=checksum(output.bytes.data()+2,output.size-3);
    return true;
}
bool decodeStatus(const std::uint8_t* data, std::size_t size,
                  std::uint8_t expected_id, Status& output) noexcept {
    if (!data || size<6 || size>kMaxPacket || data[0]!=0xff || data[1]!=0xff ||
        data[2]!=expected_id || data[3]<2 || static_cast<std::size_t>(data[3])+4!=size ||
        checksum(data+2,size-3)!=data[size-1]) return false;
    output = {};
    output.id=data[2]; output.error=data[4]; output.param_size=size-6;
    for (std::size_t i=0;i<output.param_size;++i) output.params[i]=data[5+i];
    return true;
}
} }
