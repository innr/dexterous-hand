// S5-05：验证第 1 路完成后即可被控制线程看到，及分片公平性、错误隔离。
#include "robot/runtime/S4Pipeline.hpp"
#include "robot/bus/IServoTransport.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
class BlockingBus final : public robot::IServoTransport {
public:
    bool startSend(std::size_t, float) noexcept override { return false; }
    bool finishSend() noexcept override { return false; }
    bool read(std::size_t id, float& position, float& velocity) override {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            order.push_back(id);
            if (id==1 && block_second_) {
                second_entered_=true;
                cv_.notify_all();
                cv_.wait(lock,[&] { return released_; });
            }
        }
        if (id==6 && fail_six_) return false;
        position=static_cast<float>(id);
        velocity=0.0f;
        return true;
    }
    bool waitForSecond() {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock,std::chrono::seconds(2),[&] { return second_entered_; });
    }
    void release() {
        std::lock_guard<std::mutex> lock(mutex_);
        released_=true;
        cv_.notify_all();
    }
    void recoverSix() { fail_six_=false; }
    std::vector<std::size_t> order;
private:
    std::mutex mutex_;
    std::condition_variable cv_;
    bool block_second_=true, second_entered_=false, released_=false;
    bool fail_six_=true;
};
}

int main() {
    using namespace robot;
    using namespace std::chrono;
    FeedbackBuffer feedback;
    CommandMailbox mailbox;
    StopCoordinator stop;
    BlockingBus bus;
    CommunicationWorker worker(bus,feedback,mailbox,stop,milliseconds(50));

    bool invalid=false;
    try { (void)worker.pollFeedbackSliceRealTime(0); }
    catch (const std::invalid_argument&) { invalid=true; }
    assert(invalid);
    invalid=false;
    try { (void)worker.pollFeedbackSliceRealTime(17); }
    catch (const std::invalid_argument&) { invalid=true; }
    assert(invalid);

    PollBatchResult first;
    std::thread reader([&] { first=worker.pollFeedbackSliceRealTime(4); });
    const bool entered=bus.waitForSecond();
    if (!entered) {
        bus.release();
        reader.join();
        assert(false && "timed out waiting for second servo read");
    }
    // 第二路还未读完：第一路必须已发布，不能等到完整 16 路结束。
    const auto partial=feedback.read();
    assert(partial.sequence==1);
    assert(partial.valid[0] && partial.position[0]==0.0f);
    for (std::size_t i=1;i<kServoCount;++i) assert(!partial.valid[i]);
    bus.release();
    reader.join();
    assert(first.attempted==4 && !first.completed_round);
    assert(feedback.read().sequence==4);

    assert(!worker.pollFeedbackSliceRealTime(4).completed_round);
    assert(!worker.pollFeedbackSliceRealTime(4).completed_round);
    const auto fourth=worker.pollFeedbackSliceRealTime(4);
    assert(fourth.attempted==4 && fourth.completed_round);
    const auto full=feedback.read();
    assert(full.sequence==16);
    assert(!full.valid[6]);
    for (std::size_t i=0;i<kServoCount;++i) {
        assert(bus.order[i]==i); // 不跳过、不重复读取。
        if (i!=6) {
            assert(full.valid[i]);
            assert(full.position[i]==static_cast<float>(i));
        }
    }
    assert(full.timestamp[0]<=full.timestamp[15]);
    const auto before=full.timestamp[6];
    const auto failed_at=full.sequence;
    bus.recoverSix();
    assert(!worker.pollFeedbackSliceRealTime(4).completed_round);
    assert(!worker.pollFeedbackSliceRealTime(4).completed_round);
    const auto recovered=feedback.read();
    assert(recovered.sequence==failed_at+8);
    assert(recovered.valid[6] && recovered.position[6]==6.0f);
    assert(recovered.timestamp[6]>=before);
    assert(bus.order[16]==0 && bus.order[22]==6);

    // 失败更新不能覆盖上次成功的位置和时间戳。
    const auto saved=recovered.timestamp[6];
    const auto seq=recovered.sequence;
    assert(feedback.updateJoint(6,999.0f,999.0f,ServoClock::now(),false));
    const auto failed=feedback.read();
    assert(!failed.valid[6] && failed.position[6]==6.0f);
    assert(failed.timestamp[6]==saved && failed.sequence==seq+1);
    assert(!feedback.updateJoint(16,0.0f,0.0f,ServoClock::now(),true));
    assert(feedback.read().sequence==seq+1);
}
