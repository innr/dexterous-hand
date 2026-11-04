#include "robot/bus/PosixHalfDuplexPort.hpp"
#include <stdexcept>
#if defined(__linux__)
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#endif
namespace robot {
PosixHalfDuplexPort::PosixHalfDuplexPort(const std::string& device, unsigned baud) {
#if defined(__linux__) && defined(B1000000)
    if (baud!=1000000) throw std::invalid_argument("only 1Mbps is supported");
    fd_=::open(device.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if (fd_<0) throw std::runtime_error("serial open failed");
    termios tty{};
    if (::tcgetattr(fd_,&tty)!=0) {
        ::close(fd_); fd_=-1; throw std::runtime_error("tcgetattr failed");
    }
    ::cfmakeraw(&tty);
    ::cfsetispeed(&tty,B1000000);
    ::cfsetospeed(&tty,B1000000);
    tty.c_cflag |= CLOCAL|CREAD;
    tty.c_cflag &= ~(CSTOPB|PARENB|CSIZE);
    tty.c_cflag |= CS8;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    if (::tcsetattr(fd_,TCSANOW,&tty)!=0) {
        ::close(fd_); fd_=-1; throw std::runtime_error("tcsetattr failed");
    }
#else
    (void)device; (void)baud;
    throw std::runtime_error("Linux 1Mbps serial required");
#endif
}
PosixHalfDuplexPort::~PosixHalfDuplexPort() {
#if defined(__linux__)
    if (fd_>=0) ::close(fd_);
#endif
}
bool PosixHalfDuplexPort::startWrite(const std::uint8_t* data,std::size_t count) noexcept {
#if defined(__linux__)
    if (fd_<0 || active_ || faulted_ || !data || count==0 || count>pending_.size()) return false;
    for (std::size_t i=0;i<count;++i) pending_[i]=data[i];
    const auto n=::write(fd_,pending_.data(),count); // O_NONBLOCK，不能在门禁锁内阻塞。
    if (n<=0) return false;
    size_=count; offset_=static_cast<std::size_t>(n); active_=true;
    return true;
#else
    (void)data; (void)count; return false;
#endif
}
bool PosixHalfDuplexPort::finishWrite(std::chrono::milliseconds timeout) noexcept {
#if defined(__linux__)
    if (!active_ || faulted_ || timeout<=std::chrono::milliseconds::zero()) return false;
    const auto deadline=std::chrono::steady_clock::now()+timeout;
    while (std::chrono::steady_clock::now()<deadline) {
        if (offset_<size_) {
            const auto n=::write(fd_,pending_.data()+offset_,size_-offset_);
            if (n>0) offset_+=static_cast<std::size_t>(n);
            else if (n<0 && errno!=EAGAIN && errno!=EINTR) break;
        } else {
            int queue=0;
            if (::ioctl(fd_,TIOCOUTQ,&queue)!=0) break;
            if (queue==0) {
#ifdef TIOCSERGETLSR
                int lsr=0;
                if (::ioctl(fd_,TIOCSERGETLSR,&lsr)!=0) break;
                if ((lsr&TIOCSER_TEMT)!=0) { active_=false; return true; }
#else
                break;
#endif
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    faulted_=true; // 无法证明 TC，永久禁止后续写入。
    return false;
#else
    (void)timeout; return false;
#endif
}
bool PosixHalfDuplexPort::readExact(std::uint8_t* data,std::size_t count,
                                    std::chrono::milliseconds timeout) noexcept {
#if defined(__linux__)
    if (fd_<0 || active_ || faulted_ || (count && !data) ||
        timeout<=std::chrono::milliseconds::zero()) return false;
    const auto deadline=std::chrono::steady_clock::now()+timeout;
    std::size_t done=0;
    while (done<count) {
        const auto now=std::chrono::steady_clock::now();
        if (now>=deadline) return false;
        const auto left=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-now);
        pollfd pfd{fd_,POLLIN,0};
        const int rc=::poll(&pfd,1,static_cast<int>(left.count())+1);
        if (rc<0 && errno==EINTR) continue;
        if (rc<=0 || (pfd.revents&(POLLERR|POLLHUP|POLLNVAL))) return false;
        const auto n=::read(fd_,data+done,count-done);
        if (n>0) done+=static_cast<std::size_t>(n);
        else if (n==0 || (errno!=EAGAIN && errno!=EINTR)) return false;
    }
    return true;
#else
    (void)data; (void)count; (void)timeout; return false;
#endif
}
bool PosixHalfDuplexPort::discardInput() noexcept {
#if defined(__linux__)
    return fd_>=0 && !active_ && !faulted_ && ::tcflush(fd_,TCIFLUSH)==0;
#else
    return false;
#endif
}
}
