// =============================================================================
// socketcan_transport.cpp
//   리눅스 표준 CAN 인터페이스(SocketCAN)용 전송 계층.
//   candleLight / PCAN-USB 같은 어댑터는 "can0" 같은 네트워크 장치로 보인다.
//
//   사용 전 (터미널):
//     sudo ip link set can0 up type can bitrate 1000000
//
//   ※ 1kHz x 6축 목표라면 이 방식 + 버스 여러 개가 필요하다 (ARCHITECTURE.md 참고)
// =============================================================================
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "cybergear_arm/can_transport.hpp"

namespace cybergear {

namespace {

class SocketCanTransport : public CanTransport {
 public:
  explicit SocketCanTransport(const std::string& ifname) : ifname_(ifname) {
    sock_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (sock_ < 0) throw std::runtime_error("CAN 소켓 생성 실패");

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      throw std::runtime_error("CAN 인터페이스 없음: " + ifname + " (ip link 로 확인)");
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      throw std::runtime_error("CAN bind 실패: " + std::string(std::strerror(errno)));
    }
  }

  ~SocketCanTransport() override {
    if (sock_ >= 0) ::close(sock_);
  }

  bool send(const CanFrame& f) override {
    can_frame cf{};
    cf.can_id = (f.id & CAN_EFF_MASK) | CAN_EFF_FLAG;  // 확장(29비트) 프레임
    cf.can_dlc = f.len;
    std::memcpy(cf.data, f.data.data(), f.len);
    return ::write(sock_, &cf, sizeof(cf)) == sizeof(cf);
  }

  void receive(std::vector<CanFrame>& out) override {
    can_frame cf;
    while (::read(sock_, &cf, sizeof(cf)) == sizeof(cf)) {
      if (!(cf.can_id & CAN_EFF_FLAG)) continue;  // 표준 프레임은 무시
      CanFrame f;
      f.id = cf.can_id & CAN_EFF_MASK;
      f.len = cf.can_dlc;
      std::memcpy(f.data.data(), cf.data, cf.can_dlc);
      out.push_back(f);
    }
  }

  std::string name() const override { return "socketcan:" + ifname_; }

 private:
  std::string ifname_;
  int sock_ = -1;
};

}  // namespace

std::unique_ptr<CanTransport> makeSocketCanTransport(const std::string& ifname) {
  return std::make_unique<SocketCanTransport>(ifname);
}

}  // namespace cybergear
