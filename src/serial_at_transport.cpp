// =============================================================================
// serial_at_transport.cpp
//   CH340 기반 CyberGear USB-CAN 어댑터용 전송 계층.
//
//   어댑터 시리얼 프레임:
//     'A' 'T' | ID 4바이트(big-endian) | 길이 1바이트 | 데이터 | '\r' '\n'
//     ID 4바이트 = (29비트 CAN ID << 3) | 0x04   (0x04 = 확장 프레임 표시)
//
//   termios = 리눅스에서 시리얼 포트 설정(속도, 패리티 등)을 하는 표준 API
// =============================================================================
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "cybergear_arm/can_transport.hpp"

namespace cybergear {

namespace {

speed_t toSpeed(int baud) {
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: throw std::runtime_error("지원하지 않는 baudrate: " + std::to_string(baud));
  }
}

class SerialAtTransport : public CanTransport {
 public:
  SerialAtTransport(const std::string& port, int baud) : port_(port) {
    // O_NONBLOCK: read()가 데이터 없을 때 기다리지 않고 바로 돌아옴
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("시리얼 포트 열기 실패: " + port + " (" + std::strerror(errno) +
                               ")  -> 권한(dialout)·포트 이름 확인");
    }
    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);                 // 바이너리 그대로 주고받는 "raw" 모드
    cfsetispeed(&tio, toSpeed(baud));
    cfsetospeed(&tio, toSpeed(baud));
    tio.c_cflag |= (CLOCAL | CREAD); // 모뎀 제어선 무시, 수신 활성화
    tio.c_cflag &= ~CRTSCTS;         // 하드웨어 흐름제어 끔
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSANOW, &tio);
    tcflush(fd_, TCIOFLUSH);         // 이전에 쌓인 쓰레기 데이터 비우기

    const char init[] = "AT+AT\r\n"; // 어댑터를 AT 모드로 전환
    if (::write(fd_, init, sizeof(init) - 1) < 0) { /* 초기화 실패해도 이후 통신으로 확인 */ }
  }

  ~SerialAtTransport() override {
    if (fd_ >= 0) ::close(fd_);
  }

  bool send(const CanFrame& f) override {
    uint8_t buf[17];
    const uint32_t raw = (f.id << 3) | 0x04;
    buf[0] = 'A';
    buf[1] = 'T';
    buf[2] = raw >> 24;
    buf[3] = raw >> 16;
    buf[4] = raw >> 8;
    buf[5] = raw & 0xFF;
    buf[6] = f.len;
    std::memcpy(&buf[7], f.data.data(), f.len);
    buf[7 + f.len] = '\r';
    buf[8 + f.len] = '\n';
    const ssize_t n = 9 + f.len;
    return ::write(fd_, buf, n) == n;
  }

  void receive(std::vector<CanFrame>& out) override {
    // 1) 커널 버퍼에 있는 바이트를 전부 rx_ 로 옮긴다
    uint8_t tmp[512];
    ssize_t n;
    while ((n = ::read(fd_, tmp, sizeof(tmp))) > 0) rx_.insert(rx_.end(), tmp, tmp + n);

    // 2) rx_ 에서 완성된 'AT' 프레임을 하나씩 잘라낸다 (Python poll()과 같은 로직)
    size_t i = 0;
    while (true) {
      // 'A','T' 시작 찾기
      while (i + 1 < rx_.size() && !(rx_[i] == 'A' && rx_[i + 1] == 'T')) ++i;
      if (i + 7 > rx_.size()) break;              // 헤더가 아직 덜 옴
      const uint8_t len = rx_[i + 6];
      if (len > 8) { ++i; continue; }             // 잘못된 프레임 -> 1바이트 건너뜀
      const size_t total = 9 + len;
      if (i + total > rx_.size()) break;          // 데이터가 아직 덜 옴
      if (rx_[i + total - 2] != '\r' || rx_[i + total - 1] != '\n') { ++i; continue; }

      const uint32_t raw = (uint32_t(rx_[i + 2]) << 24) | (uint32_t(rx_[i + 3]) << 16) |
                           (uint32_t(rx_[i + 4]) << 8) | uint32_t(rx_[i + 5]);
      CanFrame f;
      f.id = raw >> 3;
      f.len = len;
      std::memcpy(f.data.data(), &rx_[i + 7], len);
      out.push_back(f);
      i += total;
    }
    rx_.erase(rx_.begin(), rx_.begin() + static_cast<long>(std::min(i, rx_.size())));
  }

  std::string name() const override { return "serial-at:" + port_; }

 private:
  std::string port_;
  int fd_ = -1;
  std::vector<uint8_t> rx_;  // 아직 해석 안 된 수신 바이트
};

}  // namespace

std::unique_ptr<CanTransport> makeSerialAtTransport(const std::string& port, int baudrate) {
  return std::make_unique<SerialAtTransport>(port, baudrate);
}

}  // namespace cybergear
