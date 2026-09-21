// =============================================================================
// can_transport.hpp
//   "CAN 프레임을 어떻게 실어 나르는가"를 추상화한 인터페이스.
//
//   왜 필요한가?
//     - 지금은 CH340 USB-CAN 시리얼 어댑터(AT 프레임)를 쓰지만,
//       1kHz x 6축으로 가려면 SocketCAN 어댑터 여러 개로 바꿔야 한다.
//     - 하드웨어 없이 제어기를 시험하려면 시뮬레이터가 필요하다.
//   -> 셋 다 이 인터페이스만 구현하면, 위쪽 코드(CyberGearBus, 제어기)는 그대로 쓴다.
// =============================================================================
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cybergear_arm/cybergear_protocol.hpp"

namespace cybergear {

class CanTransport {
 public:
  virtual ~CanTransport() = default;

  // 프레임 1개 전송. 실패하면 false.
  virtual bool send(const CanFrame& frame) = 0;

  // 지금까지 도착한 프레임을 모두 out 에 추가 (기다리지 않음 = non-blocking)
  virtual void receive(std::vector<CanFrame>& out) = 0;

  // 사람이 읽을 이름 (로그용)
  virtual std::string name() const = 0;
};

// ---- 구현체 생성 함수 ------------------------------------------------------
// CH340 USB-CAN 어댑터 (시리얼 'AT' 프레임). 예: "/dev/ttyUSB0", 921600
std::unique_ptr<CanTransport> makeSerialAtTransport(const std::string& port, int baudrate);

// 리눅스 SocketCAN. 예: "can0"  (사전에: sudo ip link set can0 up type can bitrate 1000000)
std::unique_ptr<CanTransport> makeSocketCanTransport(const std::string& ifname);

}  // namespace cybergear
