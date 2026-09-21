// =============================================================================
// cybergear_bus.hpp
//   여러 개의 CyberGear 를 "관절(joint) 단위"로 한꺼번에 다루는 클래스.
//
//   역할
//     1) 관절 i  <->  (어느 버스, 모터 ID) 연결
//     2) 모터 좌표 <-> 관절 좌표 변환 (회전 방향, 원점 오프셋)
//     3) 한 주기에 모든 모터에 명령 전송 + 도착한 피드백 수집
//
//   관절 좌표 정의
//     q_joint   = direction * q_motor + offset
//     tau_motor = direction * tau_joint
//     (direction = +1 또는 -1 : 모터 장착 방향이 DH 축과 반대면 -1)
// =============================================================================
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "cybergear_arm/can_transport.hpp"
#include "cybergear_arm/cybergear_protocol.hpp"

namespace cybergear {

struct JointConfig {
  std::string name = "joint";
  uint8_t motor_id = 1;
  size_t bus_index = 0;       // 몇 번째 전송 계층(버스)에 연결됐는가
  double direction = 1.0;     // +1 / -1
  double offset = 0.0;        // [rad]  q_joint = dir*q_motor + offset
  double torque_limit = 3.0;  // [Nm]   이 값을 넘는 명령은 잘라냄
};

struct JointState {
  double position = 0.0;  // [rad]   관절 좌표
  double velocity = 0.0;  // [rad/s]
  double torque = 0.0;    // [Nm]    모터가 보고한 토크 (관절 좌표 부호)
  double temperature = 0.0;
  uint8_t fault = 0;
  uint8_t mode = 0;
  bool valid = false;     // 한 번이라도 피드백을 받았는가
  std::chrono::steady_clock::time_point stamp{};  // 마지막 수신 시각
};

class CyberGearBus {
 public:
  CyberGearBus(std::vector<std::unique_ptr<CanTransport>> buses,
               std::vector<JointConfig> joints, uint8_t host_id = 0xFD);

  size_t size() const { return joints_.size(); }
  const JointConfig& config(size_t i) const { return joints_[i]; }
  const JointState& state(size_t i) const { return states_[i]; }

  // 에러 해제 -> 운동제어 모드(run_mode=0) -> 활성화.  (각 단계 사이 짧은 대기 포함)
  void startAll();

  // 토크 0 전송 후 정지(모터 free).  ※ 로봇팔은 중력으로 떨어진다! 받쳐 둘 것.
  void stopAll();

  // 관절 i 에 토크 명령 (관절 좌표, 제한 적용).  kd>0 이면 모터 내부 댐핑 추가.
  void sendTorque(size_t i, double tau_joint, double kd = 0.0);

  // 비상용 "댐핑 모드": 토크 0 + 모터 내부 Kd. 팔이 천천히 내려오게 한다.
  void sendDamping(size_t i, double kd);

  // 도착한 피드백을 모두 읽어 states_ 갱신. 이번에 갱신된 프레임 수를 돌려줌.
  int poll();

  // 가장 오래된 피드백이 몇 초 전인가 (통신 끊김 감시용)
  double oldestFeedbackAge() const;

 private:
  void sendRaw(size_t joint, const CanFrame& f);

  std::vector<std::unique_ptr<CanTransport>> buses_;
  std::vector<JointConfig> joints_;
  std::vector<JointState> states_;
  uint8_t host_id_;
  std::vector<CanFrame> rx_buf_;  // 매 주기 재사용 (메모리 할당 줄이기)
};

}  // namespace cybergear
