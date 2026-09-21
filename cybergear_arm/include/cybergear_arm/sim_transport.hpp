// =============================================================================
// sim_transport.hpp
//   하드웨어 없이 제어기를 시험하기 위한 "가짜 모터 + 가짜 로봇팔".
//
//   - CanTransport 인터페이스를 구현하므로 CyberGearBus 입장에서는 진짜 모터와 같다.
//   - 받은 운동제어(type 1) 프레임을 해석해서 모터 내부 식
//       T = kp(p - pos) + kd(v - vel) + t_ff
//     을 계산하고, "진짜" 동역학 모델로 적분한 뒤 피드백(type 2) 프레임을 돌려준다.
//   - 피드백 값은 실제처럼 16비트로 양자화된다.
//   - 제어기용 모델과 다른 파라미터(true_model)를 넣으면 "모델 오차" 실험이 된다.
// =============================================================================
#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "cybergear_arm/arm_dynamics.hpp"
#include "cybergear_arm/can_transport.hpp"
#include "cybergear_arm/cybergear_bus.hpp"

namespace cybergear {

class SimTransport : public CanTransport {
 public:
  // joints : 관절-모터 연결 정보 (방향/오프셋 변환에 사용)
  // q0     : 초기 관절 각도
  SimTransport(std::shared_ptr<const ArmDynamics> true_model, std::vector<JointConfig> joints,
               Eigen::VectorXd q0, uint8_t host_id = 0xFD);

  bool send(const CanFrame& frame) override;
  void receive(std::vector<CanFrame>& out) override;
  std::string name() const override { return "sim"; }

  // 시뮬레이션 시간을 dt 만큼 진행 (테스트에서 실시간과 무관하게 돌릴 때)
  void step(double dt);
  // true 면 receive() 가 실제 경과 시간만큼 자동으로 step() 한다 (ROS 노드용)
  void setRealtime(bool on) { realtime_ = on; }

  // 피드백 지연 [s]: PC 가 받는 상태가 이만큼 과거의 것 (USB/어댑터 지연 흉내)
  //   모터 내부 PD 는 지연 없는 현재 상태를 쓴다 — 실제 CyberGear 와 같은 구조
  void setFeedbackDelay(double sec) { delay_ = sec; }

  const Eigen::VectorXd& q() const { return q_; }
  double time() const { return t_; }
  // 마지막 적분 스텝에서 모터가 실제로 낸 토크 (관절 좌표, 모터 내부 PD 포함)
  const Eigen::VectorXd& appliedTorque() const { return tau_applied_; }

 private:
  struct MotorCmd {
    bool enabled = false;
    double t_ff = 0, p = 0, v = 0, kp = 0, kd = 0;
    double limit = 12.0;  // limit_torque 파라미터
    bool replied = true;  // 명령을 받으면 false -> 다음 receive 때 피드백 1개 응답
  };

  std::shared_ptr<const ArmDynamics> model_;
  std::vector<JointConfig> joints_;
  uint8_t host_id_;
  Eigen::VectorXd q_, dq_;
  std::vector<MotorCmd> cmd_;
  bool realtime_ = false;
  double delay_ = 0.0;
  double t_ = 0.0;  // 시뮬레이션 시각
  Eigen::VectorXd tau_applied_;
  struct Snap { double t; Eigen::VectorXd q, dq; };
  std::vector<Snap> hist_;  // 지연 구현용 과거 상태 기록
  std::chrono::steady_clock::time_point last_{std::chrono::steady_clock::now()};
};

}  // namespace cybergear
