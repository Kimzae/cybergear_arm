// =============================================================================
// smc_controller.hpp
//   관절 공간 슬라이딩 모드 제어기 (Slotine–Li 형태) + 채터링 대책
//
//   e = q − q_d,   s = ė + Λe
//   목표 제어법칙:
//     τ = τ_model − K∘sat(s/φ) − Kd∘s − Ki∘∫s
//
//   ── 두 가지 실행 방식 (inner_loop) ─────────────────────────────────
//   HOST  : 위 식 전부를 PC에서 계산해서 "토크만" 보낸다.
//           → 속도 피드백(K/φ + Kd)이 USB 지연(수 ms)을 그대로 겪는다
//           → 이득이 조금만 커도 수십 Hz 로 떨린다 (= 영상의 채터링)
//
//   MOTOR : 선형 피드백 부분을 CyberGear 내부 PD 로 옮긴다 (권장).
//           −(Kd + K/φ)∘s = −kd_int(ė) − kp_int(e),
//              kd_int = Kd + K/φ,   kp_int = Λ∘kd_int
//           모터는 내부에서 (지연 없이, 빠른 주기로)
//              kp_int(p_ref − q) + kd_int(v_ref − q̇) + τ_ff
//           를 계산하므로, p_ref = q_d, v_ref = q̇_d 를 보내면 수학적으로 같은 식이 된다.
//           PC 가 보내는 τ_ff 는 느린 부분만:
//              τ_ff = τ_model − Ki∘∫s
//           경계층 안(|s|<φ)에서는 HOST 방식과 완전히 같은 식이다.
//           경계층 밖에서는 선형항 (K/φ)|s| ≥ K 이므로 도달 조건(reaching condition)은
//           오히려 더 강하게 만족된다. 대신 토크가 커질 수 있으므로 모터 자체
//           토크 제한(limit_torque, 0x700B)으로 막는다 (CyberGearBus::startAll 에서 설정).
//
//   τ_model
//     HOST  : M(q)q̈_r + C(q,q̇)q̇_r + G(q) + F(q̇)      (Slotine–Li, q̇_r = q̇_d − Λe)
//     MOTOR : M(q)q̈_d + C(q,q̇_d)q̇_d + G(q) + F(q̇_d)   ("desired compensation":
//             지연·노이즈 있는 측정 속도 대신 목표 궤적을 사용)
//
//   Ki∘∫s : 경계층 안에서만 적분 (조건부 적분, anti-windup).
//           경계층 때문에 남는 정상상태 오차를 없앤다. — ISMC 의 장점을 채터링 없이 얻는 방법
// =============================================================================
#pragma once

#include <memory>

#include <Eigen/Dense>

#include "cybergear_arm/arm_dynamics.hpp"

namespace cybergear {

enum class InnerLoop { kHost, kMotor };

struct SmcGains {
  Eigen::VectorXd lambda;  // Λ  [1/s]
  Eigen::VectorXd k;       // K  [Nm]
  Eigen::VectorXd phi;     // φ  [rad/s]
  Eigen::VectorXd kd;      // Kd [Nm/(rad/s)]
  Eigen::VectorXd ki;      // Ki [Nm/rad]         (0 이면 적분 없음)
  Eigen::VectorXd i_max;   // 적분항 최대 토크 [Nm] (anti-windup)
};

struct SmcOutput {
  // 모터로 보낼 값 (관절 좌표)
  Eigen::VectorXd tau;     // 토크 (HOST: 전체, MOTOR: 피드포워드 τ_ff)
  Eigen::VectorXd p_ref;   // MOTOR 모드 내부 PD 목표 위치 (= q_d)
  Eigen::VectorXd v_ref;   // MOTOR 모드 내부 PD 목표 속도 (= q̇_d)
  Eigen::VectorXd kp;      // MOTOR 모드 내부 Kp (HOST 모드면 0)
  Eigen::VectorXd kd;      // MOTOR 모드 내부 Kd (HOST 모드면 0)
  // 디버그
  Eigen::VectorXd tau_model;
  Eigen::VectorXd s;
  Eigen::VectorXd e;
};

class SmcController {
 public:
  // model 이 nullptr 이면 모델 없이 (K, Kd 만으로) 동작 -> 비교 실험용
  SmcController(std::shared_ptr<const ArmDynamics> model, SmcGains gains,
                InnerLoop mode = InnerLoop::kMotor, double dt = 0.002);

  SmcOutput compute(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                    const Eigen::VectorXd& q_d, const Eigen::VectorXd& dq_d,
                    const Eigen::VectorXd& ddq_d);

  void resetIntegral() { integ_.setZero(); }
  const SmcGains& gains() const { return gains_; }
  InnerLoop mode() const { return mode_; }

  // 모터 내부 PD 이득 (MOTOR 모드). CyberGear 범위(kp ≤ 500, kd ≤ 5)로 잘린 값
  const Eigen::VectorXd& motorKp() const { return kp_int_; }
  const Eigen::VectorXd& motorKd() const { return kd_int_; }

 private:
  std::shared_ptr<const ArmDynamics> model_;
  SmcGains gains_;
  InnerLoop mode_;
  double dt_;
  Eigen::VectorXd integ_;          // ∫s dt
  Eigen::VectorXd kp_int_, kd_int_;
  Eigen::MatrixXd M_, C_;
  Eigen::VectorXd G_, F_;
};

}  // namespace cybergear
