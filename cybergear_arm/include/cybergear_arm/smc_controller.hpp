// =============================================================================
// smc_controller.hpp
//   관절 공간 슬라이딩 모드 제어기 (Slotine–Li 형태)
//
//   오차          e   = q  - q_d
//   기준 속도     q'_r = q'_d  - Λ e
//   기준 가속도   q''_r = q''_d - Λ e'
//   슬라이딩 면   s   = q' - q'_r = e' + Λ e
//
//   제어 입력
//     τ = M̂ q''_r + Ĉ q'_r + Ĝ + F̂        ← 모델 기반 (등가 제어, equivalent control)
//         - K ∘ sat(s / φ)                 ← 스위칭 항: 모델 오차를 이겨내는 부분
//         - Kd ∘ s                         ← 선형 댐핑 (채터링 감소, 수렴 가속)
//
//   직관
//     - s = 0 이면 e' = -Λ e  ->  오차가 지수적으로 0으로 감 (시정수 1/Λ)
//     - K 는 "모델이 틀린 정도(토크 단위)"보다 커야 s -> 0 이 보장된다.
//       이 로봇은 치수가 부정확하므로 K 가 곧 "불확실성 예산"이다.
//     - φ(경계층 두께): sign() 대신 sat() 를 써서 모터 떨림(채터링)을 줄인다.
//       φ 가 크면 부드럽지만 정상상태 오차가 약간 남는다.
// =============================================================================
#pragma once

#include <memory>

#include <Eigen/Dense>

#include "cybergear_arm/arm_dynamics.hpp"

namespace cybergear {

struct SmcGains {
  Eigen::VectorXd lambda;  // Λ  [1/s]
  Eigen::VectorXd k;       // K  [Nm]
  Eigen::VectorXd phi;     // φ  [rad/s]
  Eigen::VectorXd kd;      // Kd [Nm/(rad/s)]
};

struct SmcOutput {
  Eigen::VectorXd tau;       // 최종 토크 명령
  Eigen::VectorXd tau_model; // 모델 기반 부분 (디버그용)
  Eigen::VectorXd s;         // 슬라이딩 변수 (디버그용)
  Eigen::VectorXd e;         // 위치 오차
};

class SmcController {
 public:
  // model 이 nullptr 이면 모델 없이 (K, Kd 만으로) 동작 -> 비교 실험용
  SmcController(std::shared_ptr<const ArmDynamics> model, SmcGains gains);

  SmcOutput compute(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                    const Eigen::VectorXd& q_d, const Eigen::VectorXd& dq_d,
                    const Eigen::VectorXd& ddq_d);

  const SmcGains& gains() const { return gains_; }

 private:
  std::shared_ptr<const ArmDynamics> model_;
  SmcGains gains_;
  // 매 주기 메모리 할당을 피하려고 멤버로 보관
  Eigen::MatrixXd M_, C_;
  Eigen::VectorXd G_, F_;
};

}  // namespace cybergear
