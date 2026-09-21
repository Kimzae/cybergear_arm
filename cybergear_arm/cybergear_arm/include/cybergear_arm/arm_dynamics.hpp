// =============================================================================
// arm_dynamics.hpp
//   로봇팔 동역학 모델:   M(q) q'' + C(q,q') q' + G(q) + F(q') = tau
//
//   - ArmDynamics      : N축 공통 인터페이스 (6축으로 늘릴 때 이것만 새로 구현)
//   - Arm2DofDynamics  : 지금의 2축(모터 1, 2) 해석적(closed-form) 모델
//
//   ── 2축 기구 (사용자 DH) ─────────────────────────────────────────
//        i | theta          | d  | a | alpha
//        1 | q1             | d1 | 0 | +90°
//        2 | q2 + 90° (*)   | d2 | 0 |   0
//
//   (*) "모터 2 원점에서 x2 축이 전역 z 축과 같다" 는 조건을 만족하려면
//       DH 의 theta2 = q2 + 90° 여야 한다. (theta2 = q2 이면 x2 = 전역 x 가 됨)
//       -> 코드에서는 q2 를 "수직(위쪽)에서 잰 각도"로 쓴다.
//
//   ── 링크 2 (20x20 알루미늄 프로파일) ─────────────────────────────
//       DH 에서 a2 = 0 이므로 frame 2 원점은 관절2 축 위에 있다.
//       프로파일은 x2 방향(q2=0 일 때 위쪽)으로 뻗어 있다고 가정하고,
//       무게중심 거리 lc2 를 x2 축 위에 둔다.
//
//   ── 유도 결과 (s = sin q2, c = cos q2) ───────────────────────────
//       A = Izz1 + m2*d2^2 + Ia2 + Jr1
//       B = m2*lc2^2 + It2 - Ia2
//       D = m2*d2*lc2
//       E = It2 + m2*lc2^2 + Jr2
//
//       M = [ A + B s^2     -D c ]
//           [   -D c          E  ]
//
//       C = [ B s c q2'     B s c q1' + D s q2' ]
//           [ -B s c q1'          0             ]
//
//       G = [ 0 , -m2 g lc2 s ]^T
//
//       (M' - 2C 가 반대칭(skew-symmetric)임을 확인함 -> SMC 안정성 증명에 사용)
// =============================================================================
#pragma once

#include <Eigen/Dense>

namespace cybergear {

// ---- N축 공통 인터페이스 ---------------------------------------------------
class ArmDynamics {
 public:
  virtual ~ArmDynamics() = default;
  virtual int dof() const = 0;
  // q, dq 를 넣으면 M, C, G, F 를 채워준다
  virtual void compute(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, Eigen::MatrixXd& M,
                       Eigen::MatrixXd& C, Eigen::VectorXd& G, Eigen::VectorXd& F) const = 0;
};

// ---- 2축 모델의 물리 파라미터 ----------------------------------------------
//   값을 모르면 기본값으로 시작하되, 저울/자로 실측해서 yaml 에 넣을 것.
//   (불확실한 만큼 SMC 의 K(스위칭 이득)를 키워야 한다)
struct Arm2DofPhysical {
  double g = 9.81;

  // DH
  double d2 = 0.05;  // [m] frame1 -> frame2, 관절2 축(z1) 방향 오프셋

  // CyberGear 한 개 (원통 근사)
  double motor_mass = 0.317;    // [kg]  스펙값. 실측 권장
  double motor_radius = 0.040;  // [m]   ※ 측정 필요 (대략값)
  double motor_length = 0.042;  // [m]   ※ 측정 필요 (대략값)

  // 링크 1 (관절1이 돌리는 것: 모터2 본체 + 브래킷)
  double motor2_offset = 0.03;  // [m] 모터2 중심이 o1 에서 z1 방향으로 떨어진 거리
  double link1_extra_izz = 0.0; // [kg m^2] 브래킷 등 추가 관성 (z0 축 기준)

  // 링크 2 (20x20 프로파일 + 끝단 질량)
  double profile_length = 0.20;      // [m]
  double profile_start = 0.0;        // [m] 관절2 축에서 프로파일 시작점까지 (x2 방향)
  double profile_lin_density = 0.45; // [kg/m] ※ 제품 스펙 확인 (대략 0.4~0.5)
  double profile_side = 0.02;        // [m] 20x20 -> 0.02
  double bracket2_mass = 0.0;        // [kg] 링크2 쪽 브래킷
  double bracket2_pos = 0.0;         // [m]  x2 방향 위치
  double tip_mass = 0.0;             // [kg] 끝단(엔드이펙터+물체)
  double tip_pos = 0.20;             // [m]  x2 방향 위치

  // 모터 회전자 반사 관성 (감속비^2 * 회전자 관성). 모르면 0 -> 불확실성으로 처리
  double rotor_reflected1 = 0.0;  // [kg m^2]
  double rotor_reflected2 = 0.0;

  // 마찰 (모르면 0)
  double viscous1 = 0.0, viscous2 = 0.0;  // [Nm/(rad/s)]
  double coulomb1 = 0.0, coulomb2 = 0.0;  // [Nm]
};

// 물리 파라미터 -> 식에 들어가는 "집약 파라미터"
struct Arm2DofLumped {
  double m2 = 0, lc2 = 0, It2 = 0, Ia2 = 0, Izz1 = 0, d2 = 0, g = 9.81;
  double Jr1 = 0, Jr2 = 0, fv1 = 0, fv2 = 0, fc1 = 0, fc2 = 0;
};

Arm2DofLumped lumpParameters(const Arm2DofPhysical& p);

class Arm2DofDynamics : public ArmDynamics {
 public:
  explicit Arm2DofDynamics(const Arm2DofPhysical& p) : lp_(lumpParameters(p)) {}
  explicit Arm2DofDynamics(const Arm2DofLumped& lp) : lp_(lp) {}

  int dof() const override { return 2; }
  void compute(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, Eigen::MatrixXd& M,
               Eigen::MatrixXd& C, Eigen::VectorXd& G, Eigen::VectorXd& F) const override;

  const Arm2DofLumped& lumped() const { return lp_; }

 private:
  Arm2DofLumped lp_;
};

}  // namespace cybergear
