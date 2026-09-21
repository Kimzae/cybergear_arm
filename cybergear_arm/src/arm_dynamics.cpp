// =============================================================================
// arm_dynamics.cpp  —  2축 동역학 구현
// =============================================================================
#include "cybergear_arm/arm_dynamics.hpp"

#include <cmath>

namespace cybergear {

Arm2DofLumped lumpParameters(const Arm2DofPhysical& p) {
  Arm2DofLumped L;
  L.g = p.g;
  L.d2 = p.d2;

  // ---------------- 링크 2 ----------------
  // 구성: 프로파일(균일 막대) + 브래킷(점질량) + 끝단(점질량)
  const double m_prof = p.profile_lin_density * p.profile_length;
  const double x_prof = p.profile_start + 0.5 * p.profile_length;  // 프로파일 중심 위치

  L.m2 = m_prof + p.bracket2_mass + p.tip_mass;
  L.lc2 = (L.m2 > 0) ? (m_prof * x_prof + p.bracket2_mass * p.bracket2_pos + p.tip_mass * p.tip_pos) / L.m2
                     : 0.0;

  // 무게중심 기준 "옆으로 돌리는" 관성 (y2, z2 축).
  //   막대: m(L^2 + a^2)/12,   평행축 정리: + m*거리^2
  const double a = p.profile_side;
  L.It2 = m_prof * (p.profile_length * p.profile_length + a * a) / 12.0 +
          m_prof * std::pow(x_prof - L.lc2, 2) +
          p.bracket2_mass * std::pow(p.bracket2_pos - L.lc2, 2) +
          p.tip_mass * std::pow(p.tip_pos - L.lc2, 2);
  // 길이 방향(x2 축) 관성: 정사각 단면 m a^2 / 6
  L.Ia2 = m_prof * a * a / 6.0;

  // ---------------- 링크 1 ----------------
  // 관절1(z0 축)이 돌리는 것: 모터2 본체 (축이 수평인 원통) + 브래킷
  //   원통을 "옆으로" 돌리는 관성: m(3R^2 + h^2)/12,  + 평행축 m*r^2
  const double R = p.motor_radius, h = p.motor_length, mm = p.motor_mass;
  const double I_motor_trans = mm * (3 * R * R + h * h) / 12.0;
  L.Izz1 = I_motor_trans + mm * p.motor2_offset * p.motor2_offset + p.link1_extra_izz;

  L.Jr1 = p.rotor_reflected1;
  L.Jr2 = p.rotor_reflected2;
  L.fv1 = p.viscous1;
  L.fv2 = p.viscous2;
  L.fc1 = p.coulomb1;
  L.fc2 = p.coulomb2;
  return L;
}

void Arm2DofDynamics::compute(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                              Eigen::MatrixXd& M, Eigen::MatrixXd& C, Eigen::VectorXd& G,
                              Eigen::VectorXd& F) const {
  const auto& L = lp_;
  const double s = std::sin(q(1)), c = std::cos(q(1));

  const double A = L.Izz1 + L.m2 * L.d2 * L.d2 + L.Ia2 + L.Jr1;
  const double B = L.m2 * L.lc2 * L.lc2 + L.It2 - L.Ia2;
  const double D = L.m2 * L.d2 * L.lc2;
  const double E = L.It2 + L.m2 * L.lc2 * L.lc2 + L.Jr2;

  M.resize(2, 2);
  M << A + B * s * s, -D * c,
       -D * c,        E;

  // Christoffel 기호로 구한 C (헤더 주석 참고)
  C.resize(2, 2);
  C << B * s * c * dq(1),  B * s * c * dq(0) + D * s * dq(1),
       -B * s * c * dq(0), 0.0;

  // 중력: V = m2 g (d1 + lc2 cos q2)  ->  G = dV/dq
  G.resize(2);
  G << 0.0, -L.m2 * L.g * L.lc2 * s;

  // 마찰: 점성 + 쿨롱(tanh 로 부드럽게 근사)
  F.resize(2);
  F << L.fv1 * dq(0) + L.fc1 * std::tanh(dq(0) / 0.01),
       L.fv2 * dq(1) + L.fc2 * std::tanh(dq(1) / 0.01);
}

}  // namespace cybergear
