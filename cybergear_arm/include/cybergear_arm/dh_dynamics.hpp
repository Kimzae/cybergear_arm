// =============================================================================
// dh_dynamics.hpp
//   DH 파라미터 + 링크 부품 목록으로 만드는 "N축 범용" 동역학 모델.
//   (2축 해석해 Arm2DofDynamics 를 일반화한 것 — 4축, 6축 모두 이것으로)
//
//   계산 방법 (라그랑주 / 야코비안 합)
//     M(q) = Σ_i [ m_i Jv_iᵀ Jv_i + Jω_iᵀ (R_i I_i R_iᵀ) Jω_i ] + diag(armature)
//     G(q) = -Σ_i m_i Jv_iᵀ g
//     C(q,q̇): Christoffel 기호  c_ijk = ½(∂M_kj/∂q_i + ∂M_ki/∂q_j − ∂M_ij/∂q_k)
//             (∂M/∂q 는 중앙 차분으로 계산)  ->  Ṁ − 2C 반대칭 성질 유지 (SMC 증명에 필요)
//
//   좌표계 약속 (표준 DH)
//     - frame 0 = 전역 좌표계 (모터 1 축 = z0)
//     - 관절 i 는 z_{i-1} 축을 돈다
//     - frame i 는 링크 i 에 붙어 있다 -> 링크 i 의 부품 위치는 "frame i 기준" 으로 적는다
//     - θ_i = q_i + theta_offset_i
// =============================================================================
#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "cybergear_arm/arm_dynamics.hpp"

namespace cybergear {

struct DhRow {
  double theta_offset = 0, d = 0, a = 0, alpha = 0;  // [rad], [m], [m], [rad]
};

// 강체 1개의 관성 정보 (링크 i 의 모든 부품을 합친 결과)
struct RigidBody {
  double mass = 0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();      // frame i 기준 무게중심
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();  // 무게중심 기준 관성 텐서 (frame i 축)

  // 다른 강체를 합치기 (평행축 정리 사용)
  void add(const RigidBody& o);
};

// ---- 부품 -> 강체 변환 도우미 -----------------------------------------------
// axis: 0=x, 1=y, 2=z  /  sign: +1, -1
RigidBody makePointMass(double m, const Eigen::Vector3d& pos);
RigidBody makeCylinder(double m, double radius, double length, const Eigen::Vector3d& center, int axis);
// 사각 단면 막대(알루미늄 프로파일): start 에서 axis 방향(sign)으로 length 만큼
RigidBody makeSquareRod(double lin_density, double side, double length, const Eigen::Vector3d& start,
                        int axis, int sign);

struct DhJoint {
  std::string name;
  DhRow dh;
  RigidBody link;         // 이 관절이 움직이는 링크 (frame i 기준)
  double armature = 0;    // 회전자 반사 관성 [kg m^2]
  double viscous = 0;     // [Nm s/rad]
  double coulomb = 0;     // [Nm]
};

struct DhModel {
  Eigen::Vector3d gravity{0, 0, -9.81};
  std::vector<DhJoint> joints;
};

// yaml 파일에서 모델 읽기.  mass_scale: 모든 질량에 곱할 값 (시뮬레이터 오차 실험용)
DhModel loadDhModel(const std::string& path, double mass_scale = 1.0);

class DhDynamics : public ArmDynamics {
 public:
  explicit DhDynamics(DhModel model) : m_(std::move(model)) {}
  int dof() const override { return static_cast<int>(m_.joints.size()); }
  void compute(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, Eigen::MatrixXd& M,
               Eigen::MatrixXd& C, Eigen::VectorXd& G, Eigen::VectorXd& F) const override;

  // 개별 항 (테스트·디버그용)
  Eigen::MatrixXd massMatrix(const Eigen::VectorXd& q) const;
  Eigen::VectorXd gravityVector(const Eigen::VectorXd& q) const;
  // 정기구학: 각 frame 의 4x4 변환 (T[0] = 단위행렬, T[i] = frame i)
  std::vector<Eigen::Matrix4d> forwardKinematics(const Eigen::VectorXd& q) const;

  const DhModel& model() const { return m_; }

 private:
  DhModel m_;
};

}  // namespace cybergear
