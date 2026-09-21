// =============================================================================
// model_test.cpp
//   DH 범용 동역학 모델 검증 (하드웨어 불필요)
//     1) 2축: 범용 모델(model_2dof.yaml) == 해석해(Arm2DofDynamics)
//     2) 4축: Ṁ − 2C 반대칭  (SMC 안정성 증명의 전제)
//     3) 4축: G = ∂V/∂q       (중력 항이 위치 에너지와 일치)
//   실행: ros2 run cybergear_arm model_test $(ros2 pkg prefix cybergear_arm)/share/cybergear_arm/config
// =============================================================================
#include <cstdio>
#include <random>
#include <string>

#include "cybergear_arm/arm_dynamics.hpp"
#include "cybergear_arm/dh_dynamics.hpp"

using namespace cybergear;

static int fails = 0;
static void check(bool ok, const char* what, double val) {
  std::printf("  [%s] %-40s (%.2e)\n", ok ? " OK " : "FAIL", what, val);
  if (!ok) ++fails;
}

int main(int argc, char** argv) {
  const std::string dir = argc > 1 ? argv[1] : "config";
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> U(-1.5, 1.5);

  // ---------------- 1) 2축 비교 ----------------
  std::printf("1) 2축 범용 모델 vs 해석해\n");
  DhDynamics dh2(loadDhModel(dir + "/model_2dof.yaml"));
  Arm2DofPhysical p;  // model_2dof.yaml 과 같은 값
  p.d2 = 0.05; p.motor2_offset = 0.03; p.profile_length = 0.20; p.tip_mass = 0.0;
  Arm2DofDynamics an(p);
  double worst = 0;
  for (int t = 0; t < 50; ++t) {
    Eigen::VectorXd q(2), dq(2);
    q << U(rng), U(rng);
    dq << U(rng), U(rng);
    Eigen::MatrixXd M1, C1, M2, C2;
    Eigen::VectorXd G1, F1, G2, F2;
    dh2.compute(q, dq, M1, C1, G1, F1);
    an.compute(q, dq, M2, C2, G2, F2);
    worst = std::max({worst, (M1 - M2).cwiseAbs().maxCoeff(), (C1 - C2).cwiseAbs().maxCoeff(),
                      (G1 - G2).cwiseAbs().maxCoeff()});
  }
  check(worst < 1e-6, "M, C, G 일치 (최대 차이)", worst);

  // ---------------- 2), 3) 4축 성질 ----------------
  std::printf("2) 4축 모델 성질\n");
  DhDynamics dh4(loadDhModel(dir + "/model_4dof.yaml"));
  const int n = dh4.dof();
  double skew = 0, grav = 0;
  for (int t = 0; t < 50; ++t) {
    Eigen::VectorXd q(n), dq(n), x(n);
    for (int i = 0; i < n; ++i) { q(i) = U(rng); dq(i) = U(rng); x(i) = U(rng); }
    Eigen::MatrixXd M, C;
    Eigen::VectorXd G, F;
    dh4.compute(q, dq, M, C, G, F);
    // Ṁ = Σ ∂M/∂q_k q̇_k (수치 미분)
    const double h = 1e-6;
    const Eigen::MatrixXd Mdot = (dh4.massMatrix(q + h * dq) - dh4.massMatrix(q - h * dq)) / (2 * h);
    skew = std::max(skew, std::fabs(x.dot((Mdot - 2 * C) * x)));

    // V(q) = −Σ m gᵀ p_c   ->  G = ∂V/∂q
    auto V = [&](const Eigen::VectorXd& qq) {
      const auto T = dh4.forwardKinematics(qq);
      double v = 0;
      for (int i = 0; i < n; ++i) {
        const auto& b = dh4.model().joints[i].link;
        const Eigen::Vector3d pc = T[i + 1].block<3, 1>(0, 3) + T[i + 1].block<3, 3>(0, 0) * b.com;
        v -= b.mass * dh4.model().gravity.dot(pc);
      }
      return v;
    };
    for (int k = 0; k < n; ++k) {
      Eigen::VectorXd e = Eigen::VectorXd::Zero(n);
      e(k) = 1e-6;
      grav = std::max(grav, std::fabs((V(q + e) - V(q - e)) / 2e-6 - G(k)));
    }
  }
  check(skew < 1e-5, "x^T (Mdot - 2C) x = 0 (반대칭)", skew);
  check(grav < 1e-5, "G = dV/dq (중력 항)", grav);

  std::printf(fails ? "%d test(s) FAILED\n" : "all model tests passed\n", fails);
  return fails ? 1 : 0;
}
