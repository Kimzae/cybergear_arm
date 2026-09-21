// =============================================================================
// smc_sim_test.cpp
//   ROS 없이 "제어기 + 버스 + 시뮬레이터"를 한 번에 돌려보는 오프라인 테스트.
//   - 제어기 모델: 공칭(nominal) 파라미터
//   - 시뮬레이터 : 질량 +30%, 길이 오차 등 "틀린" 파라미터 -> SMC 강인성 확인
//   실행: ros2 run cybergear_arm smc_sim_test
// =============================================================================
#include <cmath>
#include <cstdio>
#include <memory>

#include "cybergear_arm/arm_dynamics.hpp"
#include "cybergear_arm/cybergear_bus.hpp"
#include "cybergear_arm/sim_transport.hpp"
#include "cybergear_arm/smc_controller.hpp"

using namespace cybergear;

int main() {
  const double dt = 0.001;  // 1 kHz
  const double T = 6.0;

  // ---- 공칭 모델 (제어기가 믿는 값) ----
  Arm2DofPhysical nominal;
  nominal.tip_mass = 0.2;
  nominal.tip_pos = 0.20;

  // ---- 실제 로봇 (일부러 틀리게) ----
  Arm2DofPhysical truth = nominal;
  truth.profile_lin_density *= 1.3;  // 프로파일 30% 더 무거움
  truth.tip_mass *= 1.3;
  truth.tip_pos += 0.02;             // 끝단 2cm 더 멀리
  truth.viscous1 = truth.viscous2 = 0.02;
  truth.coulomb1 = truth.coulomb2 = 0.03;
  truth.rotor_reflected1 = truth.rotor_reflected2 = 5e-4;

  auto model = std::make_shared<Arm2DofDynamics>(nominal);
  auto true_model = std::make_shared<Arm2DofDynamics>(truth);

  std::vector<JointConfig> joints(2);
  joints[0] = {"joint1", 1, 0, 1.0, 0.0, 6.0};
  joints[1] = {"joint2", 2, 0, -1.0, 0.0, 6.0};  // 방향 반대로 달린 경우도 시험

  Eigen::VectorXd q0(2);
  q0 << 0.0, 0.3;
  auto sim_owner = std::make_unique<SimTransport>(true_model, joints, q0);
  SimTransport* sim = sim_owner.get();
  std::vector<std::unique_ptr<CanTransport>> buses;
  buses.push_back(std::move(sim_owner));
  CyberGearBus bus(std::move(buses), joints);
  bus.startAll();

  SmcGains g;
  g.lambda = Eigen::Vector2d(20, 20);
  g.k = Eigen::Vector2d(0.3, 0.6);
  g.phi = Eigen::Vector2d(0.2, 0.2);
  g.kd = Eigen::Vector2d(0.3, 0.3);
  // 지연 없는 이상적 조건이므로 HOST 모드로 (모델 기반 SMC 자체의 강인성 확인용)
  SmcController smc(model, g, InnerLoop::kHost, dt);

  Eigen::VectorXd q(2), dq(2), qd(2), dqd(2), ddqd(2);
  double max_err_late = 0.0;
  for (int k = 0; k < static_cast<int>(T / dt); ++k) {
    const double t = k * dt;
    // 기준 궤적: 두 관절 모두 사인파
    const double w = 2 * M_PI * 0.5;
    qd << 0.5 * std::sin(w * t), 0.3 + 0.4 * std::sin(w * t);
    dqd << 0.5 * w * std::cos(w * t), 0.4 * w * std::cos(w * t);
    ddqd << -0.5 * w * w * std::sin(w * t), -0.4 * w * w * std::sin(w * t);

    bus.poll();
    for (int i = 0; i < 2; ++i) { q(i) = bus.state(i).position; dq(i) = bus.state(i).velocity; }
    auto out = smc.compute(q, dq, qd, dqd, ddqd);
    for (int i = 0; i < 2; ++i) bus.sendTorque(i, out.tau(i));
    sim->step(dt);

    if (t > 2.0) max_err_late = std::max(max_err_late, out.e.cwiseAbs().maxCoeff());
    if (k % 500 == 0)
      std::printf("t=%.1f  e=[%+.4f %+.4f] rad  tau=[%+.3f %+.3f] Nm\n", t, out.e(0), out.e(1),
                  out.tau(0), out.tau(1));
  }
  std::printf("max |e| after 2 s = %.4f rad (%.2f deg)\n", max_err_late,
              max_err_late * 180 / M_PI);
  return max_err_late < 0.05 ? 0 : 1;
}
