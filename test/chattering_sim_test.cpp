// =============================================================================
// chattering_sim_test.cpp
//   "왜 떨리는가 / 어떻게 고치는가" 를 시뮬레이터로 재현하는 실험.
//
//   조건: 500 Hz 제어, 피드백 지연 4 ms (CH340 USB 흉내), 실제 로봇 질량 +30%
//   비교:
//     A) HOST  모드 + 이전 기본 이득 (λ15, K .2/.4, φ .3, Kd .2)   ← 영상 상황
//     B) HOST  모드 + φ 크게 / Kd 작게
//     C) MOTOR 모드 (선형 피드백을 모터 내부 PD 로)               ← 권장
//     D) MOTOR 모드 + 조건부 적분
//   지표: 토크 명령의 주기간 변화량 RMS (채터링 크기), 위치 오차
//   실행: ros2 run cybergear_arm chattering_sim_test $(ros2 pkg prefix cybergear_arm)/share/cybergear_arm/config
// =============================================================================
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "cybergear_arm/cybergear_bus.hpp"
#include "cybergear_arm/dh_dynamics.hpp"
#include "cybergear_arm/sim_transport.hpp"
#include "cybergear_arm/smc_controller.hpp"

using namespace cybergear;

struct Result { double chatter, max_err, rms_err; bool safe; };

static Result run(const std::string& dir, InnerLoop mode, SmcGains g, double delay) {
  const double dt = 0.002, T = 6.0;
  auto model = std::make_shared<DhDynamics>(loadDhModel(dir + "/model_2dof.yaml"));
  auto truth = std::make_shared<DhDynamics>(loadDhModel(dir + "/model_2dof.yaml", 1.3));

  std::vector<JointConfig> joints(2);
  joints[0] = {"joint1", 1, 0, 1.0, 0.0, 4.0};
  joints[1] = {"joint2", 2, 0, 1.0, 0.0, 4.0};
  Eigen::VectorXd q0(2);
  q0 << 0.0, 0.3;
  auto sim_owner = std::make_unique<SimTransport>(truth, joints, q0);
  SimTransport* sim = sim_owner.get();
  sim->setFeedbackDelay(delay);
  std::vector<std::unique_ptr<CanTransport>> buses;
  buses.push_back(std::move(sim_owner));
  CyberGearBus bus(std::move(buses), joints);
  bus.startAll();
  bus.poll();

  SmcController smc(model, g, mode, dt);
  Eigen::VectorXd q(2), dq(2), qd(2), dqd(2), ddqd(2), prev_tau = Eigen::VectorXd::Zero(2);
  double chat = 0, maxe = 0, rmse = 0;
  int cnt = 0;
  for (int k = 0; k < static_cast<int>(T / dt); ++k) {
    const double t = k * dt, w = 2 * M_PI * 0.3;
    qd << 0.3 * std::sin(w * t), 0.3 + 0.3 * std::sin(w * t);
    dqd << 0.3 * w * std::cos(w * t), 0.3 * w * std::cos(w * t);
    ddqd << -0.3 * w * w * std::sin(w * t), -0.3 * w * w * std::sin(w * t);

    bus.poll();
    for (int i = 0; i < 2; ++i) { q(i) = bus.state(i).position; dq(i) = bus.state(i).velocity; }
    const auto out = smc.compute(q, dq, qd, dqd, ddqd);
    for (int i = 0; i < 2; ++i) {
      if (mode == InnerLoop::kMotor)
        bus.sendMotion(i, out.tau(i), out.p_ref(i), out.v_ref(i), out.kp(i), out.kd(i));
      else
        bus.sendTorque(i, out.tau(i));
    }
    sim->step(dt);

    // 실제 모터가 낸 총 토크 (모터 내부 PD 포함) 의 주기간 변화 = 채터링 크기
    const Eigen::VectorXd tau_total = sim->appliedTorque();
    if (t > 2.0) {
      chat += (tau_total - prev_tau).squaredNorm();
      const Eigen::VectorXd e_true = sim->q() - qd;
      maxe = std::max(maxe, e_true.cwiseAbs().maxCoeff());
      rmse += e_true.squaredNorm();
      ++cnt;
    }
    prev_tau = tau_total;
    if (!std::isfinite(sim->q().norm()) || sim->q().cwiseAbs().maxCoeff() > 10) return {1e9, 1e9, 1e9, true};
  }
  return {std::sqrt(chat / cnt), maxe, std::sqrt(rmse / cnt), false};
}

static SmcGains gains(double lam, double k1, double k2, double phi, double kd, double ki) {
  SmcGains g;
  g.lambda = Eigen::Vector2d(lam, lam);
  g.k = Eigen::Vector2d(k1, k2);
  g.phi = Eigen::Vector2d(phi, phi);
  g.kd = Eigen::Vector2d(kd, kd);
  g.ki = Eigen::Vector2d(ki, ki);
  g.i_max = Eigen::Vector2d(0.3, 0.3);
  return g;
}

int main(int argc, char** argv) {
  const std::string dir = argc > 1 ? argv[1] : "config";
  const double delay = 0.004;
  struct Case { const char* name; InnerLoop mode; SmcGains g; };
  const Case cases[] = {
      {"A) HOST  이전 이득 (영상 상황)", InnerLoop::kHost, gains(15, 0.2, 0.4, 0.3, 0.2, 0)},
      {"B) HOST  phi 크게, Kd 작게", InnerLoop::kHost, gains(10, 0.2, 0.3, 1.5, 0.02, 0)},
      {"C) MOTOR 이전 이득 그대로", InnerLoop::kMotor, gains(15, 0.2, 0.4, 0.3, 0.2, 0)},
      {"D) MOTOR 이전 이득 + 조건부 적분", InnerLoop::kMotor, gains(15, 0.2, 0.4, 0.3, 0.2, 3.0)},
  };
  std::printf("피드백 지연 %.1f ms, 500 Hz, 실제 질량 +30%%\n", delay * 1e3);
  std::printf("%-34s %14s %12s %12s\n", "경우", "채터링[Nm]", "최대오차[deg]", "RMS오차[deg]");
  Result r[4];
  for (int i = 0; i < 4; ++i) {
    r[i] = run(dir, cases[i].mode, cases[i].g, delay);
    if (r[i].safe) std::printf("%-34s  발산!\n", cases[i].name);
    else std::printf("%-34s %14.4f %12.3f %12.3f\n", cases[i].name, r[i].chatter,
                     r[i].max_err * 180 / M_PI, r[i].rms_err * 180 / M_PI);
  }
  return 0;
}
