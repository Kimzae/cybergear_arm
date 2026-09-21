// =============================================================================
// sim_transport.cpp
// =============================================================================
#include "cybergear_arm/sim_transport.hpp"

#include <algorithm>

namespace cybergear {

SimTransport::SimTransport(std::shared_ptr<const ArmDynamics> true_model,
                           std::vector<JointConfig> joints, Eigen::VectorXd q0, uint8_t host_id)
    : model_(std::move(true_model)), joints_(std::move(joints)), host_id_(host_id),
      q_(std::move(q0)) {
  dq_ = Eigen::VectorXd::Zero(q_.size());
  cmd_.resize(joints_.size());
}

bool SimTransport::send(const CanFrame& f) {
  const uint8_t mid = f.id & 0xFF;
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].motor_id != mid) continue;
    auto& c = cmd_[i];
    switch (commTypeOf(f.id)) {
      case kEnable: c.enabled = true; c.replied = false; break;
      case kStop: c.enabled = false; c.t_ff = c.kp = c.kd = 0; c.replied = false; break;
      case kMotion: {
        auto u16 = [&](int k) { return uint16_t((f.data[k] << 8) | f.data[k + 1]); };
        c.t_ff = uint16ToFloat((f.id >> 8) & 0xFFFF, kTorMin, kTorMax);
        c.p = uint16ToFloat(u16(0), kPosMin, kPosMax);
        c.v = uint16ToFloat(u16(2), kVelMin, kVelMax);
        c.kp = uint16ToFloat(u16(4), kKpMin, kKpMax);
        c.kd = uint16ToFloat(u16(6), kKdMin, kKdMax);
        c.replied = false;
        break;
      }
      default: break;
    }
  }
  return true;
}

void SimTransport::step(double dt) {
  const int n = static_cast<int>(q_.size());
  Eigen::MatrixXd M, C;
  Eigen::VectorXd G, F, tau(n);
  // 0.2 ms 이하로 쪼개서 적분 (안정성)
  const int sub = std::max(1, static_cast<int>(dt / 2e-4 + 0.5));
  const double h = dt / sub;
  for (int k = 0; k < sub; ++k) {
    for (int i = 0; i < n; ++i) {
      const auto& j = joints_[i];
      const auto& c = cmd_[i];
      // 모터 좌표로 변환해서 모터 내부 제어식 계산
      const double pm = (q_(i) - j.offset) * j.direction;
      const double vm = dq_(i) * j.direction;
      double tm = c.enabled ? c.kp * (c.p - pm) + c.kd * (c.v - vm) + c.t_ff : 0.0;
      tm = std::clamp(tm, kTorMin, kTorMax);
      tau(i) = j.direction * tm;
    }
    model_->compute(q_, dq_, M, C, G, F);
    const Eigen::VectorXd ddq = M.ldlt().solve(tau - C * dq_ - G - F);
    dq_ += h * ddq;  // semi-implicit Euler
    q_ += h * dq_;
  }
}

void SimTransport::receive(std::vector<CanFrame>& out) {
  if (realtime_) {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_).count();
    last_ = now;
    if (dt > 0 && dt < 0.1) step(dt);
  }
  // 명령을 받은 모터마다 피드백 1개 (실제 CyberGear 동작과 같음)
  for (size_t i = 0; i < joints_.size(); ++i) {
    auto& c = cmd_[i];
    if (c.replied) continue;
    c.replied = true;
    const auto& j = joints_[i];
    CanFrame f;
    f.id = (uint32_t(kFeedback) << 24) | (uint32_t(c.enabled ? 2 : 0) << 22) |
           (uint32_t(j.motor_id) << 8) | host_id_;
    const double pm = (q_(i) - j.offset) * j.direction;
    const double vm = dq_(i) * j.direction;
    auto put = [&](int k, uint16_t v) { f.data[k] = v >> 8; f.data[k + 1] = v & 0xFF; };
    put(0, floatToUint16(pm, kPosMin, kPosMax));
    put(2, floatToUint16(vm, kVelMin, kVelMax));
    put(4, floatToUint16(c.t_ff, kTorMin, kTorMax));
    put(6, 250);  // 25.0 °C
    out.push_back(f);
  }
}

}  // namespace cybergear
