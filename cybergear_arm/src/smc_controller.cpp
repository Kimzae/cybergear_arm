// =============================================================================
// smc_controller.cpp
// =============================================================================
#include "cybergear_arm/smc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "cybergear_arm/cybergear_protocol.hpp"

namespace cybergear {

SmcController::SmcController(std::shared_ptr<const ArmDynamics> model, SmcGains gains,
                             InnerLoop mode, double dt)
    : model_(std::move(model)), gains_(std::move(gains)), mode_(mode), dt_(dt) {
  const auto n = gains_.lambda.size();
  if (gains_.ki.size() == 0) gains_.ki = Eigen::VectorXd::Zero(n);
  if (gains_.i_max.size() == 0) gains_.i_max = Eigen::VectorXd::Constant(n, 0.5);
  if (gains_.k.size() != n || gains_.phi.size() != n || gains_.kd.size() != n ||
      gains_.ki.size() != n || gains_.i_max.size() != n)
    throw std::runtime_error("SMC 이득 벡터 길이가 서로 다름");
  if (model_ && model_->dof() != n) throw std::runtime_error("모델 자유도와 이득 길이가 다름");
  integ_ = Eigen::VectorXd::Zero(n);

  // 모터 내부 PD 이득:  kd_int = Kd + K/φ,  kp_int = Λ kd_int
  kd_int_ = gains_.kd + gains_.k.cwiseQuotient(gains_.phi);
  kp_int_ = gains_.lambda.cwiseProduct(kd_int_);
  for (int i = 0; i < n; ++i) {
    kd_int_(i) = std::clamp(kd_int_(i), kKdMin, kKdMax);
    kp_int_(i) = std::clamp(kp_int_(i), kKpMin, kKpMax);
  }
}

SmcOutput SmcController::compute(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                 const Eigen::VectorXd& q_d, const Eigen::VectorXd& dq_d,
                                 const Eigen::VectorXd& ddq_d) {
  const auto& g = gains_;
  const int n = static_cast<int>(q.size());
  SmcOutput out;
  out.e = q - q_d;
  const Eigen::VectorXd de = dq - dq_d;
  out.s = de + g.lambda.cwiseProduct(out.e);

  // ---- 1) 모델 기반 항 ----
  if (model_) {
    if (mode_ == InnerLoop::kHost) {
      const Eigen::VectorXd dq_r = dq_d - g.lambda.cwiseProduct(out.e);
      const Eigen::VectorXd ddq_r = ddq_d - g.lambda.cwiseProduct(de);
      model_->compute(q, dq, M_, C_, G_, F_);
      out.tau_model = M_ * ddq_r + C_ * dq_r + G_ + F_;
    } else {
      model_->compute(q, dq_d, M_, C_, G_, F_);  // 속도 자리에 목표 속도
      out.tau_model = M_ * ddq_d + C_ * dq_d + G_ + F_;
    }
  } else {
    out.tau_model = Eigen::VectorXd::Zero(n);
  }

  // ---- 2) 조건부 적분: 경계층 안에서만 ∫s ----
  for (int i = 0; i < n; ++i) {
    if (g.ki(i) <= 0) continue;
    if (std::fabs(out.s(i)) < g.phi(i)) integ_(i) += out.s(i) * dt_;
    const double lim = g.i_max(i) / g.ki(i);
    integ_(i) = std::clamp(integ_(i), -lim, lim);
  }
  const Eigen::VectorXd tau_i = g.ki.cwiseProduct(integ_);

  // ---- 3) 스위칭 + 선형 항 ----
  Eigen::VectorXd sat(n);
  for (int i = 0; i < n; ++i) sat(i) = std::clamp(out.s(i) / g.phi(i), -1.0, 1.0);

  if (mode_ == InnerLoop::kHost) {
    out.tau = out.tau_model - g.k.cwiseProduct(sat) - g.kd.cwiseProduct(out.s) - tau_i;
    out.p_ref = Eigen::VectorXd::Zero(n);
    out.v_ref = Eigen::VectorXd::Zero(n);
    out.kp = Eigen::VectorXd::Zero(n);
    out.kd = Eigen::VectorXd::Zero(n);
  } else {
    // 선형 부분 −(Kd + K/φ)s 는 모터 내부 PD 가 지연 없이 담당.
    // PC 는 느린 부분(모델 피드포워드 + 적분)만 보낸다.
    out.tau = out.tau_model - tau_i;
    out.p_ref = q_d;
    out.v_ref = dq_d;
    out.kp = kp_int_;
    out.kd = kd_int_;
  }
  return out;
}

}  // namespace cybergear
