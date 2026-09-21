// =============================================================================
// smc_controller.cpp
// =============================================================================
#include "cybergear_arm/smc_controller.hpp"

#include <stdexcept>

namespace cybergear {

SmcController::SmcController(std::shared_ptr<const ArmDynamics> model, SmcGains gains)
    : model_(std::move(model)), gains_(std::move(gains)) {
  const auto n = gains_.lambda.size();
  if (gains_.k.size() != n || gains_.phi.size() != n || gains_.kd.size() != n)
    throw std::runtime_error("SMC 이득 벡터 길이가 서로 다름");
  if (model_ && model_->dof() != n) throw std::runtime_error("모델 자유도와 이득 길이가 다름");
}

SmcOutput SmcController::compute(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                 const Eigen::VectorXd& q_d, const Eigen::VectorXd& dq_d,
                                 const Eigen::VectorXd& ddq_d) {
  const auto& g = gains_;
  SmcOutput out;
  out.e = q - q_d;
  const Eigen::VectorXd de = dq - dq_d;

  const Eigen::VectorXd dq_r = dq_d - g.lambda.cwiseProduct(out.e);
  const Eigen::VectorXd ddq_r = ddq_d - g.lambda.cwiseProduct(de);
  out.s = dq - dq_r;  // = de + Λ e

  // 1) 모델 기반 항
  if (model_) {
    model_->compute(q, dq, M_, C_, G_, F_);
    out.tau_model = M_ * ddq_r + C_ * dq_r + G_ + F_;
  } else {
    out.tau_model = Eigen::VectorXd::Zero(q.size());
  }

  // 2) 스위칭 항 (경계층 sat) + 3) 선형 댐핑
  Eigen::VectorXd sat(q.size());
  for (int i = 0; i < q.size(); ++i) {
    const double x = out.s(i) / g.phi(i);
    sat(i) = x > 1.0 ? 1.0 : (x < -1.0 ? -1.0 : x);
  }
  out.tau = out.tau_model - g.k.cwiseProduct(sat) - g.kd.cwiseProduct(out.s);
  return out;
}

}  // namespace cybergear
