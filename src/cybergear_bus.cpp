// =============================================================================
// cybergear_bus.cpp  —  cybergear_bus.hpp 구현
// =============================================================================
#include "cybergear_arm/cybergear_bus.hpp"

#include <algorithm>
#include <stdexcept>
#include <thread>

namespace cybergear {

using namespace std::chrono_literals;

CyberGearBus::CyberGearBus(std::vector<std::unique_ptr<CanTransport>> buses,
                           std::vector<JointConfig> joints, uint8_t host_id)
    : buses_(std::move(buses)), joints_(std::move(joints)), host_id_(host_id) {
  for (const auto& j : joints_) {
    if (j.bus_index >= buses_.size()) {
      throw std::runtime_error("joint '" + j.name + "' 의 bus_index 가 버스 개수를 넘음");
    }
  }
  states_.resize(joints_.size());
  rx_buf_.reserve(64);
}

void CyberGearBus::sendRaw(size_t i, const CanFrame& f) {
  buses_[joints_[i].bus_index]->send(f);
}

void CyberGearBus::startAll() {
  for (size_t i = 0; i < size(); ++i) sendRaw(i, makeStop(joints_[i].motor_id, host_id_, true));
  std::this_thread::sleep_for(20ms);
  for (size_t i = 0; i < size(); ++i)
    sendRaw(i, makeWriteParamU8(joints_[i].motor_id, host_id_, kParamRunMode, 0));
  std::this_thread::sleep_for(20ms);
  // 모터 자체 토크 제한 (모터 내부 PD 토크까지 포함해 전체를 제한)
  for (size_t i = 0; i < size(); ++i)
    sendRaw(i, makeWriteParamFloat(joints_[i].motor_id, host_id_, kParamLimitTorque,
                                   static_cast<float>(joints_[i].torque_limit)));
  std::this_thread::sleep_for(20ms);
  for (size_t i = 0; i < size(); ++i) sendRaw(i, makeEnable(joints_[i].motor_id, host_id_));
  std::this_thread::sleep_for(20ms);
  poll();
}

void CyberGearBus::stopAll() {
  for (size_t i = 0; i < size(); ++i) sendTorque(i, 0.0);
  std::this_thread::sleep_for(10ms);
  for (size_t i = 0; i < size(); ++i) sendRaw(i, makeStop(joints_[i].motor_id, host_id_, false));
}

void CyberGearBus::sendTorque(size_t i, double tau_joint, double kd) {
  const auto& j = joints_[i];
  const double tau = std::clamp(tau_joint, -j.torque_limit, j.torque_limit);
  // 관절 좌표 -> 모터 좌표 (방향만 바뀜. 오프셋은 토크와 무관)
  sendRaw(i, makeMotion(j.motor_id, j.direction * tau, 0.0, 0.0, 0.0, kd));
}

void CyberGearBus::sendMotion(size_t i, double tau_ff, double p_ref, double v_ref, double kp,
                              double kd) {
  const auto& j = joints_[i];
  const double tau = std::clamp(tau_ff, -j.torque_limit, j.torque_limit);
  // 관절 -> 모터 좌표:  q_motor = (q_joint − offset) * dir   (dir = ±1 이므로 1/dir = dir)
  const double p_m = (p_ref - j.offset) * j.direction;
  const double v_m = v_ref * j.direction;
  // kp, kd 는 부호 반전에 대해 불변 (양쪽 오차가 같이 뒤집힘)
  sendRaw(i, makeMotion(j.motor_id, j.direction * tau, p_m, v_m, kp, kd));
}

void CyberGearBus::sendDamping(size_t i, double kd) {
  sendRaw(i, makeMotion(joints_[i].motor_id, 0.0, 0.0, 0.0, 0.0, kd));
}

int CyberGearBus::poll() {
  int updated = 0;
  const auto now = std::chrono::steady_clock::now();
  for (size_t b = 0; b < buses_.size(); ++b) {
    rx_buf_.clear();
    buses_[b]->receive(rx_buf_);
    for (const auto& f : rx_buf_) {
      auto fb = parseFeedback(f);
      if (!fb) continue;
      // 어느 관절의 모터인지 찾기: 같은 버스 + 같은 모터 ID
      // (관절 수가 적으므로 선형 탐색으로 충분)
      for (size_t i = 0; i < size(); ++i) {
        if (joints_[i].bus_index != b || joints_[i].motor_id != fb->motor_id) continue;
        const auto& j = joints_[i];
        auto& s = states_[i];
        s.position = j.direction * fb->position + j.offset;  // 모터 -> 관절 좌표
        s.velocity = j.direction * fb->velocity;
        s.torque = j.direction * fb->torque;
        s.temperature = fb->temperature;
        s.fault = fb->fault;
        s.mode = fb->mode;
        s.valid = true;
        s.stamp = now;
        ++updated;
        break;
      }
    }
  }
  return updated;
}

double CyberGearBus::oldestFeedbackAge() const {
  const auto now = std::chrono::steady_clock::now();
  double worst = 0.0;
  for (const auto& s : states_) {
    if (!s.valid) return 1e9;
    worst = std::max(worst, std::chrono::duration<double>(now - s.stamp).count());
  }
  return worst;
}

}  // namespace cybergear
