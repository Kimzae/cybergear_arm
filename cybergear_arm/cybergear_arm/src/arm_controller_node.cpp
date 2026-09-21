// =============================================================================
// arm_controller_node.cpp
//   CyberGear 로봇팔 SMC 제어 ROS2 노드 (초안)
//
//   구조
//     ┌ ROS 스레드 (rclcpp::spin) ─────────────┐   ┌ 제어 스레드 (고정 주기) ────────────┐
//     │  /arm/reference 구독 -> ref_ 에 저장     │   │  1) poll()   : 피드백 읽기            │
//     │                                          │-->│  2) 안전 검사                         │
//     │  (publish 는 제어 스레드에서 직접 호출)  │   │  3) SMC 계산                          │
//     └──────────────────────────────────────────┘   │  4) sendTorque() : 모든 모터에 명령   │
//                                                     │  5) N 주기마다 상태 publish          │
//                                                     └──────────────────────────────────────┘
//
//   왜 ROS 타이머가 아니라 별도 스레드인가?
//     ROS 타이머는 콜백 처리 순서·실행기(executor) 부하에 따라 주기가 흔들린다.
//     1 kHz 제어에서는 clock_nanosleep(절대시각) 으로 직접 주기를 지키는 편이 훨씬 안정적이다.
//     (정석은 ros2_control 이며, 6축 확장 때 이 코드를 hardware_interface 로 옮기면 된다)
//
//   토픽
//     구독  /arm/reference   trajectory_msgs/JointTrajectoryPoint  (positions, velocities, accelerations)
//     발행  /arm/joint_states sensor_msgs/JointState
//           /arm/debug        std_msgs/Float64MultiArray  [t, q, dq, q_d, e, s, tau, period_ms, compute_ms]
// =============================================================================
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <atomic>
#include <cmath>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "cybergear_arm/arm_dynamics.hpp"
#include "cybergear_arm/cybergear_bus.hpp"
#include "cybergear_arm/sim_transport.hpp"
#include "cybergear_arm/smc_controller.hpp"

using namespace cybergear;
using namespace std::chrono_literals;

namespace {

// std::vector<double> -> Eigen::VectorXd
Eigen::VectorXd toEigen(const std::vector<double>& v) {
  return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<long>(v.size()));
}

// timespec 에 나노초 더하기
void addNs(timespec& t, long ns) {
  t.tv_nsec += ns;
  while (t.tv_nsec >= 1000000000L) { t.tv_nsec -= 1000000000L; ++t.tv_sec; }
}

double diffMs(const timespec& a, const timespec& b) {
  return (a.tv_sec - b.tv_sec) * 1e3 + (a.tv_nsec - b.tv_nsec) * 1e-6;
}

}  // namespace

class ArmControllerNode : public rclcpp::Node {
 public:
  ArmControllerNode() : Node("arm_controller") {
    // ------------------------------------------------------------------
    // 1) 파라미터 읽기  (config/arm_2dof.yaml 참고)
    // ------------------------------------------------------------------
    const auto transport = declare_parameter<std::string>("transport", "serial");  // serial|socketcan|sim
    const auto bus_names = declare_parameter<std::vector<std::string>>("buses", {"/dev/ttyUSB0"});
    const auto baud = declare_parameter<int>("baudrate", 921600);
    const auto host_id = declare_parameter<int>("host_id", 0xFD);

    const auto names = declare_parameter<std::vector<std::string>>("joint_names", {"joint1", "joint2"});
    const auto ids = declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2});
    const auto bus_idx = declare_parameter<std::vector<int64_t>>("bus_indices", {0, 0});
    const auto dirs = declare_parameter<std::vector<double>>("directions", {1.0, 1.0});
    const auto offs = declare_parameter<std::vector<double>>("offsets", {0.0, 0.0});
    const auto tlim = declare_parameter<std::vector<double>>("torque_limits", {2.0, 2.0});

    rate_hz_ = declare_parameter<double>("rate_hz", 500.0);
    publish_decimation_ = declare_parameter<int>("publish_decimation", 5);
    ref_timeout_ = declare_parameter<double>("reference_timeout", 0.5);
    vel_filter_hz_ = declare_parameter<double>("velocity_filter_hz", 0.0);  // 0 = 끔
    rt_priority_ = declare_parameter<int>("realtime_priority", 80);

    // 안전
    q_min_ = toEigen(declare_parameter<std::vector<double>>("joint_min", {-2.5, -1.8}));
    q_max_ = toEigen(declare_parameter<std::vector<double>>("joint_max", {2.5, 1.8}));
    max_vel_ = declare_parameter<double>("max_velocity", 6.0);
    fb_timeout_ = declare_parameter<double>("feedback_timeout", 0.05);
    safe_kd_ = declare_parameter<double>("safe_damping_kd", 1.0);

    // SMC 이득
    SmcGains gains;
    gains.lambda = toEigen(declare_parameter<std::vector<double>>("smc.lambda", {15.0, 15.0}));
    gains.k = toEigen(declare_parameter<std::vector<double>>("smc.k", {0.2, 0.4}));
    gains.phi = toEigen(declare_parameter<std::vector<double>>("smc.phi", {0.3, 0.3}));
    gains.kd = toEigen(declare_parameter<std::vector<double>>("smc.kd", {0.2, 0.2}));
    const bool use_model = declare_parameter<bool>("smc.use_model", true);

    // 동역학 모델 물리 파라미터
    Arm2DofPhysical p;
    p.d2 = declare_parameter("model.d2", p.d2);
    p.motor_mass = declare_parameter("model.motor_mass", p.motor_mass);
    p.motor_radius = declare_parameter("model.motor_radius", p.motor_radius);
    p.motor_length = declare_parameter("model.motor_length", p.motor_length);
    p.motor2_offset = declare_parameter("model.motor2_offset", p.motor2_offset);
    p.link1_extra_izz = declare_parameter("model.link1_extra_izz", p.link1_extra_izz);
    p.profile_length = declare_parameter("model.profile_length", p.profile_length);
    p.profile_start = declare_parameter("model.profile_start", p.profile_start);
    p.profile_lin_density = declare_parameter("model.profile_lin_density", p.profile_lin_density);
    p.profile_side = declare_parameter("model.profile_side", p.profile_side);
    p.bracket2_mass = declare_parameter("model.bracket2_mass", p.bracket2_mass);
    p.bracket2_pos = declare_parameter("model.bracket2_pos", p.bracket2_pos);
    p.tip_mass = declare_parameter("model.tip_mass", p.tip_mass);
    p.tip_pos = declare_parameter("model.tip_pos", p.tip_pos);
    p.rotor_reflected1 = declare_parameter("model.rotor_reflected1", p.rotor_reflected1);
    p.rotor_reflected2 = declare_parameter("model.rotor_reflected2", p.rotor_reflected2);
    p.viscous1 = declare_parameter("model.viscous1", p.viscous1);
    p.viscous2 = declare_parameter("model.viscous2", p.viscous2);
    p.coulomb1 = declare_parameter("model.coulomb1", p.coulomb1);
    p.coulomb2 = declare_parameter("model.coulomb2", p.coulomb2);

    // 시뮬레이터 전용
    const double sim_mass_scale = declare_parameter("sim.mass_scale", 1.0);
    const auto sim_q0 = declare_parameter<std::vector<double>>("sim.q0", {0.0, 0.3});

    // ------------------------------------------------------------------
    // 2) 관절 설정 만들기
    // ------------------------------------------------------------------
    n_ = names.size();
    if (ids.size() != n_ || bus_idx.size() != n_ || dirs.size() != n_ || offs.size() != n_ ||
        tlim.size() != n_) {
      throw std::runtime_error("joint_names / motor_ids / bus_indices / directions / offsets / "
                               "torque_limits 길이가 모두 같아야 함");
    }
    if (static_cast<size_t>(q_min_.size()) != n_ || static_cast<size_t>(q_max_.size()) != n_)
      throw std::runtime_error("joint_min / joint_max 길이가 관절 수와 다름");
    if (n_ != 2) throw std::runtime_error("현재 동역학 모델은 2축 전용 (Arm2DofDynamics)");

    std::vector<JointConfig> joints(n_);
    for (size_t i = 0; i < n_; ++i) {
      joints[i].name = names[i];
      joints[i].motor_id = static_cast<uint8_t>(ids[i]);
      joints[i].bus_index = static_cast<size_t>(bus_idx[i]);
      joints[i].direction = dirs[i];
      joints[i].offset = offs[i];
      joints[i].torque_limit = tlim[i];
    }
    joint_names_ = names;

    // ------------------------------------------------------------------
    // 3) 모델, 제어기
    // ------------------------------------------------------------------
    auto model = std::make_shared<Arm2DofDynamics>(p);
    const auto& L = model->lumped();
    RCLCPP_INFO(get_logger(), "모델: m2=%.3f kg, lc2=%.3f m, It2=%.2e, Izz1=%.2e",
                L.m2, L.lc2, L.It2, L.Izz1);
    smc_ = std::make_unique<SmcController>(use_model ? model : nullptr, gains);

    // ------------------------------------------------------------------
    // 4) 전송 계층(버스) 생성
    // ------------------------------------------------------------------
    std::vector<std::unique_ptr<CanTransport>> buses;
    if (transport == "serial") {
      for (const auto& b : bus_names) buses.push_back(makeSerialAtTransport(b, baud));
    } else if (transport == "socketcan") {
      for (const auto& b : bus_names) buses.push_back(makeSocketCanTransport(b));
    } else if (transport == "sim") {
      Arm2DofPhysical truth = p;  // 시뮬레이터는 "실제 로봇" 역할
      truth.profile_lin_density *= sim_mass_scale;
      truth.tip_mass *= sim_mass_scale;
      auto sim = std::make_unique<SimTransport>(std::make_shared<Arm2DofDynamics>(truth), joints,
                                                toEigen(sim_q0), host_id);
      sim->setRealtime(true);
      buses.push_back(std::move(sim));
      for (auto& j : joints) j.bus_index = 0;
    } else {
      throw std::runtime_error("transport 는 serial / socketcan / sim 중 하나");
    }
    for (const auto& b : buses) RCLCPP_INFO(get_logger(), "버스: %s", b->name().c_str());

    bus_ = std::make_unique<CyberGearBus>(std::move(buses), joints, host_id);

    // ------------------------------------------------------------------
    // 5) ROS 통신
    // ------------------------------------------------------------------
    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("arm/joint_states", 10);
    dbg_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("arm/debug", 10);
    ref_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        "arm/reference", 10, [this](trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
          onReference(*msg);
        });

    // ------------------------------------------------------------------
    // 6) 모터 시작 + 첫 피드백 대기
    // ------------------------------------------------------------------
    bus_->startAll();
    const auto t0 = std::chrono::steady_clock::now();
    while (true) {
      for (size_t i = 0; i < n_; ++i) bus_->sendDamping(i, safe_kd_);  // 명령을 줘야 응답이 온다
      std::this_thread::sleep_for(5ms);
      bus_->poll();
      bool all = true;
      for (size_t i = 0; i < n_; ++i) all &= bus_->state(i).valid;
      if (all) break;
      if (std::chrono::steady_clock::now() - t0 > 1s) {
        for (size_t i = 0; i < n_; ++i)
          if (!bus_->state(i).valid)
            RCLCPP_ERROR(get_logger(), "%s (모터 ID %d) 응답 없음", names[i].c_str(), (int)ids[i]);
        bus_->stopAll();
        throw std::runtime_error("모터 피드백 없음 — 전원/배선/ID 확인");
      }
    }

    // 시작 시 기준 = 현재 자세 (그 자리에서 버티기)
    ref_q_ = Eigen::VectorXd(n_);
    for (size_t i = 0; i < n_; ++i) ref_q_(i) = bus_->state(i).position;
    ref_dq_ = Eigen::VectorXd::Zero(n_);
    ref_ddq_ = Eigen::VectorXd::Zero(n_);
    ref_stamp_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "제어 시작: %.0f Hz, 관절 %zu 개", rate_hz_, n_);
    running_ = true;
    thread_ = std::thread(&ArmControllerNode::controlLoop, this);
  }

  ~ArmControllerNode() override {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    // ※ stopAll() 하면 모터가 풀려서 팔이 중력으로 떨어진다.
    //    먼저 댐핑으로 잠시 천천히 내려오게 한 뒤 정지.
    for (int k = 0; k < 100; ++k) {
      for (size_t i = 0; i < n_; ++i) bus_->sendDamping(i, safe_kd_);
      std::this_thread::sleep_for(5ms);
      bus_->poll();
    }
    bus_->stopAll();
  }

 private:
  // ---------------- 기준 궤적 수신 (ROS 스레드) ----------------
  void onReference(const trajectory_msgs::msg::JointTrajectoryPoint& m) {
    if (m.positions.size() != n_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "reference positions 길이 != %zu", n_);
      return;
    }
    std::lock_guard<std::mutex> lk(ref_mtx_);
    ref_q_ = toEigen(m.positions);
    ref_dq_ = m.velocities.size() == n_ ? toEigen(m.velocities) : Eigen::VectorXd::Zero(n_);
    ref_ddq_ = m.accelerations.size() == n_ ? toEigen(m.accelerations) : Eigen::VectorXd::Zero(n_);
    ref_stamp_ = std::chrono::steady_clock::now();
  }

  // ---------------- 실시간 우선순위 설정 ----------------
  void trySetRealtime() {
    sched_param sp{};
    sp.sched_priority = rt_priority_;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      RCLCPP_WARN(get_logger(),
                  "실시간 우선순위(SCHED_FIFO) 설정 실패 -> 일반 우선순위로 동작. "
                  "(ARCHITECTURE.md '실시간 설정' 참고)");
    } else {
      RCLCPP_INFO(get_logger(), "SCHED_FIFO 우선순위 %d 로 동작", rt_priority_);
    }
  }

  // ---------------- 제어 루프 (제어 스레드) ----------------
  void controlLoop() {
    trySetRealtime();
    const long period_ns = static_cast<long>(1e9 / rate_hz_);
    const double dt = 1.0 / rate_hz_;
    const double alpha = vel_filter_hz_ > 0 ? dt / (dt + 1.0 / (2 * M_PI * vel_filter_hz_)) : 1.0;

    Eigen::VectorXd q(n_), dq = Eigen::VectorXd::Zero(n_), qd(n_), dqd(n_), ddqd(n_);
    timespec next, now, prev, after;
    clock_gettime(CLOCK_MONOTONIC, &next);
    prev = next;
    const auto t_start = std::chrono::steady_clock::now();
    uint64_t cycle = 0;
    double worst_period = 0.0, worst_compute = 0.0;

    while (running_ && rclcpp::ok()) {
      // 다음 주기 시각까지 "절대 시간"으로 잠들기 -> 오차가 누적되지 않음
      addNs(next, period_ns);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
      clock_gettime(CLOCK_MONOTONIC, &now);
      const double period_ms = diffMs(now, prev);
      prev = now;

      // 1) 피드백 읽기 (직렬 어댑터는 보통 "지난 주기" 명령에 대한 응답이 도착)
      bus_->poll();
      for (size_t i = 0; i < n_; ++i) {
        q(i) = bus_->state(i).position;
        dq(i) = alpha * bus_->state(i).velocity + (1 - alpha) * dq(i);  // 1차 저역통과
      }

      // 2) 안전 검사 -> 문제 생기면 SAFE 상태로 고정(latch)
      if (!safe_) checkSafety(q, dq);
      if (safe_) {
        for (size_t i = 0; i < n_; ++i) bus_->sendDamping(i, safe_kd_);
        continue;
      }

      // 3) 기준 궤적 가져오기 (짧게 잠금)
      {
        std::lock_guard<std::mutex> lk(ref_mtx_);
        qd = ref_q_;
        dqd = ref_dq_;
        ddqd = ref_ddq_;
        const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - ref_stamp_).count();
        if (age > ref_timeout_) { dqd.setZero(); ddqd.setZero(); }  // 끊기면 마지막 위치에서 정지
      }

      // 4) SMC 계산 + 명령 전송
      const auto out = smc_->compute(q, dq, qd, dqd, ddqd);
      for (size_t i = 0; i < n_; ++i) bus_->sendTorque(i, out.tau(i));

      clock_gettime(CLOCK_MONOTONIC, &after);
      const double compute_ms = diffMs(after, now);
      if (cycle > 10) {
        worst_period = std::max(worst_period, period_ms);
        worst_compute = std::max(worst_compute, compute_ms);
      }

      // 5) 상태 발행 (매 주기 하면 부하가 크므로 N 주기마다)
      if (++cycle % publish_decimation_ == 0) {
        const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
        publish(t, q, dq, qd, out, period_ms, compute_ms);
      }
      if (cycle % static_cast<uint64_t>(rate_hz_ * 5) == 0) {
        RCLCPP_INFO(get_logger(), "최근 5초: 최대 주기 %.3f ms (목표 %.3f), 최대 계산 %.3f ms",
                    worst_period, 1e3 / rate_hz_, worst_compute);
        worst_period = worst_compute = 0.0;
      }
    }
  }

  void checkSafety(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
    std::string why;
    for (size_t i = 0; i < n_ && why.empty(); ++i) {
      const auto& s = bus_->state(i);
      if (s.fault) why = joint_names_[i] + " 모터 에러 코드 " + std::to_string(s.fault);
      else if (std::fabs(dq(i)) > max_vel_) why = joint_names_[i] + " 속도 초과";
      else if (q(i) < q_min_(i) || q(i) > q_max_(i)) why = joint_names_[i] + " 관절 한계 초과";
    }
    if (why.empty() && bus_->oldestFeedbackAge() > fb_timeout_) why = "피드백 끊김";
    if (!why.empty()) {
      safe_ = true;
      RCLCPP_ERROR(get_logger(), "SAFE 모드 진입 (%s) -> 댐핑만 유지. 노드를 재시작하세요.", why.c_str());
    }
  }

  void publish(double t, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
               const Eigen::VectorXd& qd, const SmcOutput& out, double period_ms, double compute_ms) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = joint_names_;
    for (size_t i = 0; i < n_; ++i) {
      js.position.push_back(q(i));
      js.velocity.push_back(dq(i));
      js.effort.push_back(bus_->state(i).torque);
    }
    js_pub_->publish(js);

    std_msgs::msg::Float64MultiArray d;
    d.data.reserve(1 + 6 * n_ + 2);
    d.data.push_back(t);
    for (const Eigen::VectorXd* v : {&q, &dq, &qd, &out.e, &out.s, &out.tau})
      for (long i = 0; i < v->size(); ++i) d.data.push_back((*v)(i));
    d.data.push_back(period_ms);
    d.data.push_back(compute_ms);
    dbg_pub_->publish(d);
  }

  // ---- 멤버 ----
  size_t n_ = 0;
  std::vector<std::string> joint_names_;
  std::unique_ptr<CyberGearBus> bus_;
  std::unique_ptr<SmcController> smc_;

  double rate_hz_ = 500.0, ref_timeout_ = 0.5, vel_filter_hz_ = 0.0;
  int publish_decimation_ = 5, rt_priority_ = 80;
  Eigen::VectorXd q_min_, q_max_;
  double max_vel_ = 6.0, fb_timeout_ = 0.05, safe_kd_ = 1.0;

  std::mutex ref_mtx_;
  Eigen::VectorXd ref_q_, ref_dq_, ref_ddq_;
  std::chrono::steady_clock::time_point ref_stamp_;

  std::atomic<bool> running_{false};
  bool safe_ = false;
  std::thread thread_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dbg_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr ref_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ArmControllerNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[arm_controller] 오류: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
