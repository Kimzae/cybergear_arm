// =============================================================================
// dh_dynamics.cpp
// =============================================================================
#include "cybergear_arm/dh_dynamics.hpp"

#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace cybergear {

namespace {

constexpr double kDeg = M_PI / 180.0;

// 벡터 r 만큼 떨어진 점에 대한 평행축 정리 항: m (|r|² I − r rᵀ)
Eigen::Matrix3d parallelAxis(double m, const Eigen::Vector3d& r) {
  return m * (r.squaredNorm() * Eigen::Matrix3d::Identity() - r * r.transpose());
}

Eigen::Matrix4d dhTransform(double theta, double d, double a, double alpha) {
  const double ct = std::cos(theta), st = std::sin(theta);
  const double ca = std::cos(alpha), sa = std::sin(alpha);
  Eigen::Matrix4d T;
  T << ct, -st * ca, st * sa, a * ct,
       st, ct * ca, -ct * sa, a * st,
       0, sa, ca, d,
       0, 0, 0, 1;
  return T;
}

int parseAxis(const std::string& s, int* sign) {
  *sign = 1;
  std::string a = s;
  if (!a.empty() && (a[0] == '-' || a[0] == '+')) {
    if (a[0] == '-') *sign = -1;
    a = a.substr(1);
  }
  if (a == "x") return 0;
  if (a == "y") return 1;
  if (a == "z") return 2;
  throw std::runtime_error("axis 는 x, y, z, -x, -y, -z 중 하나: " + s);
}

Eigen::Vector3d vec3(const YAML::Node& n) {
  if (!n || !n.IsSequence() || n.size() != 3) throw std::runtime_error("3개짜리 배열이 필요함");
  return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}

}  // namespace

// ---------------------------------------------------------------------------
void RigidBody::add(const RigidBody& o) {
  const double mt = mass + o.mass;
  if (mt <= 0) return;
  const Eigen::Vector3d c = (mass * com + o.mass * o.com) / mt;  // 새 무게중심
  inertia = inertia + parallelAxis(mass, com - c) + o.inertia + parallelAxis(o.mass, o.com - c);
  mass = mt;
  com = c;
}

RigidBody makePointMass(double m, const Eigen::Vector3d& pos) {
  RigidBody b;
  b.mass = m;
  b.com = pos;
  return b;
}

RigidBody makeCylinder(double m, double r, double h, const Eigen::Vector3d& center, int axis) {
  RigidBody b;
  b.mass = m;
  b.com = center;
  const double Iax = 0.5 * m * r * r;                  // 축 방향
  const double Itr = m * (3 * r * r + h * h) / 12.0;   // 옆 방향
  b.inertia = Eigen::Vector3d::Constant(Itr).asDiagonal();
  b.inertia(axis, axis) = Iax;
  return b;
}

RigidBody makeSquareRod(double rho, double a, double L, const Eigen::Vector3d& start, int axis, int sign) {
  RigidBody b;
  b.mass = rho * L;
  Eigen::Vector3d dir = Eigen::Vector3d::Zero();
  dir(axis) = sign;
  b.com = start + 0.5 * L * dir;
  const double Ilong = b.mass * a * a / 6.0;             // 길이 방향
  const double Itr = b.mass * (L * L + a * a) / 12.0;    // 옆 방향
  b.inertia = Eigen::Vector3d::Constant(Itr).asDiagonal();
  b.inertia(axis, axis) = Ilong;
  return b;
}

// ---------------------------------------------------------------------------
DhModel loadDhModel(const std::string& path, double mass_scale) {
  YAML::Node root = YAML::LoadFile(path);
  DhModel model;
  if (root["gravity"]) model.gravity = vec3(root["gravity"]);

  for (const auto& jn : root["joints"]) {
    DhJoint j;
    j.name = jn["name"].as<std::string>();
    const auto dh = jn["dh"];
    j.dh.theta_offset = dh["theta_offset_deg"].as<double>(0.0) * kDeg;
    j.dh.d = dh["d"].as<double>(0.0);
    j.dh.a = dh["a"].as<double>(0.0);
    j.dh.alpha = dh["alpha_deg"].as<double>(0.0) * kDeg;
    j.armature = jn["armature"].as<double>(0.0);
    j.viscous = jn["viscous"].as<double>(0.0);
    j.coulomb = jn["coulomb"].as<double>(0.0);

    // 링크 부품들을 하나의 강체로 합치기
    for (const auto& p : jn["link"]) {
      const auto type = p["type"].as<std::string>();
      RigidBody b;
      if (type == "point") {
        b = makePointMass(p["mass"].as<double>() * mass_scale, vec3(p["pos"]));
      } else if (type == "cylinder") {  // CyberGear 본체 등
        int sign;
        const int ax = parseAxis(p["axis"].as<std::string>("z"), &sign);
        b = makeCylinder(p["mass"].as<double>() * mass_scale, p["radius"].as<double>(),
                         p["length"].as<double>(), vec3(p["pos"]), ax);
      } else if (type == "profile") {   // 알루미늄 프로파일
        int sign;
        const int ax = parseAxis(p["axis"].as<std::string>("x"), &sign);
        b = makeSquareRod(p["lin_density"].as<double>(0.45) * mass_scale, p["side"].as<double>(0.02),
                          p["length"].as<double>(), vec3(p["start"]), ax, sign);
      } else if (type == "custom") {    // CAD 에서 뽑은 값 직접 입력
        b.mass = p["mass"].as<double>() * mass_scale;
        b.com = vec3(p["com"]);
        const auto I = p["inertia"];    // [ixx, iyy, izz, ixy, ixz, iyz]
        b.inertia << I[0].as<double>(), I[3].as<double>(), I[4].as<double>(),
                     I[3].as<double>(), I[1].as<double>(), I[5].as<double>(),
                     I[4].as<double>(), I[5].as<double>(), I[2].as<double>();
        b.inertia *= mass_scale;
      } else {
        throw std::runtime_error("알 수 없는 부품 type: " + type);
      }
      j.link.add(b);
    }
    model.joints.push_back(j);
  }
  if (model.joints.empty()) throw std::runtime_error("모델 파일에 joints 가 없음: " + path);
  return model;
}

// ---------------------------------------------------------------------------
std::vector<Eigen::Matrix4d> DhDynamics::forwardKinematics(const Eigen::VectorXd& q) const {
  const size_t n = m_.joints.size();
  std::vector<Eigen::Matrix4d> T(n + 1);
  T[0].setIdentity();
  for (size_t i = 0; i < n; ++i) {
    const auto& dh = m_.joints[i].dh;
    T[i + 1] = T[i] * dhTransform(q(static_cast<long>(i)) + dh.theta_offset, dh.d, dh.a, dh.alpha);
  }
  return T;
}

Eigen::MatrixXd DhDynamics::massMatrix(const Eigen::VectorXd& q) const {
  const int n = dof();
  const auto T = forwardKinematics(q);
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n, n);
  Eigen::MatrixXd Jv(3, n), Jw(3, n);

  for (int i = 0; i < n; ++i) {  // 링크 i (frame i+1 에 붙음)
    const auto& body = m_.joints[i].link;
    if (body.mass <= 0) continue;
    const Eigen::Matrix3d R = T[i + 1].block<3, 3>(0, 0);
    const Eigen::Vector3d pc = T[i + 1].block<3, 1>(0, 3) + R * body.com;  // 무게중심 (전역)
    Jv.setZero();
    Jw.setZero();
    for (int j = 0; j <= i; ++j) {  // 관절 j 는 z_j 축(frame j)을 돈다
      const Eigen::Vector3d z = T[j].block<3, 1>(0, 2);
      const Eigen::Vector3d o = T[j].block<3, 1>(0, 3);
      Jv.col(j) = z.cross(pc - o);
      Jw.col(j) = z;
    }
    M += body.mass * Jv.transpose() * Jv + Jw.transpose() * (R * body.inertia * R.transpose()) * Jw;
  }
  for (int i = 0; i < n; ++i) M(i, i) += m_.joints[i].armature;
  return M;
}

Eigen::VectorXd DhDynamics::gravityVector(const Eigen::VectorXd& q) const {
  const int n = dof();
  const auto T = forwardKinematics(q);
  Eigen::VectorXd G = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    const auto& body = m_.joints[i].link;
    if (body.mass <= 0) continue;
    const Eigen::Vector3d pc = T[i + 1].block<3, 1>(0, 3) + T[i + 1].block<3, 3>(0, 0) * body.com;
    for (int j = 0; j <= i; ++j) {
      const Eigen::Vector3d z = T[j].block<3, 1>(0, 2);
      const Eigen::Vector3d o = T[j].block<3, 1>(0, 3);
      G(j) -= body.mass * z.cross(pc - o).dot(m_.gravity);  // G = −Σ m Jvᵀ g
    }
  }
  return G;
}

void DhDynamics::compute(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, Eigen::MatrixXd& M,
                         Eigen::MatrixXd& C, Eigen::VectorXd& G, Eigen::VectorXd& F) const {
  const int n = dof();
  M = massMatrix(q);
  G = gravityVector(q);

  // ∂M/∂q_k (중앙 차분)
  const double h = 1e-6;
  std::vector<Eigen::MatrixXd> dM(n);
  Eigen::VectorXd qp = q, qm = q;
  for (int k = 0; k < n; ++k) {
    qp(k) += h;
    qm(k) -= h;
    dM[k] = (massMatrix(qp) - massMatrix(qm)) / (2 * h);
    qp(k) = q(k);
    qm(k) = q(k);
  }
  // C_kj = Σ_i ½ (∂M_kj/∂q_i + ∂M_ki/∂q_j − ∂M_ij/∂q_k) q̇_i
  C = Eigen::MatrixXd::Zero(n, n);
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        C(k, j) += 0.5 * (dM[i](k, j) + dM[j](k, i) - dM[k](i, j)) * dq(i);

  F.resize(n);
  for (int i = 0; i < n; ++i) {
    const auto& j = m_.joints[i];
    F(i) = j.viscous * dq(i) + j.coulomb * std::tanh(dq(i) / 0.01);
  }
}

}  // namespace cybergear
