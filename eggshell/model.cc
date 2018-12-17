#include "model.h"
#include "constants.h"
#include "glog/logging.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector3f;

static double angle1 = 0.5;
static double angle2 = 0;
static Matrix3d random_R = Matrix3d::Identity();
static Matrix3d R_normalized = Matrix3d::Identity();
static Matrix3d R_explicit = Matrix3d::Identity();
static Vector3d omega0{0, 0, 80}; // fixed angular velocity

void SimulationInitialization() {
  srand(time(0));
  random_R = RandomRotation();
  R_normalized = random_R;
  R_explicit = random_R;
};

void SimulationStep() {
  Eigen::AngleAxisd Raa1(angle1, Vector3d(0, 0, 1));
  Eigen::AngleAxisd Raa2(angle2, Vector3d(0, 1, 0));
  Quaterniond q = Raa1 * Raa2;
  Matrix3d R = q.matrix();

  DrawBox(Vector3d(1, 0, 0.5), R, Vector3d(1, 0.5, 0.5));
  DrawCapsule(Vector3d(-1, 0, 0.5), R, 0.2, 0.6);
  DrawSphere(Vector3d(0, 0, 0.5), R, 0.3);

  DrawPoint(Vector3d(0, 0, 0));
  DrawLine(Vector3d(0, 0, 0), Vector3d(0, 0, 1));

  angle1 += 0.01;
  angle2 += 0.002;

  // Draw box with random initial orientation
  Vector3d box_origin{0, 0, 1.5};
  Vector3d box_dims{0.3, 0.3, 0.3};
  auto box_R = R * random_R;
  DrawBox(box_origin, box_R, box_dims);
  Vector3d point_on_box_body = box_dims / 2.0;
  Vector3d point_on_box_global = box_origin + box_R * point_on_box_body;
  DrawPoint(point_on_box_global);

  // Draw boxes that rotates with angular velocity omega0, start with identical
  // boxes, use different integrators
  Vector3d normalized_box_origin{1.5, 0, 1.5};
  Vector3d explicit_box_origin{3, 0, 1.5};
  DrawBox(normalized_box_origin, R_normalized, box_dims);
  DrawBox(explicit_box_origin, R_explicit, box_dims);
  DrawPoint(normalized_box_origin + R_normalized * point_on_box_body);
  DrawPoint(explicit_box_origin + R_explicit * point_on_box_body);
  R_normalized = NormalizedRotationUpdate(R_normalized, omega0, kSimTimeStep);
  R_explicit = ExplicitEulerRotationUpdate(R_explicit, omega0, kSimTimeStep);

  // TODO: how to check implemented angular velocity
}

Matrix3d ExplicitEulerRotationUpdate(const Matrix3d &R, const Vector3d &omega0,
                                     double dt) {
  Matrix3d R_update = GramSchmidt(R + dt * CrossMat(omega0) * R);
  return R_update;
}

Matrix3d NormalizedRotationUpdate(const Matrix3d &R, const Vector3d &omega0,
                                  double dt) {
  Eigen::AngleAxisd Raa(omega0.norm() * dt, omega0.normalized());
  Quaterniond q(Raa);
  Matrix3d R_update = q.matrix() * R;
  return R_update;
}

Matrix3d CrossMat(const Vector3d &a) {
  Matrix3d m;
  // clang-format off
  m << 0, -a(2), a(1),
       a(2), 0, -a(0),
       -a(1), a(0), 0;
  // clang-format on
  return m;
}

Matrix3d RandomRotation() { return RandomRotationViaQuaternion(); }

Matrix3d RandomRotationViaQuaternion() {
  Quaterniond q = Quaterniond::UnitRandom();
  return q.matrix();
}

Matrix3d RandomRotationViaGramSchmidt() {
  Matrix3d m = GramSchmidt(Matrix3d::Random());
  // TODO: Matrix3d m = Matrix3d::Random().householderQr().householderQ();??
  if ((m.col(0).cross(m.col(1)) - m.col(2)).norm() > kAllowNumericalError) {
    m.col(2) = -1.0 * m.col(2);
  }
  return m;
}

Matrix3d GramSchmidt(const Matrix3d &m) {
  // u0, u1, u2 are normalized bases, orthogonal to each other

  // Naive Gram-Schmidt
  Vector3d u0 = m.col(0) / m.col(0).norm();
  Vector3d u1 = m.col(1) - m.col(1).dot(u0) * u0;
  u1 = u1 / u1.norm();
  Vector3d u2 = m.col(2) - m.col(2).dot(u0) * u0 - m.col(2).dot(u1) * u1;
  u2 = u2 / u2.norm();
  Matrix3d m_normalized;
  m_normalized << u0, u1, u2;

  // TODO: stablized Gram-Schmidt

  return m_normalized;
}
