#include "model.h"
#include "glog/logging.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector3f;

static double angle1 = 0.5;
static double angle2 = 0;
static Matrix3d random_R = Matrix3d::Identity();

void SimulationInitialization() {
  srand(time(0));
  random_R = RandomRotation();
};

void SimulationStep() {
  Eigen::AngleAxisd Raa1(angle1, Vector3d(0, 0, 1));
  Eigen::AngleAxisd Raa2(angle2, Vector3d(0, 1, 0));
  Quaterniond q = Raa1 * Raa2;
  Matrix3d R = q.matrix();

  DrawBox(Vector3d(1, 0, 0.5), R, Vector3d(1, 0.5, 0.5));
  DrawCapsule(Vector3d(-1, 0, 0.5), R, 0.2, 0.6);
  DrawSphere(Vector3d(0, 0, 0.5), R, 0.3);

  // Draw box with random initial orientation
  Vector3d box_origin{0, 0, 1.5};
  Vector3d box_dims{0.3, 0.3, 0.3};
  auto box_R = random_R * R;
  DrawBox(box_origin, box_R, box_dims);
  Vector3d point_on_box_body = box_dims/2.0;
  Vector3d point_on_box_global = box_origin + box_R * point_on_box_body;
  DrawPoint(point_on_box_global);

  DrawPoint(Vector3d(0, 0, 0));
  DrawLine(Vector3d(0, 0, 0), Vector3d(0, 0, 1));

  angle1 += 0.01;
  angle2 += 0.002;
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

Matrix3d RandomRotation() { return RandomRotationViaGramSchmidt(); }

Matrix3d RandomRotationViaQuaternion() {
  Quaterniond q;
  q.UnitRandom();
  LOG(INFO) << q.coeffs();
  return q.matrix();
}

Matrix3d RandomRotationViaGramSchmidt() {
  Matrix3d m = Matrix3d::Random();
  return GramSchmidt(m);
  // TODO: return m.householderQr().householderQ();
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

  return m_normalized;

  // TODO: stablized Gram-Schmidt
}
