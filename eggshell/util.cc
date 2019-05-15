#include "util.h"

#include "constants.h"

using namespace Eigen;

Matrix3d CrossMat(const Vector3d& a) {
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

Matrix3d GramSchmidt(const Matrix3d& m) {
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

Quaterniond WtoQ(const Vector3d& w, double dt) {
  // TODO: this implementation handles zero angular velocity correctly because
  // Eigen normalizes near-zero vectors to (0,0,0). However, normalize() uses
  // Pythagorean magnitude and thus overflows earlier than is optimal.
  Eigen::AngleAxisd aa(w.norm() * dt, w.normalized());
  Quaterniond q(aa);
  return q;
}
