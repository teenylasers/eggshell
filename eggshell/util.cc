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

Vector3d RandomPosition(const std::array<double, 2>& x_bound,
                        const std::array<double, 2>& y_bound,
                        const std::array<double, 2>& z_bound) {
  // Generate a random Vector3d, whose coeffients are <0, 1>;
  Vector3d v = (Vector3d::Random() + Vector3d::Ones()) / 2;
  v(0) = v(0) * std::abs(x_bound.at(1) - x_bound.at(0)) +
         std::min(x_bound.at(0), x_bound.at(1));
  v(1) = v(1) * std::abs(y_bound.at(1) - y_bound.at(0)) +
         std::min(y_bound.at(0), y_bound.at(1));
  v(2) = v(2) * std::abs(z_bound.at(1) - z_bound.at(0)) +
         std::min(z_bound.at(0), z_bound.at(1));
  return v;
}

Vector3d RandomVelocity(double limit) {
  Vector3d v = Vector3d::Random() * limit;
  return v;
}

Vector3d RandomAngularVelocity(double limit) {
  Vector3d v = Vector3d::Random() * limit;
  return v;
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
