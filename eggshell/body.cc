#include "body.h"

#include "model.h"

void Body::Rotate(const Matrix3d& R) { R_ = R * R_; }

void Body::Rotate(const Quaterniond& q) {}

double Body::GetRotationalKE() const {
  return w_b().transpose() * I_b() * w_b();
}

void Body::Draw() const {
  DrawBox(p(), R(), side_lengths_);
  // Vector3d point_on_box = sides_ / 2.0;
  // DrawPoint(p_ + R_ * point_on_box);
}

Matrix3d Body::CalculateInertia(double m) const {
  switch (type_) {
    case BodyType::Box: {
      double x = side_lengths_(0);
      double y = side_lengths_(1);
      double z = side_lengths_(2);
      Matrix3d inertia;
      // clang-format off
      inertia << m / 12 * (y * y + z * z), 0, 0,
        0, m / 12 * (x * x + z * z), 0,
        0, 0, m / 12 * (x * x + y * y);
      // clang-format on
      return inertia;
    }
    default:
      Panic("No inertia calculation implemented for BodyType %d\n", type_);
  }
}
