#include "body.h"
#include "model.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector3f;

void Body::Rotate(const Matrix3d& R) { R_ = R * R_; }

void Body::Rotate(const Quaterniond& q) {}

void Body::Draw() const {
  // TODO: for now, Body is a cube of size (0.3, 0.3, 0.3). Make variable in the
  // future.
  DrawBox(p_, R_, sides_);
  // Vector3d point_on_box = sides_ / 2.0;
  // DrawPoint(p_ + R_ * point_on_box);
}

double Body::GetRotationalKE() const {
  return w_b().transpose() * I_b() * w_b();
}
