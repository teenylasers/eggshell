#include "body.h"
#include "model.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector3f;

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
