#include "joints.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

Vector3d BallAndSocketJoint::ComputeError() const {
  Vector3d error = Vector3d::Zero();
  if (b2_ == nullptr) {
    error = b1_->p() + b1_->R() * c1_ - c2_;
  } else {
    error = b1_->p() + b1_->R() * c1_ - b2_->p() - b2_->R() * c2_;
  }
  return error;
}

void BallAndSocketJoint::ComputeJ(MatrixXd& J_b1, MatrixXd& J_b2) const {
  Matrix3d J_w1 = -1 * CrossMat(b1_->R() * c1_);
  J_b1 << Matrix3d::Identity(), J_w1;
  if (b2_ == nullptr) {
    J_b2 << Matrix3d::Zero(), Matrix3d::Zero();
  } else {
    Matrix3d J_w2 = CrossMat(b2_->R() * c2_);
    J_b2 << -1 * Matrix3d::Identity(), J_w2;
  }
}

void BallAndSocketJoint::ComputeJDot(MatrixXd& Jdot_b1,
                                     MatrixXd& Jdot_b2) const {
  Matrix3d Jdot_w1 = -1 * CrossMat(b1_->w_g().cross(b1_->R() * c1_));
  Jdot_b1 << Matrix3d::Zero(), Jdot_w1;
  if (b2_ != nullptr) {
    Matrix3d Jdot_w2 = CrossMat(b2_->w_g().cross(b2_->R() * c2_));
    Jdot_b2 << Matrix3d::Zero(), Jdot_w2;
  } else {
    Jdot_b2 << Matrix3d::Zero(), Matrix3d::Zero();
  }
}

void BallAndSocketJoint::Draw() const {
  DrawPoint(b1_->p() + b1_->R() * c1_);
  if (b2_ != nullptr) {
    DrawPoint(b2_->p() + b2_->R() * c2_);
  }
}
