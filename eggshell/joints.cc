#include "joints.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

Vector3d Joint::ComputeError() const { return Vector3d::Zero(); }

MatrixXd Joint::ComputeJ() const { return Matrix3d::Identity(); }

Vector3d BallAndSocketJoint::ComputeError() const {
  Vector3d error = b1_->p() + b1_->R() * c1_ - b2_->p() - b2_->R() * c2_;
  return error;
}

MatrixXd BallAndSocketJoint::ComputeJ() const {
  Matrix3d J_w1 = -1 * CrossMat(b1_->R() * c1_);
  Matrix3d J_w2 = CrossMat(b2_->R() * c2_);
  MatrixXd J = MatrixXd::Zero(12, 12);
  J.block<3, 3>(0, 0) = Matrix3d::Identity();
  J.block<3, 3>(3, 3) = J_w1;
  J.block<3, 3>(6, 6) = -1 * Matrix3d::Identity();
  J.block<3, 3>(9, 9) = J_w2;
  return J;
}

void BallAndSocketJoint::Draw() const {
  DrawPoint(b1_->p() + b1_->R() * c1_);
  DrawPoint(b2_->p() + b2_->R() * c2_);
}
