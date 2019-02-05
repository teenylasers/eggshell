#include "joints.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

Vector3d Joint::ComputeError() const { return Vector3d::Zero(); }

Vector3d BallAndSocketJoint::ComputeError() const {
  Vector3d error = b1_->p() + b1_->R() * c1_ - b2_->p() - b2_->R() * c2_;
  return error;
}

void BallAndSocketJoint::ComputeJ(MatrixXd& J_b1, MatrixXd& J_b2) const {
  CHECK(b2_ != nullptr) << "Not a joint between 2 bodies, cannot compute J_b2.";
  Matrix3d J_w1 = -1 * CrossMat(b1_->R() * c1_);
  Matrix3d J_w2 = CrossMat(b2_->R() * c2_);
  J_b1 << Matrix3d::Identity(), J_w1;
  J_b2 << -1 * Matrix3d::Identity(), J_w2;
  // LOG(INFO) << "\n" << J_b1 << "\n" << J_b2;
}

void BallAndSocketJoint::ComputeJ(MatrixXd& J_b1) const {
  CHECK(b2_ == nullptr) << "This is a joint between 2 bodies, refuse to "
                           "compute J_b1 without J_b2.";
  Matrix3d J_w1 = -1 * CrossMat(b1_->R() * c1_);
  J_b1 << Matrix3d::Identity(), J_w1;
  // LOG(INFO) << "\n" << J_b1;
}

void BallAndSocketJoint::ComputeJDot(MatrixXd& Jdot_b1,
                                     MatrixXd& Jdot_b2) const {
  CHECK(b2_ != nullptr)
      << "Not a joint between 2 bodies, cannot compute Jdot_b2.";
  Matrix3d Jdot_w1 = -1 * CrossMat(CrossMat(b1_->w_g()) * (b1_->R() * c1_));
  Jdot_b1 << Matrix3d::Zero(), Jdot_w1;
  Matrix3d Jdot_w2 = CrossMat(CrossMat(b2_->w_g()) * (b2_->R() * c2_));
  Jdot_b2 << Matrix3d::Zero(), Jdot_w2;
}

void BallAndSocketJoint::ComputeJDot(MatrixXd& Jdot_b1) const {
  CHECK(b2_ == nullptr) << "This is a joint between 2 bodies, refuse to "
                           "compute Jdot_b1 without Jdot_b2.";
  Matrix3d Jdot_w1 = -1 * CrossMat(CrossMat(b1_->w_g()) * (b1_->R() * c1_));
  Jdot_b1 << Matrix3d::Zero(), Jdot_w1;
}

void BallAndSocketJoint::Draw() const {
  DrawPoint(b1_->p() + b1_->R() * c1_);
  DrawPoint(b2_->p() + b2_->R() * c2_);
}
