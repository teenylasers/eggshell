#include "joints.h"


VectorXd BallAndSocketJoint::ComputeError() const {
  VectorXd error = Vector3d::Zero(3);
  if (b1_ == nullptr) {
    error = b0_->p() + b0_->R() * c0_ - c1_;
  } else {
    error = b0_->p() + b0_->R() * c0_ - b1_->p() - b1_->R() * c1_;
  }
  return error;
}

void BallAndSocketJoint::ComputeJ(MatrixXd* J_b0, MatrixXd* J_b1) const {
  Matrix3d J_w0 = -1 * CrossMat(b0_->R() * c0_);
  *J_b0 << Matrix3d::Identity(), J_w0;
  if (b1_ == nullptr) {
    *J_b1 << Matrix3d::Zero(), Matrix3d::Zero();
  } else {
    Matrix3d J_w1 = CrossMat(b1_->R() * c1_);
    *J_b1 << -1 * Matrix3d::Identity(), J_w1;
  }
}

void BallAndSocketJoint::ComputeJDot(MatrixXd* Jdot_b0,
                                     MatrixXd* Jdot_b1) const {
  Matrix3d Jdot_w0 = -1 * CrossMat(b0_->w_g().cross(b0_->R() * c0_));
  *Jdot_b0 << Matrix3d::Zero(), Jdot_w0;
  if (b1_ != nullptr) {
    Matrix3d Jdot_w1 = CrossMat(b1_->w_g().cross(b1_->R() * c1_));
    *Jdot_b1 << Matrix3d::Zero(), Jdot_w1;
  } else {
    *Jdot_b1 << Matrix3d::Zero(), Matrix3d::Zero();
  }
}

void BallAndSocketJoint::Draw() const {
  DrawPoint(b0_->p() + b0_->R() * c0_);
  if (b1_ != nullptr) {
    DrawPoint(b1_->p() + b1_->R() * c1_);
  }
}
