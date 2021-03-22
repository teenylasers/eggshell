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

void BallAndSocketJoint::ComputeJ(MatrixXd* J_b0, MatrixXd* J_b1,
                                  ArrayXb* constraint_type,
                                  VectorXd* constraint_lo,
                                  VectorXd* constraint_hi) const {
  // Construct Jacobians
  J_b0->resize(3, 6);
  J_b1->resize(3, 6);

  Matrix3d J_w0 = -1 * CrossMat(b0_->R() * c0_);
  *J_b0 << Matrix3d::Identity(), J_w0;
  if (b1_ == nullptr) {
    *J_b1 << Matrix3d::Zero(), Matrix3d::Zero();
  } else {
    Matrix3d J_w1 = CrossMat(b1_->R() * c1_);
    *J_b1 << -1 * Matrix3d::Identity(), J_w1;
  }

  // Construct constraints
  constraint_type->resize(3, 1);
  *constraint_type << 1, 1, 1;
  *constraint_lo = VectorXd::Zero(3);
  *constraint_hi = VectorXd::Zero(3);
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

Vector3d BallAndSocketJoint::GetConstraintPosition() const {
  Vector3d p0, p1;
  if (b0_ != nullptr) {
    p0 = b0_->p() + b0_->R() * c0_;
  }
  if (b1_ != nullptr) {
    p1 = b1_->p() + b1_->R() * c1_;
  }

  if (b0_ != nullptr && b1_ != nullptr) {
    return (p0 + p1) / 2;
  } else if (b0_ != nullptr && b1_ == nullptr) {
    return p0;
  } else if (b0_ == nullptr && b1_ != nullptr) {
    return p1;
  } else {
    Panic("b0_ and b1_ are both nullptr.");
    return Vector3d::Zero();
  }
}
