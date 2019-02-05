#include "ensembles.h"

#include <cmath>
#include <memory>

#include "constants.h"
#include "glog/logging.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

void Ensemble::Init() {
  ConstructMassInertiaMatrixInverse();
  InitializeExternalForceTorqueVector();
  CHECK(CheckInitialConditions()) << "Check initial conditions failed.";
}

VectorXd Ensemble::ComputeJointError() const {
  VectorXd prev_errors(0, 0);
  for (const auto& joint : joints_) {
    const auto e = joint->ComputeError();
    VectorXd errors(prev_errors.rows() + e.rows(), e.cols());
    errors << prev_errors, e;
    prev_errors = errors;
  }
  CHECK(CheckErrorDims(prev_errors))
      << "Unexpected error vector dimensions: " << prev_errors.rows() << "x"
      << prev_errors.cols();
  return prev_errors;
}

bool Ensemble::CheckInitialConditions() const {
  auto error = ComputeJointError();
  if (!error.isZero(kAllowNumericalError)) {
    LOG(ERROR) << "Initial error: " << std::endl << error;
    return false;
  } else {
    return true;
  }
}

void Ensemble::ConstructMassInertiaMatrixInverse() {
  M_inverse_ = MatrixXd::Zero(n_ * 2 * 3, n_ * 2 * 3);
  for (int i = 0; i < n_; ++i) {
    const Body& b = components_.at(i);
    M_inverse_.block<3, 3>(i * 2 * 3, i * 2 * 3) =
        1.0 / b.m() * Matrix3d::Identity();
    // TODO: I_b vs I_g here.
    M_inverse_.block<3, 3>((i * 2 + 1) * 3, (i * 2 + 1) * 3) =
        b.I_g().inverse();
  }
  // LOG(INFO) << "M^-1 = \n" << M_inverse_;
}

void Ensemble::InitializeExternalForceTorqueVector() {
  external_force_torque_ = VectorXd::Zero(n_ * 6);
  for (int i = 0; i < n_; ++i) {
    const Body& b = components_.at(i);
    external_force_torque_.segment<3>(i * 2 * 3) = b.m() * kGravity;
    external_force_torque_.segment<3>((i * 2 + 1) * 3) =
        -1 * CrossMat(b.w_g()) * b.I_g() * b.w_g();
  }
  // LOG(INFO) << "f_e = \n" << external_force_torque_;
}

void Ensemble::Step(double dt, Integrator g) {
  VectorXd v = GetCurrentVelocities();
  if (g == Integrator::EXPLICIT_EULER) {
    StepVelocities_ExplicitEuler(dt, v);
    StepPositions_ExplicitEuler(dt, v);
  } else if (g == Integrator::LINEARIZED_IMPLICIT_MIDPOINT) {
    VectorXd v_new = StepVelocities_LIM(dt, v);
    StepPositions_LIM(dt, v, v_new);
  } else {
    LOG(ERROR) << "Unknown integrator type " << static_cast<int>(g);
  }
}

VectorXd Ensemble::GetCurrentVelocities() {
  VectorXd v = VectorXd::Zero(n_ * 6);
  for (int i = 0; i < n_; ++i) {
    v.segment<3>(i * 2 * 3) = components_.at(i).v();
    v.segment<3>((i * 2 + 1) * 3) = components_.at(i).w_g();
  }
  // LOG(INFO) << "v = \n" << v;
  return v;
}

void Ensemble::UpdateComponentsVelocities(const Eigen::VectorXd& v) {
  for (int i = 0; i < n_; ++i) {
    components_.at(i).SetV(v.segment<3>(i * 2 * 3));
    components_.at(i).SetW_GlobalFrame(v.segment<3>((i * 2 + 1) * 3));
  }
}

VectorXd Ensemble::StepVelocities_ExplicitEuler(double dt, const VectorXd& v) {
  MatrixXd J = ComputeJ();
  MatrixXd J_dot = ComputeJDot();
  MatrixXd JMJt = J * M_inverse_ * J.transpose();
  MatrixXd lagrange_rhs = J * M_inverse_ * external_force_torque_ - J_dot * v;
  MatrixXd lambda = JMJt.ldlt().solve(lagrange_rhs);
  VectorXd v_dot =
      M_inverse_ * external_force_torque_ - M_inverse_ * J.transpose() * lambda;
  VectorXd v_new = v + dt * v_dot;
  UpdateComponentsVelocities(v_new);
  return v_new;
}

void Ensemble::StepPositions_ExplicitEuler(double dt, const VectorXd& v) {
  for (int i = 0; i < n_; ++i) {
    Vector3d p_new = components_.at(i).p() + dt * v.segment<3>(i * 2 * 3);
    components_.at(i).SetP(p_new);
    Matrix3d R_new = WtoQ(v.segment<3>((i * 2 + 1) * 3), dt).matrix() *
                     components_.at(i).R();
    components_.at(i).SetR(R_new);
  }
}

VectorXd Ensemble::StepVelocities_LIM(double dt, const VectorXd& v,
                                      double alpha, double beta) {
  return Vector3d::Zero();
}

void Ensemble::StepPositions_LIM(double dt, const VectorXd& v,
                                 const VectorXd& v_new, double alpha,
                                 double beta) {}

Chain::Chain(int num_links) {
  // TODO: set max allowed n_
  CHECK(num_links > 0) << "Cannot create a Chain with " << num_links
                       << " links, must be > 0.";
  n_ = num_links;

  InitLinks();
  InitJoints();
  SetAnchor();
}

void Chain::SetAnchor() {
  anchor_.reset(new BallAndSocketJoint(components_.at(0), Vector3d::Zero()));
}

void Chain::InitLinks() {
  Vector3d v = Vector3d::Zero();
  Quaterniond q =
      Eigen::AngleAxisd(0.95531661812451, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX());
  Matrix3d R = q.matrix();
  Vector3d w = Vector3d::Zero();
  for (int i = 0; i < n_; ++i) {
    Vector3d p{sqrt(3.0) * 0.3 * i, 0, 6};
    Body b(p, v, R, w);
    components_.push_back(b);
  }
}

void Chain::InitJoints() {
  Vector3d c1{0.15, -0.15, 0.15};
  Vector3d c2{-0.15, 0.15, -0.15};
  for (int i = 0; i < n_ - 1; ++i) {
    joints_.push_back(std::shared_ptr<Joint>(new BallAndSocketJoint(
        components_.at(i), c1, components_.at(i + 1), c2)));
  }
}

bool Chain::CheckErrorDims(VectorXd error) const {
  // TODO: hardcoded for all BallAndSocketJoints, can be smarter or delete after
  // debug because Ensemble::ComputeJointError is correct by construction?
  return error.rows() == 3 * joints_.size() && error.cols() == 1;
}

MatrixXd Chain::ComputeJ() const {
  MatrixXd J = MatrixXd::Zero(3 * (joints_.size() + 1), 6 * n_);
  // for joints_
  for (int i = 0; i < joints_.size(); ++i) {
    MatrixXd J_b1(3, 6);
    MatrixXd J_b2(3, 6);
    joints_.at(i)->ComputeJ(J_b1, J_b2);
    J.block<3, 6>(i * 3, i * 6) = J_b1;
    J.block<3, 6>(i * 3, (i + 1) * 6) = J_b2;
  }
  // for anchor_
  MatrixXd J_b1(3, 6);
  anchor_->ComputeJ(J_b1);
  J.block<3, 6>(joints_.size() * 3, 0) = J_b1;
  CHECK(CheckJDims(J)) << "Unexpected J matrix dimensions: " << J.rows() << "x"
                       << J.cols();
  return J;
}

MatrixXd Chain::ComputeJDot() const {
  MatrixXd Jdot = MatrixXd::Zero(3 * (joints_.size() + 1), 6 * n_);
  // for joints_
  for (int i = 0; i < joints_.size(); ++i) {
    MatrixXd Jdot_b1(3, 6);
    MatrixXd Jdot_b2(3, 6);
    joints_.at(i)->ComputeJDot(Jdot_b1, Jdot_b2);
    Jdot.block<3, 6>(i * 3, i * 6) = Jdot_b1;
    Jdot.block<3, 6>(i * 3, (i + 1) * 6) = Jdot_b2;
  }
  // for anchor_
  MatrixXd Jdot_b1(3, 6);
  anchor_->ComputeJDot(Jdot_b1);
  Jdot.block<3, 6>(joints_.size() * 3, 0) = Jdot_b1;
  CHECK(CheckJDims(Jdot)) << "Unexpected Jdot matrix dimensions: "
                          << Jdot.rows() << "x" << Jdot.cols();
  return Jdot;
}

bool Chain::CheckJDims(MatrixXd j) const {
  return j.rows() == 3 * (joints_.size() + 1) && j.cols() == 6 * n_;
}

void Chain::Draw() const {
  for (const auto& l : components_) {
    l.Draw();
  }
  for (const auto& j : joints_) {
    j->Draw();
  }
}
