#include "ensembles.h"

#include <cmath>
#include <memory>

#include "constants.h"
#include "glog/logging.h"
#include "lcp.h"

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

MatrixXd Ensemble::ComputeJ() const {
  MatrixXd J = MatrixXd::Zero(3 * joints_.size(), 6 * n_);
  for (int i = 0; i < joints_.size(); ++i) {
    MatrixXd J_b0(3, 6);
    MatrixXd J_b1(3, 6);
    joints_.at(i).j->ComputeJ(J_b0, J_b1);
    if (joints_.at(i).b1 != -1) {
      J.block<3, 6>(i * 3, joints_.at(i).b0 * 6) = J_b0;
      J.block<3, 6>(i * 3, joints_.at(i).b1 * 6) = J_b1;
    } else {
      CHECK(joints_.at(i).b1 == -1);
      J.block<3, 6>(i * 3, joints_.at(i).b0 * 6) = J_b0;
    }
  }
  CHECK(CheckJDims(J)) << "Unexpected J matrix dimensions: " << J.rows() << "x"
                       << J.cols();
  return J;
}

VectorXd Ensemble::ComputeJDotV() const {
  VectorXd JdotV = VectorXd::Zero(6 * n_);
  // for joints_
  for (int i = 0; i < joints_.size(); ++i) {
    MatrixXd Jdot_b0(3, 6);
    MatrixXd Jdot_b1(3, 6);
    joints_.at(i).j->ComputeJDot(Jdot_b0, Jdot_b1);
    const int b0 = joints_.at(i).b0;
    const int b1 = joints_.at(i).b1;
    if (b1 != -1) {
      VectorXd v0(6);
      v0.segment<3>(0) = components_.at(b0).v();
      v0.segment<3>(3) = components_.at(b0).w_g();
      VectorXd v1(6);
      v1.segment<3>(0) = components_.at(b1).v();
      v1.segment<3>(3) = components_.at(b1).w_g();
      JdotV.segment<6>(joints_.at(i).b0 * 6) = Jdot_b0 * v0;
      JdotV.segment<6>(joints_.at(i).b1 * 6) = Jdot_b1 * v1;
    } else {
      VectorXd v0(6);
      v0.segment<3>(0) = components_.at(b0).v();
      v0.segment<3>(3) = components_.at(b0).w_g();
      JdotV.segment<6>(b0 * 6) = Jdot_b0 * v0;
    }
  }
  return JdotV;
}

VectorXd Ensemble::ComputeJointError() const {
  VectorXd prev_errors(0, 0);
  for (const auto& joint : joints_) {
    const auto e = joint.j->ComputeError();
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

void Ensemble::Draw() const {
  for (const auto& l : components_) {
    l.Draw();
  }
  for (const auto& j : joints_) {
    j.j->Draw();
  }
}

void Ensemble::ConstructMassInertiaMatrixInverse() {
  M_inverse_ = MatrixXd::Zero(n_ * 2 * 3, n_ * 2 * 3);
  for (int i = 0; i < n_; ++i) {
    const Body& b = components_.at(i);
    M_inverse_.block<3, 3>(i * 2 * 3, i * 2 * 3) =
        1.0 / b.m() * Matrix3d::Identity();
    // TODO: think through I_b vs I_g here.
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
        -1 * CrossMat(b.w_g()) * b.I_g() * b.w_g();  // TODO: -1?
  }
  // LOG(INFO) << "f_e = \n" << external_force_torque_;
}

void Ensemble::Step(double dt, Integrator g) {
  VectorXd v = GetCurrentVelocities();
  if (g == Integrator::EXPLICIT_EULER) {
    StepVelocities_ExplicitEuler(dt, v);
    StepPositions_ExplicitEuler(dt, v);
  } else if (g == Integrator::IMPLICIT_MIDPOINT) {
    VectorXd v_new = StepVelocities_ImplicitMidpoint(dt, v);
    StepPositions_ImplicitMidpoint(dt, v, v_new);
  } else if (g == Integrator::OPEN_DYNAMICS_ENGINE) {
    VectorXd v_new = StepVelocities_ODE(dt, v);
    StepPositions_ODE(dt, v, v_new);
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
  VectorXd v_dot;
  if (joints_.empty() && contacts_.empty()) {
    v_dot = M_inverse_ * external_force_torque_;
  } else if (contacts_.empty()) {
    MatrixXd J = ComputeJ();
    MatrixXd JdotV = ComputeJDotV();
    MatrixXd JMJt = J * M_inverse_ * J.transpose();
    MatrixXd rhs = J * M_inverse_ * external_force_torque_ + JdotV;
    VectorXd lambda = JMJt.ldlt().solve(rhs);
    v_dot = M_inverse_ * external_force_torque_ -
            M_inverse_ * J.transpose() * lambda;
  } else if (joints_.empty()) {
    MatrixXd J = ComputeJ();
    MatrixXd JdotV = ComputeJDotV();
    MatrixXd JMJt = J * M_inverse_ * J.transpose();
    MatrixXd rhs = J * M_inverse_ * external_force_torque_ + JdotV;
    // JMJt * lambda >= rhs = rhs + w, lambda >= 0, w >= 0
    VectorXd lambda(JMJt.rows());
    VectorXd weights(JMJt.rows());
    Lcp::MurtyPrinciplePivot(JMJt, rhs, lambda, weights);
  } else {
    CHECK(false)
        << "Still need to implement when joints_ and contacts_ are both not "
           "empty.";
  }
  // TODO: invariance check
  // error = J * v_dot + J_dot * v, error.norm() < kAllowedNumericalError
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

VectorXd Ensemble::StepVelocities_ImplicitMidpoint(double dt, const VectorXd& v,
                                                   double alpha, double beta) {
  return Vector3d::Zero();
}

void Ensemble::StepPositions_ImplicitMidpoint(double dt, const VectorXd& v,
                                              const VectorXd& v_new,
                                              double alpha, double beta) {}

VectorXd Ensemble::StepVelocities_ODE(double dt, const VectorXd& v,
                                      double error_reduction_param) {
  VectorXd v_new;
  if (joints_.empty()) {
    v_new = v + dt * M_inverse_ * external_force_torque_;
  } else {
    MatrixXd J = ComputeJ();
    MatrixXd JMJt = J * M_inverse_ * J.transpose();
    VectorXd joint_error = ComputeJointError();
    MatrixXd rhs = -error_reduction_param / dt / dt * joint_error -
                   J * (v / dt + M_inverse_ * external_force_torque_);
    MatrixXd lambda = JMJt.ldlt().solve(rhs);
    v_new =
        v + dt * M_inverse_ * (external_force_torque_ + J.transpose() * lambda);
  }
  UpdateComponentsVelocities(v_new);
  return v_new;
}

void Ensemble::StepPositions_ODE(double dt, const VectorXd& v,
                                 const VectorXd& v_new) {
  for (int i = 0; i < n_; ++i) {
    Vector3d v_mid =
        (v.segment<3>(i * 2 * 3) + v_new.segment<3>(i * 2 * 3)) / 2.0;
    Vector3d p_new = components_.at(i).p() + dt * v_mid;
    components_.at(i).SetP(p_new);

    Vector3d w_mid =
        (v.segment<3>((i * 2 + 1) * 3) + v_new.segment<3>((i * 2 + 1) * 3)) /
        2.0;
    Matrix3d R_new = WtoQ(w_mid, dt).matrix() * components_.at(i).R();
    components_.at(i).SetR(R_new);
  }
}

void Ensemble::StepPositionRelaxation(double dt, double step_scale) {
  VectorXd velocity_relaxation = CalculateVelocityRelaxation(step_scale);
  StepPositions_ExplicitEuler(dt, velocity_relaxation);
}

void Ensemble::StepPostStabilization(double dt, double step_scale) {
  VectorXd velocity_relaxation = CalculateVelocityRelaxation(step_scale);
  StepPositions_ExplicitEuler(dt, velocity_relaxation);
  VectorXd v = GetCurrentVelocities();
  UpdateComponentsVelocities(v + velocity_relaxation);
}

VectorXd Ensemble::CalculateVelocityRelaxation(double step_scale) {
  MatrixXd J = ComputeJ();
  VectorXd err = ComputeJointError();
  CHECK(J.rows() == err.rows()) << "(J.rows() = " << J.rows()
                                << ") != (err.rows() = " << err.rows() << ")";
  VectorXd velocity_correction =
      -1.0 * step_scale * J.transpose() * (J * J.transpose()).ldlt().solve(err);
  return velocity_correction;
}

Chain::Chain(int num_links, const Vector3d& anchor_position) {
  // TODO: set max allowed n_
  CHECK(num_links > 0) << "Cannot create a Chain with " << num_links
                       << " links, must be > 0.";
  n_ = num_links;

  InitLinks(anchor_position);
  InitJoints();
  SetAnchor();
}

void Chain::InitLinks(const Vector3d& anchor_position) {
  Vector3d v = Vector3d::Zero();
  Quaterniond q =
      Eigen::AngleAxisd(0.95531661812451, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX());
  Matrix3d R = q.matrix();
  Vector3d w = Vector3d::Zero();
  for (int i = 0; i < n_; ++i) {
    Vector3d p{sqrt(3.0) * 0.3 * i, 0, 0};
    p = p + anchor_position;
    Body b(p, v, R, w);
    components_.push_back(b);
  }
}

void Chain::InitJoints() {
  Vector3d c1{0.15, -0.15, 0.15};
  Vector3d c2{-0.15, 0.15, -0.15};
  for (int i = 0; i < n_ - 1; ++i) {
    joints_.push_back(
        JointBodies(std::shared_ptr<Joint>(new BallAndSocketJoint(
                        components_.at(i), c1, components_.at(i + 1), c2)),
                    i, i + 1));
  }
}

void Chain::SetAnchor() {
  Vector3d anchor_position = components_.at(0).p();
  joints_.push_back(
      JointBodies(std::shared_ptr<Joint>(new BallAndSocketJoint(
                      components_.at(0), Vector3d::Zero(), anchor_position)),
                  0));
}

bool Chain::CheckErrorDims(VectorXd error) const {
  // TODO: hardcoded for all BallAndSocketJoints, can be smarter or delete
  // after debug because Ensemble::ComputeJointError is correct by
  // construction?
  return error.rows() == 3 * joints_.size() && error.cols() == 1;
}

bool Chain::CheckJDims(MatrixXd j) const {
  return j.rows() == 3 * joints_.size() && j.cols() == 6 * n_;
}

Cairn::Cairn(int num_rocks, const std::array<double, 2>& x_bound,
             const std::array<double, 2>& y_bound,
             const std::array<double, 2>& z_bound) {
  n_ = num_rocks;
  const double max_v = 5;
  const double max_w = 15;
  Vector3d p;
  Matrix3d R;
  Vector3d v;
  Vector3d w;
  for (int i = 0; i < num_rocks; ++i) {
    p = RandomPosition(x_bound, y_bound, z_bound);
    R = RandomRotation();
    v = RandomVelocity(max_v);
    w = RandomAngularVelocity(max_w);
    Body b(p, v, R, w);
    components_.push_back(b);
  }
}

bool Cairn::CheckErrorDims(VectorXd) const { return true; }
bool Cairn::CheckJDims(MatrixXd) const { return true; }
