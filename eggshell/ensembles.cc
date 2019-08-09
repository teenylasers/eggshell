#include "ensembles.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "constants.h"
#include "error.h"
#include "lcp.h"

void Ensemble::Init() {
  ConstructMassInertiaMatrixInverse();
  InitializeExternalForceTorqueVector();
  CHECK(CheckInitialConditions()) << "Check initial conditions failed.";
}

MatrixXd Ensemble::ComputeJ() const {
  CHECK(contacts_.empty())
      << "Should never get here when there are contact constraints present.";
  MatrixXd J = ComputeJ_Joints();
  return J;
}

MatrixXd Ensemble::ComputeJ(ArrayXb& C, VectorXd& x_lo, VectorXd& x_hi) const {
  const int jn = 3;  // num rows in J per joint
  const int cn = 3;  // num rows in J per contact
  const int num_joint_constraints = jn * joints_.size();
  const int num_contact_constraints = cn * contacts_.size();
  MatrixXd J(num_joint_constraints + num_contact_constraints, 6 * n_);
  // initialize to constraint type to all equality
  ArrayXb C_joints = ArrayXb::Constant(num_joint_constraints, true);
  ArrayXb C_contacts = ArrayXb::Constant(num_contact_constraints, true);
  C = ArrayXb::Constant(num_joint_constraints + num_contact_constraints, true);
  VectorXd x_lo_joints = VectorXd::Zero(num_joint_constraints);
  VectorXd x_lo_contacts = VectorXd::Zero(num_contact_constraints);
  x_lo = VectorXd::Zero(num_joint_constraints + num_contact_constraints);
  VectorXd x_hi_joints = VectorXd::Zero(num_joint_constraints);
  VectorXd x_hi_contacts = VectorXd::Zero(num_contact_constraints);
  x_hi = VectorXd::Zero(num_joint_constraints + num_contact_constraints);

  // clang-format off
  J << ComputeJ_Joints(),
       ComputeJ_Contacts(C_contacts, x_lo_contacts, x_hi_contacts);
  C << C_joints,
       C_contacts;
  x_lo << x_lo_joints,
          x_lo_contacts;
  x_hi << x_hi_joints,
          x_hi_contacts;
  // clang-format on

  // std::cout << "J =\n" << J << std::endl;
  // std::cout << "C =\n" << C << std::endl;
  // std::cout << "x_lo =\n" << x_lo << std::endl;
  // std::cout << "x_hi =\n" << x_hi << std::endl;

  CHECK(CheckJ(J));
  return J;
}

MatrixXd Ensemble::ComputeJ_Joints() const {
  MatrixXd J = MatrixXd::Zero(3 * joints_.size(), 6 * n_);
  for (int i = 0; i < joints_.size(); ++i) {
    MatrixXd j0(3, 6);
    MatrixXd j1(3, 6);
    joints_.at(i).j->ComputeJ(j0, j1);
    if (joints_.at(i).b0 == -1) {
      CHECK(j1.sum() == 0);
      J.block<3, 6>(i * 3, joints_.at(i).b1 * 6) = j0;
    } else if (joints_.at(i).b1 == -1) {
      CHECK(j1.sum() == 0);
      J.block<3, 6>(i * 3, joints_.at(i).b0 * 6) = j0;
    } else {
      J.block<3, 6>(i * 3, joints_.at(i).b0 * 6) = j0;
      J.block<3, 6>(i * 3, joints_.at(i).b1 * 6) = j1;
    }
  }
  return J;
}

MatrixXd Ensemble::ComputeJ_Contacts(ArrayXb& C, VectorXd& x_lo, VectorXd& x_hi) const {
  // TODO:
  // Do J dimensions need to be hard-coded here?
  const int jn = 3;  // num rows in J per contact

  MatrixXd J = MatrixXd::Zero(jn * contacts_.size(), 6 * n_);

  for (int i = 0; i < contacts_.size(); ++i) {
    MatrixXd j0(jn, 6), j1(jn, 6);
    Array<bool, Dynamic, 1> ct(jn);
    VectorXd c_lo(jn), c_hi(jn);
    contacts_.at(i).c.ComputeJ(j0, j1, ct, c_lo, c_hi);

    if (contacts_.at(i).b1 != -1) {
      J.block<jn, 6>(i * jn, contacts_.at(i).b0 * 6) = j0;
      J.block<jn, 6>(i * jn, contacts_.at(i).b1 * 6) = j1;
    } else {
      J.block<jn, 6>(i * jn, contacts_.at(i).b0 * 6) = j0;
    }

    C.block<jn, 1>(i * jn, 0) = ct;
    x_lo.block<jn, 1>(i * jn, 0) = c_lo;
    x_hi.block<jn, 1>(i * jn, 0) = c_hi;
  }
  return J;
}

bool Ensemble::CheckJ(const MatrixXd& J) const {
  if (J.rows() > J.cols()) {
    LOG(ERROR) << "J has " << J.rows() << " rows and " << J.cols()
               << " cols => singular.";
    return false;
  } else {
    return true;
  }
}

VectorXd Ensemble::ComputeJDotV() const {
  VectorXd JDotV(3 * joints_.size() + contacts_.size());
  JDotV << ComputeJDotV_Joints(), ComputeJDotV_Contacts();
  return JDotV;
}

VectorXd Ensemble::ComputeJDotV_Joints() const {
  VectorXd JdotV = VectorXd::Zero(3 * joints_.size());
  for (int i = 0; i < joints_.size(); ++i) {
    MatrixXd Jdot_b0 = MatrixXd::Zero(3, 6);
    MatrixXd Jdot_b1 = MatrixXd::Zero(3, 6);
    joints_.at(i).j->ComputeJDot(Jdot_b0, Jdot_b1);
    const int b0 = joints_.at(i).b0;
    const int b1 = joints_.at(i).b1;
    VectorXd v0 = VectorXd::Zero(6);
    VectorXd v1 = VectorXd::Zero(6);
    if (b0 == -1) {
      v1 << components_.at(b1)->v(), components_.at(b1)->w_g();
      JdotV.segment<3>(i * 3) = Jdot_b0 * v1;
    } else if (b1 == -1) {
      v0 << components_.at(b0)->v(), components_.at(b0)->w_g();
      JdotV.segment<3>(i * 3) = Jdot_b0 * v0;
    } else {
      v0 << components_.at(b0)->v(), components_.at(b0)->w_g();
      v1 << components_.at(b1)->v(), components_.at(b1)->w_g();
      JdotV.segment<3>(i * 3) = Jdot_b0 * v0 + Jdot_b1 * v1;
    }
  }
  return JdotV;
}

VectorXd Ensemble::ComputeJDotV_Contacts() const {
  Panic(
      "ODE time stepper does not need to compute JdotV for contact "
      "constraints. Other integrators do not yet support contacts.");

  // Implemented but unused.
  VectorXd JdotV = VectorXd::Zero(1 * contacts_.size());
  for (int i = 0; i < contacts_.size(); ++i) {
    MatrixXd Jdot_b0 = MatrixXd::Zero(1, 6);
    MatrixXd Jdot_b1 = MatrixXd::Zero(1, 6);
    contacts_.at(i).c.ComputeJDot(Jdot_b0, Jdot_b1);
    const int b0 = contacts_.at(i).b0;
    const int b1 = contacts_.at(i).b1;
    VectorXd v0 = VectorXd::Zero(6);
    VectorXd v1 = VectorXd::Zero(6);
    if (b0 == -1) {
      v1 << components_.at(b1)->v(), components_.at(b1)->w_g();
      JdotV.segment<1>(i) = Jdot_b0 * v1;
    } else if (b1 == -1) {
      v0 << components_.at(b0)->v(), components_.at(b0)->w_g();
      JdotV.segment<1>(i) = Jdot_b0 * v0;
    } else {
      v0 << components_.at(b0)->v(), components_.at(b0)->w_g();
      v1 << components_.at(b1)->v(), components_.at(b1)->w_g();
      JdotV.segment<1>(i) = Jdot_b0 * v0 + Jdot_b1 * v1;
    }
  }
  return JdotV;
}

VectorXd Ensemble::ComputePositionConstraintError() const {
  VectorXd prev_errors(0, 0);
  if (contacts_.empty()) {
    for (const auto& joint : joints_) {
      const auto e = joint.j->ComputeError();
      VectorXd errors(prev_errors.rows() + e.rows(), e.cols());
      errors << prev_errors, e;
      prev_errors = errors;
    }
  } else if (joints_.empty()) {
    for (const auto& contact : contacts_) {
      const auto e = contact.c.ComputeError();
      VectorXd errors(prev_errors.rows() + e.rows(), e.cols());
      errors << prev_errors, e;
      prev_errors = errors;
    }
  } else {
    LOG(ERROR) << "Mixed joints and contact constraints not yet implemented.";
    CHECK(false);
  }
  CHECK(CheckErrorDims(prev_errors))
      << "Unexpected error vector dimensions: " << prev_errors.rows() << "x"
      << prev_errors.cols();
  return prev_errors;
}

bool Ensemble::CheckInitialConditions() const {
  auto error = ComputePositionConstraintError();
  if (!error.isZero(kAllowNumericalError)) {
    LOG(ERROR) << "Initial error: " << std::endl << error;
    return false;
  } else {
    return true;
  }
}

void Ensemble::Draw() const {
  for (const auto& l : components_) {
    l->Draw();
  }
  for (const auto& j : joints_) {
    j.j->Draw();
  }
}

void Ensemble::ConstructMassInertiaMatrixInverse() {
  M_inverse_ = MatrixXd::Zero(n_ * 2 * 3, n_ * 2 * 3);
  for (int i = 0; i < n_; ++i) {
    const auto b = components_.at(i);
    M_inverse_.block<3, 3>(i * 2 * 3, i * 2 * 3) =
        1.0 / b->m() * Matrix3d::Identity();
    // TODO: think through I_b vs I_g here.
    M_inverse_.block<3, 3>((i * 2 + 1) * 3, (i * 2 + 1) * 3) =
        b->I_g().inverse();
  }
  // std::cout << "M^-1 = \n" << M_inverse_;
}

void Ensemble::InitializeExternalForceTorqueVector() {
  external_force_torque_ = VectorXd::Zero(n_ * 6);
  for (int i = 0; i < n_; ++i) {
    const auto b = components_.at(i);
    external_force_torque_.segment<3>(i * 2 * 3) = b->m() * kGravity;
    external_force_torque_.segment<3>((i * 2 + 1) * 3) =
        -1 * CrossMat(b->w_g()) * b->I_g() * b->w_g();  // TODO: -1?
  }
  // std::cout << "f_e = \n" << external_force_torque_;
}

void Ensemble::Step(double dt, Integrator g) {
  // Survey current state
  VectorXd v = GetCurrentVelocities();
  UpdateContacts();

  // Time step
  if (g == Integrator::EXPLICIT_EULER) {
    CHECK(contacts_.empty())
        << "Cannot use explicit Euler integrator when contact constraints "
           "are present.";
    StepVelocities_ExplicitEuler(dt, v);
    StepPositions_ExplicitEuler(dt, v);
  } else if (g == Integrator::IMPLICIT_MIDPOINT) {
    Panic(
        "Implicit midpoint integrator is not properly implemented and tested.");
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
    v.segment<3>(i * 2 * 3) = components_.at(i)->v();
    v.segment<3>((i * 2 + 1) * 3) = components_.at(i)->w_g();
  }
  // std::cout << "v = \n" << v;
  return v;
}

void Ensemble::UpdateComponentsVelocities(const Eigen::VectorXd& v) {
  for (int i = 0; i < n_; ++i) {
    components_.at(i)->SetV(v.segment<3>(i * 2 * 3));
    components_.at(i)->SetW_GlobalFrame(v.segment<3>((i * 2 + 1) * 3));
  }
}

void Ensemble::UpdateContacts() {
  contacts_.clear();
  // Detect collision with ground
  for (int i = 0; i < components_.size(); ++i) {
    std::shared_ptr<Body> b = components_.at(i);
    std::vector<ContactGeometry> cgs;
    CollideBoxAndGround(b->p(), b->R(), b->GetSideLengths(), &cgs);
    for (auto cg : cgs) {
      // TODO: okay to use raw pointer b.get()? would it ever become dangling?
      Contact c(b.get(), cg);
      ContactingBodies cb(c, i);
      contacts_.push_back(cb);
    }
  }

  // Detect collision with other bodies
  for (int i = 0; i < components_.size(); ++i) {
    for (int j = i + 1; j < components_.size(); ++j) {
      std::shared_ptr<Body> b0 = components_.at(i);
      std::shared_ptr<Body> b1 = components_.at(j);
      CollisionInfo ci;
      std::vector<ContactGeometry> cgs;
      CollideBoxes(b0->p(), b0->R(), b0->GetSideLengths(), b1->p(), b1->R(),
                   b1->GetSideLengths(), &ci, &cgs);
      for (auto cg : cgs) {
        Contact c(b0.get(), b1.get(), cg, ci);
        // TODO: okay to use raw pointer b.get()? would it ever become dangling?
        ContactingBodies cb(c, i, j);
        contacts_.push_back(cb);
      }
    }
  }
}

VectorXd Ensemble::ComputeVDot(const MatrixXd& J, const VectorXd& rhs) const {
  VectorXd v_dot(6 * n_);

  if (joints_.empty() && contacts_.empty()) {
    v_dot = M_inverse_ * external_force_torque_;
  } else {
    const MatrixXd JMJt = J * M_inverse_ * J.transpose();
    const VectorXd lambda = JMJt.ldlt().solve(rhs);
    v_dot = M_inverse_ * (external_force_torque_ + J.transpose() * lambda);
  }

  return v_dot;
}

VectorXd Ensemble::ComputeVDot(const MatrixXd& J, const VectorXd& rhs,
                               const ArrayXb& C, const VectorXd& x_lo,
                               const VectorXd& x_hi) const {
  VectorXd v_dot(6 * n_);

  if (joints_.empty() && contacts_.empty()) {
    v_dot = M_inverse_ * external_force_torque_;
  } else {
    // JMJt * lambda = rhs, rhs depends on Integrator type.
    // C indicates (in)equality constraints.
    // {x_lo, x_hi} set LCP solver x-limits in Ax=b+w.
    const MatrixXd JMJt = J * M_inverse_ * J.transpose();
    VectorXd lambda(JMJt.rows());
    VectorXd weights(JMJt.rows());
    if (!Lcp::MixedConstraintsSolver(JMJt, rhs, C, x_lo, x_hi, lambda,
                                     weights)) {
      Panic("Lcp::MixedConstraintsSolver exited without reaching a solution.");
    }
    v_dot = M_inverse_ * (external_force_torque_ + J.transpose() * lambda);
  }
  return v_dot;
}

VectorXd Ensemble::StepVelocities_ExplicitEuler(double dt, const VectorXd& v) {
  MatrixXd J = ComputeJ();
  MatrixXd JdotV = ComputeJDotV();
  VectorXd rhs = -J * M_inverse_ * external_force_torque_ - JdotV;
  VectorXd v_dot = ComputeVDot(J, rhs);
  VectorXd v_new = v + dt * v_dot;
  UpdateComponentsVelocities(v_new);
  return v_new;
}

void Ensemble::StepPositions_ExplicitEuler(double dt, const VectorXd& v) {
  for (int i = 0; i < n_; ++i) {
    Vector3d p_new = components_.at(i)->p() + dt * v.segment<3>(i * 2 * 3);
    components_.at(i)->SetP(p_new);
    Matrix3d R_new = WtoQ(v.segment<3>((i * 2 + 1) * 3), dt).matrix() *
                     components_.at(i)->R();
    components_.at(i)->SetR(R_new);
  }
}

VectorXd Ensemble::StepVelocities_ODE(double dt, const VectorXd& v,
                                      double error_reduction_param) {
  ArrayXb C;
  VectorXd x_lo, x_hi;
  MatrixXd J = ComputeJ(C, x_lo, x_hi);
  VectorXd joint_error = ComputePositionConstraintError();
  VectorXd rhs = -error_reduction_param / dt / dt * joint_error -
                 J * (v / dt + M_inverse_ * external_force_torque_);
  VectorXd v_dot = ComputeVDot(J, rhs, C, x_lo, x_hi);
  VectorXd v_new = v + dt * v_dot;
  UpdateComponentsVelocities(v_new);
  return v_new;
}

void Ensemble::StepPositions_ODE(double dt, const VectorXd& v,
                                 const VectorXd& v_new) {
  for (int i = 0; i < n_; ++i) {
    Vector3d v_mid =
        (v.segment<3>(i * 2 * 3) + v_new.segment<3>(i * 2 * 3)) / 2.0;
    Vector3d p_new = components_.at(i)->p() + dt * v_mid;
    components_.at(i)->SetP(p_new);

    Vector3d w_mid =
        (v.segment<3>((i * 2 + 1) * 3) + v_new.segment<3>((i * 2 + 1) * 3)) /
        2.0;
    Matrix3d R_new = WtoQ(w_mid, dt).matrix() * components_.at(i)->R();
    components_.at(i)->SetR(R_new);
  }
}

VectorXd Ensemble::StepVelocities_ImplicitMidpoint(double dt, const VectorXd& v,
                                                   double alpha, double beta) {
  return VectorXd::Zero(6 * n_);
}

void Ensemble::StepPositions_ImplicitMidpoint(double dt, const VectorXd& v,
                                              const VectorXd& v_new,
                                              double alpha, double beta) {}

void Ensemble::Stablize(const int max_steps) {
  Eigen::VectorXd err = ComputePositionConstraintError();
  double err_sq = (err * err.transpose()).sum();
  // std::cout << "Pre-stabilization error sum : " << err_sq;
  int step_counter = 0;
  while (err_sq > kAllowNumericalError && step_counter < max_steps) {
    // TODO: position relaxation versus post stablization
    // TODO: should it be post stablization or velocity stablization?
    // StepPositionRelaxation(kSimTimeStep*1000);
    StepPostStabilization(kSimTimeStep * 100);
    err = ComputePositionConstraintError();
    err_sq = (err * err.transpose()).sum();
    ++step_counter;
  }
  // std::cout << "Post-stabilization steps count : " << step_counter;
  // std::cout << "Explicit Euler post-stabilization error_sq sum : " <<
  // err_sq;
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
  VectorXd err = ComputePositionConstraintError();
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
    components_.push_back(std::shared_ptr<Body>(new Body(p, v, R, w)));
  }
}

void Chain::InitJoints() {
  Vector3d c1{0.15, -0.15, 0.15};
  Vector3d c2{-0.15, 0.15, -0.15};
  for (int i = 0; i < n_ - 1; ++i) {
    // TODO: should joints use plain pointer or shared_ptr?
    joints_.push_back(JointBodies(
        std::shared_ptr<Joint>(new BallAndSocketJoint(
            components_.at(i).get(), c1, components_.at(i + 1).get(), c2)),
        i, i + 1));
  }
}

void Chain::SetAnchor() {
  Vector3d anchor_position = components_.at(0)->p();
  // TODO: should joints use plain pointer or shared_ptr?
  joints_.push_back(JointBodies(
      std::shared_ptr<Joint>(new BallAndSocketJoint(
          components_.at(0).get(), Vector3d::Zero(), anchor_position)),
      0));
}

bool Chain::CheckErrorDims(VectorXd error) const {
  // TODO: hardcoded for all BallAndSocketJoints, can be smarter or delete
  // after debug because Ensemble::ComputeJointError is correct by
  // construction?
  return error.rows() == 3 * joints_.size() && error.cols() == 1;
}

Cairn::Cairn(int num_rocks, const std::array<double, 2>& x_bound,
             const std::array<double, 2>& y_bound,
             const std::array<double, 2>& z_bound) {
  n_ = num_rocks;

  // Randomly suspend rocks of the cairn
  Vector3d p;
  Matrix3d R;
  Vector3d v;
  Vector3d w;
  double m = 1.0;
  Matrix3d I = Eigen::Matrix3d::Identity() * 0.1;
  for (int i = 0; i < num_rocks; ++i) {
    p = RandomPosition(x_bound, y_bound, z_bound);
    R = RandomRotation();
    v = RandomVelocity(max_init_v_);
    w = RandomAngularVelocity(max_init_w_);
    components_.push_back(std::shared_ptr<Body>(new Body(p, v, m, R, w, I)));
  }

  // TODO: Check for overlaps (severe collisions) and move them into a correct
  // initial configuration.
}

bool Cairn::CheckErrorDims(VectorXd) const { return true; }
