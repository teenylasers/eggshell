#include "ensembles.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <set>
#include <string>

#include "constants.h"
#include "error.h"
#include "lcp.h"

namespace {
constexpr double kCfmCoeff = 1;
constexpr double kMinConstraintDistance = 1e-6;
}  // namespace

void Ensemble::Init() {
  ConstructMassInertiaMatrixInverse();
  InitializeExternalForceTorqueVector();
  CHECK(CheckInitialConditions()) << "Check initial conditions failed.";
  CheckAndCorrectEnsembleState();
}

MatrixXd Ensemble::ComputeJ() const {
  ArrayXb C;
  VectorXd x_lo, x_hi;
  const MatrixXd J = ComputeJ(&C, &x_lo, &x_hi);
  return J;
}

MatrixXd Ensemble::ComputeJ(ArrayXb* C, VectorXd* x_lo, VectorXd* x_hi) const {
  // TODO: check with russ
  std::vector<std::shared_ptr<Constraint>> constraints;
  constraints.insert(constraints.end(), joints_.begin(), joints_.end());
  constraints.insert(constraints.end(), contacts_.begin(), contacts_.end());

  MatrixXd J;

  for (int i = 0; i < constraints.size(); ++i) {
    MatrixXd j0, j1;
    ArrayXb ct;
    VectorXd c_lo;
    VectorXd c_hi;
    constraints.at(i)->ComputeJ(&j0, &j1, &ct, &c_lo, &c_hi);

    // Construct new rows of J from j0 and j1
    CHECK(j0.rows() == j1.rows());
    MatrixXd J_new_rows = MatrixXd::Zero(j0.rows(), 6 * n_);
    if (constraints.at(i)->i0_ >= 0) {
      J_new_rows.block(0, constraints.at(i)->i0_ * 6, j0.rows(), j0.cols()) =
          j0;
    }
    if (constraints.at(i)->i1_ >= 0) {
      J_new_rows.block(0, constraints.at(i)->i1_ * 6, j1.rows(), j1.cols()) =
          j1;
    }

    // Append new rows to J, C, x_lo, x_hi
    J.conservativeResize(J.rows() + J_new_rows.rows(), J_new_rows.cols());
    J.block(J.rows() - J_new_rows.rows(), 0, J_new_rows.rows(),
            J_new_rows.cols()) = J_new_rows;

    CHECK(C->cols() == ct.cols());
    C->conservativeResize(C->rows() + ct.rows(), ct.cols());
    C->block(C->rows() - ct.rows(), 0, ct.rows(), ct.cols()) = ct;

    CHECK(x_lo->cols() == c_lo.cols());
    x_lo->conservativeResize(x_lo->rows() + c_lo.rows(), x_lo->cols());
    x_lo->block(x_lo->rows() - c_lo.rows(), 0, c_lo.rows(), c_lo.cols()) = c_lo;

    CHECK(x_hi->cols() == c_hi.cols());
    x_hi->conservativeResize(x_hi->rows() + c_hi.rows(), x_hi->cols());
    x_hi->block(x_hi->rows() - c_hi.rows(), 0, c_hi.rows(), c_hi.cols()) = c_hi;
  }

  // DEBUG ONLY
  // std::cout << "=== ComputeJ() ===" << std::endl;
  // std::cout << "J =\n" << J << std::endl;
  // std::cout << "C =\n" << *C << std::endl;
  // std::cout << "x_lo =\n" << *x_lo << std::endl;
  // std::cout << "x_hi =\n" << *x_hi << std::endl;

  return J;
}

bool Ensemble::CheckJ(const MatrixXd& J) const {
  if (J.rows() > J.cols()) {
    // std::cout << "ERROR: J has " << J.rows() << " rows and " << J.cols()
    //           << " cols => singular." << std::endl;
    return false;
  } else {
    bool good = GetConditionNumber(J) < kGoodConditionNumber;
    // if (!good) {
    //   std::cout << "ERROR: GetConditionNumber(J) [" << GetConditionNumber(J)
    //             << "] > kGoodConditionNumber [" << kGoodConditionNumber
    //             << "]\n.";
    // }
    return good;
  }
}

VectorXd Ensemble::ComputeJDotV() const {
  VectorXd JDotV(3 * joints_.size() + contacts_.size());
  JDotV << ComputeJDotV_Joints(), ComputeJDotV_Contacts();
  return JDotV;
}

VectorXd Ensemble::ComputeJDotV_Joints() const {
  Panic(
      "Jdot is hardcoded to have 3 rows. Should be decided by the Constraint.");

  VectorXd JdotV = VectorXd::Zero(3 * joints_.size());
  for (int i = 0; i < joints_.size(); ++i) {
    MatrixXd Jdot_b0 = MatrixXd::Zero(3, 6);
    MatrixXd Jdot_b1 = MatrixXd::Zero(3, 6);
    joints_.at(i)->ComputeJDot(&Jdot_b0, &Jdot_b1);
    const int b0 = joints_.at(i)->i0_;
    const int b1 = joints_.at(i)->i1_;
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

  Panic(
      "Jdot is hardcoded to have 1 row. Should be decided by the Constraint.");

  // Implemented but unused.
  VectorXd JdotV = VectorXd::Zero(1 * contacts_.size());
  for (int i = 0; i < contacts_.size(); ++i) {
    MatrixXd Jdot_b0 = MatrixXd::Zero(1, 6);
    MatrixXd Jdot_b1 = MatrixXd::Zero(1, 6);
    contacts_.at(i)->ComputeJDot(&Jdot_b0, &Jdot_b1);
    const int b0 = contacts_.at(i)->i0_;
    const int b1 = contacts_.at(i)->i1_;
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
  for (const auto& joint : joints_) {
    const auto e = joint->ComputeError();
    VectorXd errors(prev_errors.rows() + e.rows(), e.cols());
    errors << prev_errors, e;
    prev_errors = errors;
  }
  for (const auto& contact : contacts_) {
    const auto e = contact->ComputeError();
    VectorXd errors(prev_errors.rows() + e.rows(), e.cols());
    errors << prev_errors, e;
    prev_errors = errors;
  }
  return prev_errors;
}

void Ensemble::Draw() const {
  for (const auto& l : components_) {
    l->Draw();
  }
  for (const auto& j : joints_) {
    j->Draw();
  }
  // DEBUG ONLY: Draw the contacts for viz
  for (const auto& ct : contacts_) {
    ct->Draw();
  }
}

bool Ensemble::CheckConservationOfEnergy() {
  double energy = 0;
  for (const auto& b : components_) {
    energy = energy + b->GetRotationalKE();
  }
  if (std::fabs(energy - total_rotational_ke_) > kAllowNumericalError &&
      total_rotational_ke_ != std::numeric_limits<double>::infinity()) {
    std::cout << "Total rotational KE was " << total_rotational_ke_
              << ", now it's " << energy << std::endl;
    total_rotational_ke_ = energy;
    return false;
  }
  total_rotational_ke_ = energy;
  return true;
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
}

void Ensemble::InitializeExternalForceTorqueVector() {
  external_force_torque_ = VectorXd::Zero(n_ * 6);
  for (int i = 0; i < n_; ++i) {
    const auto b = components_.at(i);
    external_force_torque_.segment<3>(i * 2 * 3) = b->m() * kGravity;
    external_force_torque_.segment<3>((i * 2 + 1) * 3) =
        -1 * CrossMat(b->w_g()) * b->I_g() * b->w_g();  // TODO: -1?
  }
}

bool Ensemble::CheckInitialConditions() const {
  auto error = ComputePositionConstraintError();
  if (!error.isZero(kAllowNumericalError)) {
    std::cout << "ERROR: Initial conditions: " << std::endl << error;
    return false;
  } else {
    return true;
  }
}

void Ensemble::CheckAndCorrectEnsembleState() {
  // 1. Check whether M_inverse_, external_force_torques_ have the correct
  // dimensions.
  if (M_inverse_.rows() != M_inverse_.cols() && M_inverse_.rows() != 6 * n_) {
    std::cout << "num components = " << n_ << ". M_inverse_ size = ("
              << M_inverse_.rows() << ", " << M_inverse_.cols() << ").\n";
    Panic("M_inverse_ dimensions are incorrect.");
  }
  if (external_force_torque_.size() != 6 * n_) {
    std::cout << "num components = " << n_ << ". external_force_torque_ size = "
              << external_force_torque_.size() << std::endl;
    Panic("external_force_torque_ dimensions are incorrect.");
  }

  // 2. Check whether there exist constraint pairs that are too close to each
  // other, which can cause overconstraint singular J. Try to correct if so.
  ConstructPairwiseJointsMap();
  ConstructPairwiseContactsMap();

  auto contacts_set_compare = [](int a, int b) { return a > b; };
  std::set<int, decltype(contacts_set_compare)> contacts_to_delete(
      contacts_set_compare);

  for (int i = 0; i < components_.size() - 1; ++i) {
    for (int j = i + 1; j < components_.size(); ++j) {
      const auto map_key = std::make_tuple(i, j);
      // Using operator[] here, expect an empty std::vector if map_key does not
      // exist in the pairwise_joints_ or pairwise_contacts_.
      auto& pair_joints = pairwise_joints_[map_key];
      auto& pair_contacts = pairwise_contacts_[map_key];

      // DEBUG ONLY
      // std::cout << "pair_joints.size = " << pair_joints.size()
      //           << ", pair_contacts.size = " << pair_contacts.size()
      //           << std::endl;

      // Check for neighbouring joint constraints. If fails, panic abort.
      for (auto it1 = pair_joints.begin(); it1 != pair_joints.end(); ++it1) {
        for (auto it2 = it1 + 1; it2 != pair_joints.end(); ++it2) {
          if (!CheckConstraintPair(joints_.at(*it1), joints_.at(*it2))) {
            Panic(
                "Joint constraints between components %d and %d conflict or "
                "cause overconstraint.",
                i, j);
          }
        }
      }

      // Check for a pair of neighbouring joint constraint and contact
      // constraint. If fails, delete the contact constraint.
      for (auto it1 = pair_joints.begin(); it1 != pair_joints.end(); ++it1) {
        for (auto it2 = pair_contacts.begin(); it2 != pair_contacts.end();
             ++it2) {
          if (!CheckConstraintPair(joints_.at(*it1), contacts_.at(*it2))) {
            std::cout << "Components " << i << " and " << j
                      << ", joint vs contact check.\n";
            contacts_to_delete.insert(*it2);
          }
        }
      }

      // Check neighbouring contact constraints. If fails, delete one of them.
      for (auto it1 = pair_contacts.begin(); it1 != pair_contacts.end();
           ++it1) {
        for (auto it2 = it1 + 1; it2 != pair_contacts.end(); ++it2) {
          if (!CheckConstraintPair(contacts_.at(*it1), contacts_.at(*it2))) {
            std::cout << "Components " << i << " and " << j
                      << ", contact vs contact check.\n";
            contacts_to_delete.insert(*it2);
          }
        }
      }
    }
  }  // Finish pairwise check of constraints.

  // Erase contacts_to_delete
  if (!contacts_to_delete.empty()) {
    std::cout << "contacts_.size() = " << contacts_.size()
              << ". Contacts to delete are: ";
    for (const auto c : contacts_to_delete) {
      std::cout << c << ", ";
    }
    std::cout << std::endl;
  }
  for (const auto c : contacts_to_delete) {
    contacts_.erase(contacts_.begin() + c);
  }
}

void Ensemble::ConstructPairwiseJointsMap() {
  // pairwise_joints_ only needs to be constructed once.
  if (!pairwise_joints_.empty()) {
    return;
  }

  for (int i = 0; i < joints_.size(); ++i) {
    const auto& j = joints_.at(i);
    auto map_key = j->i0_ < j->i1_ ? std::make_tuple(j->i0_, j->i1_)
                                   : std::make_tuple(j->i1_, j->i0_);
    std::vector<int> map_val{i};
    if (pairwise_joints_.find(map_key) == pairwise_joints_.end()) {
      pairwise_joints_.emplace(map_key, map_val);
    } else {
      pairwise_joints_.at(map_key).push_back(i);
    }
  }

  // DEBUG ONLY:
  // std::cout << "pairwise_joints_.size() = " << pairwise_joints_.size()
  //           << std::endl;
}

void Ensemble::ConstructPairwiseContactsMap() {
  // Always reconstruct pairwise_contacts_, because contacts_ are updated with
  // every stablization or time step.
  pairwise_contacts_.clear();

  for (int i = 0; i < contacts_.size(); ++i) {
    const auto& c = contacts_.at(i);
    auto map_key = c->i0_ < c->i1_ ? std::make_tuple(c->i0_, c->i1_)
                                   : std::make_tuple(c->i1_, c->i0_);
    std::vector<int> map_val{i};
    if (pairwise_contacts_.find(map_key) == pairwise_contacts_.end()) {
      pairwise_contacts_.emplace(map_key, map_val);
    } else {
      pairwise_contacts_.at(map_key).push_back(i);
    }
  }

  // DEBUG ONLY
  // std::cout << "pairwise_contacts_.size() = " << pairwise_contacts_.size()
  //           << std::endl;
}

bool Ensemble::CheckConstraintPair(const std::shared_ptr<Constraint> c1,
                                   const std::shared_ptr<Constraint> c2) const {
  Vector3d constraint_position_difference =
      c1->GetConstraintPosition() - c2->GetConstraintPosition();
  if (constraint_position_difference.norm() < kMinConstraintDistance) {
    std::cout << "Found conflict between constraints: distance = "
              << constraint_position_difference.norm() << "\n"
              << c1->PrintInfo() << std::endl
              << c2->PrintInfo() << std::endl;
    return false;
  }
  return true;
}

void Ensemble::Step(double dt, Integrator g) {
  // Survey current state
  const VectorXd v = GetVelocities();
  UpdateContacts();
  CheckAndCorrectEnsembleState();

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
    std::cout << "ERROR: Unknown integrator type " << static_cast<int>(g);
  }

  // DEBUG ONLY: print all linear velocities.
  // std::cout << "Num contacts = " << contacts_.size() << std::endl;
  // const VectorXd v1 = GetVelocities();
  // for (int i = 0; i < v1.size() / 6; ++i) {
  //   std::cout << "Linear velocity = " << v1.block<3, 1>(i * 6, 0).norm()
  //             << std::endl;
  // }

  // DEBUG ONLY: check and print rotational kinetic energy at each step.
  // if (!CheckConservationOfEnergy()) {
  //   Panic("Conservation of rotational kinetic energy violated. Exit.");
  // }
}

const VectorXd Ensemble::GetVelocities() const {
  VectorXd v = VectorXd::Zero(n_ * 6);
  for (int i = 0; i < n_; ++i) {
    v.segment<3>(i * 2 * 3) = components_.at(i)->v();
    v.segment<3>((i * 2 + 1) * 3) = components_.at(i)->w_g();
  }
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
      auto contact = std::shared_ptr<Contact>(new Contact(b, i, cg));
      contacts_.push_back(contact);
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
        auto contact =
            std::shared_ptr<Contact>(new Contact(b0, i, b1, j, cg, ci));
        contacts_.push_back(contact);
      }
    }
  }

  // DEBUG ONLY: Draw the contacts for viz
  // for (const auto& ct : contacts_) {
  //   ct->Draw();
  // }
}

VectorXd Ensemble::ComputeVDot(const MatrixXd& J, const VectorXd& rhs) const {
  VectorXd v_dot(6 * n_);

  if (joints_.empty() && contacts_.empty()) {
    v_dot = M_inverse_ * external_force_torque_;
  } else {
    CHECK(CheckJ(J));  // J needs to be non-singular for this algorithm to work.
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
  const int J_rows = J.rows();

  if (joints_.empty() && contacts_.empty()) {
    v_dot = M_inverse_ * external_force_torque_;
  } else {
    // JMJt * lambda = rhs, rhs depends on Integrator type.
    // C indicates (in)equality constraints.
    // {x_lo, x_hi} set LCP solver x-limits in Ax=b+w.
    const MatrixXd JMJt = J * M_inverse_ * J.transpose();
    MatrixXd Cfm = MatrixXd::Zero(J_rows, J_rows);
    // If CheckJ() finds an ill-conditioned J, then apply constraint force
    // mixing (cfm).
    if (!CheckJ(J)) {
      Cfm = kCfmCoeff * MatrixXd::Identity(J_rows, J_rows);
    }
    const MatrixXd lhs = JMJt + Cfm;

    // DEBUG ONLY: Print and visualize systems matrix structure.
    // std::cout << "JMJt \n" << JMJt << std::endl;
    // std::cout << "lhs = JMJt + Cfm \n" << lhs << std::endl;

    // Solve systems equation
    VectorXd lambda(J_rows);
    VectorXd weights(J_rows);
    if (!Lcp::MixedConstraintsSolver(lhs, rhs, C, x_lo, x_hi, lambda,
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
  MatrixXd J = ComputeJ(&C, &x_lo, &x_hi);
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

void Ensemble::InitStabilize() {
  UpdateContacts();
  Eigen::VectorXd err = ComputePositionConstraintError();
  double err_sq = err.transpose() * err;
  std::cout << "Initial err_sq : " << err_sq << std::endl;

  const int max_steps = 100;
  int step_counter = 0;
  while (err_sq > kAllowNumericalError && step_counter < max_steps) {
    StepPositionRelaxation(kSimTimeStep * 500);
    // StepPostStabilization(kSimTimeStep * 100);
    UpdateContacts();
    err = ComputePositionConstraintError();
    err_sq = err.transpose() * err;
    ++step_counter;
  }
  CheckAndCorrectEnsembleState();

  std::cout << "Pre-stabilization steps count : " << step_counter << std::endl;
  std::cout << "Final err_sq : " << err_sq << std::endl;
}

void Ensemble::PostStabilize(int max_steps) {
  Eigen::VectorXd err = ComputePositionConstraintError();
  double err_sq = err.transpose() * err;
  // std::cout << "err = " << err.transpose() << std::endl;
  // std::cout << "Pre-stabilization error sum : " << err_sq << std::endl;
  int step_counter = 0;
  while (err_sq > kAllowNumericalError && step_counter < max_steps) {
    // TODO: position relaxation versus post stablization
    // TODO: should it be post stablization or velocity stablization?
    // StepPositionRelaxation(kSimTimeStep*1000);
    StepPostStabilization(kSimTimeStep * 100);
    err = ComputePositionConstraintError();
    err_sq = err.transpose() * err;
    // std::cout << "err = " << err.transpose() << std::endl;
    // std::cout << "err_sq = " << err_sq << std::endl;
    ++step_counter;
  }
  // std::cout << "Post-stabilization steps count : " << step_counter <<
  // std::endl;
  // std::cout << "Explicit Euler post-stabilization error_sq sum : " << err_sq
  // << std::endl;
}

void Ensemble::StepPositionRelaxation(double dt, double step_scale) {
  const VectorXd velocity_relaxation = CalculateVelocityRelaxation(step_scale);
  StepPositions_ExplicitEuler(dt, velocity_relaxation);
}

void Ensemble::StepPostStabilization(double dt, double step_scale) {
  const VectorXd velocity_relaxation = CalculateVelocityRelaxation(step_scale);
  StepPositions_ExplicitEuler(dt, velocity_relaxation);
  const VectorXd v = GetVelocities();
  UpdateComponentsVelocities(v + velocity_relaxation);
}

VectorXd Ensemble::CalculateVelocityRelaxation(double step_scale) const {
  const MatrixXd J = ComputeJ();
  const VectorXd err = ComputePositionConstraintError();
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
    auto joint = std::shared_ptr<Joint>(new BallAndSocketJoint(
        components_.at(i), i, c1, components_.at(i + 1), i + 1, c2));
    joints_.push_back(joint);
  }
}

void Chain::SetAnchor() {
  Vector3d anchor_position = components_.at(0)->p();
  auto joint = std::shared_ptr<Joint>(new BallAndSocketJoint(
      components_.at(0), 0, Vector3d::Zero(), anchor_position));
  joints_.push_back(joint);
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
}
