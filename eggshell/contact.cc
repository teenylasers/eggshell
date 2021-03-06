#include "contact.h"

#include <iostream>
#include <limits>

#include "constants.h"
#include "error.h"
#include "model.h"
#include "util.h"

namespace {
constexpr double kBoxFrictionBound = 100;
}

VectorXd Contact::ComputeError() const {
  if (f_ == FrictionModel::NO_FRICTION) {
    return VectorXd::Constant(1, -cg_.depth);
  } else {
    VectorXd error(3);
    error << 0, 0, -cg_.depth;
    return error;
  }
}

// TODO:
// Explain this out loud to make sure the logic makes sense.
//
// 1. write constraints in a coordinate system that is aligned with contact
// normal
//    - find the rotation matrix that re-writes vector a in global frame
//      to one that is in contact normal frame.
//    - row 3 of contact constraint in new frame should equal the no-friction
//      contact constraint. check that normal and transverse are separated out
//      in different row. (how is this result different from 1 row of dot-n
//      constraint and 3 rows of cross-n constraint)
//    - does the alignment/direction of transverse axes matter in some cases?
// 2. implement LCP solver that takes in low and high limits
// 3. Schur complement alternative to solve the mixed constraint problem.
void Contact::ComputeJ(MatrixXd* J_b0, MatrixXd* J_b1, ArrayXb* C,
                       VectorXd* x_lo, VectorXd* x_hi) const {
  // Constraint in the direction of contact normal is an inequality,
  // constraints that are transverse to contact normal are equalities.

  // 1. Write out the ball-n-socket J for the contact. R * c = x - p
  // 2. Find rotation matrices R, such that R*v expresses v in the local
  //    coordinates where z aligns with contact normal.
  // 3. Return R * J.
  // Because of the z alignment with contact normal, row 0-1 are equality
  // constraints, row 2 is an inequality constraint

  Vector3d z_axis(0, 0, 1);
  Matrix3d R = AlignVectors(cg_.normal, z_axis);

  // TODO: debug only
  // std::cout << "R\n" << R << std::endl;

  MatrixXd j0(3, 6);
  if (b0_ == nullptr) {
    j0 << Matrix3d::Zero(), Matrix3d::Zero();
  } else {
    Matrix3d J_v0 = -1 * Matrix3d::Identity();
    Matrix3d J_w0 = CrossMat(cg_.position - b0_->p());
    j0 << R * J_v0, R * J_w0;
  }

  MatrixXd j1(3, 6);
  if (b1_ == nullptr) {
    j1 << Matrix3d::Zero(), Matrix3d::Zero();
  } else {
    Matrix3d J_v1 = Matrix3d::Identity();
    Matrix3d J_w1 = -1 * CrossMat(cg_.position - b1_->p());
    j1 << R * J_v1, R * J_w1;
  }

  // TODO: debug only
  // std::cout << "J_b1 in contact frame = \n" << *J_b1 << std::endl;
  // std::cout << "J_b1 (global frame) *dot* contact normal = "
  //           << cg_.normal.transpose() * J_v1 << " "
  //           << cg_.normal.transpose() * J_w1 << std::endl;

  switch (f_) {
    case FrictionModel::NO_FRICTION:
      // When there is no friction, there is only constraint in the direction of
      // contact normal. We can discard the first 2 rows of J
      *J_b0 = j0.block(j0.rows() - 1, 0, 1, j0.cols());
      *J_b1 = j1.block(j1.rows() - 1, 0, 1, j1.cols());
      C->resize(1);
      x_lo->resize(1);
      x_hi->resize(1);
      *C << 0;
      *x_lo << 0;
      *x_hi << std::numeric_limits<double>::infinity();
      break;
    case FrictionModel::INFINITE:
      *J_b0 = j0;
      *J_b1 = j1;
      C->resize(3);
      x_lo->resize(3);
      x_hi->resize(3);
      *C << 1, 1, 0;
      *x_lo << 0, 0, 0;
      *x_hi << 0, 0, std::numeric_limits<double>::infinity();
      break;
    case FrictionModel::BOX:
      *J_b0 = j0;
      *J_b1 = j1;
      C->resize(3);
      x_lo->resize(3);
      x_hi->resize(3);
      *C << 0, 0, 0;
      *x_lo << -1 * kBoxFrictionBound, -1 * kBoxFrictionBound, 0;
      *x_hi << kBoxFrictionBound, kBoxFrictionBound,
          std::numeric_limits<double>::infinity();
      break;
    default:
      Panic("Contact constraint encountered an unknown FrictionModel::%i", f_);
  }
}

void Contact::ComputeJDot(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const {
  ComputeJDot_NoFriction(Jdot_b0, Jdot_b1);
}

void Contact::ComputeJDot_NoFriction(MatrixXd* Jdot_b0,
                                     MatrixXd* Jdot_b1) const {
  Panic(
      "ODE time stepper does not need to compute JdotV for contact "
      "constraints. Other integrators do not yet support contacts.");
  // // R * c = x - p
  // MatrixXd Jdot_w0 = -1 * cg_.normal.transpose() *
  //                    CrossMat(b0_->w_g().cross(cg_.position - b0_->p()));
  // Jdot_b0 << MatrixXd::Zero(1, 3), Jdot_w0;
  // if (b1_ == nullptr) {
  //   Jdot_b1 << MatrixXd::Zero(1, 3), MatrixXd::Zero(1, 3);
  // } else {
  //   MatrixXd Jdot_w1 = cg_.normal.transpose() *
  //                      CrossMat(b1_->w_g().cross(cg_.position - b1_->p()));
  //   Jdot_b1 << MatrixXd::Zero(1, 3), Jdot_w1;
  // }
}

void Contact::ComputeJDot_InfiniteFriction(MatrixXd* Jdot_b0,
                                           MatrixXd* Jdot_b1) const {
  Panic(
      "ODE time stepper does not need to compute JdotV for contact "
      "constraints. Other integrators do not yet support contacts.");
}

void Contact::ComputeJDot_BoxFriction(MatrixXd* Jdot_b0,
                                      MatrixXd* Jdot_b1) const {}
void Contact::ComputeJ_CoulombPyramid(MatrixXd* J_b0, MatrixXd* J_b1) const {}
void Contact::ComputeJDot_CoulombPyramid(MatrixXd* Jdot_b0,
                                         MatrixXd* Jdot_b1) const {}

void Contact::Draw() const {
  DrawPoint(cg_.position);
  DrawLine(cg_.position, cg_.position + cg_.normal * 0.1);
}

std::string Contact::PrintInfo() const {
  std::ostringstream s;
  s << "Contact between components " << i0_ << " and " << i1_
    << ", @ position = \n"
    << cg_.position << "\nContact normal = \n"
    << cg_.normal << "\nContact depth = " << cg_.depth;
  return s.str();
}

std::ostream& operator<<(std::ostream* out, const Contact& c) {
  return *out << c.PrintInfo();
}
