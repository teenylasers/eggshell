#ifndef __CONTACTS_H__
#define __CONTACTS_H__

#include <Eigen/Dense>
#include <vector>

#include "body.h"
#include "collision.h"
#include "util.h"

class Contact {
 public:
  explicit Contact(const Body* b, const ContactGeometry& cg)
      : b0_(nullptr), b1_(b), cg_(cg), ci_(CollisionInfo()) {}
  explicit Contact(const Body* b0, const Body* b1, const ContactGeometry& cg,
                   const CollisionInfo& ci)
      : b0_(b0), b1_(b1), cg_(cg), ci_(ci) {}

  enum struct FrictionModel {
    NO_FRICTION,
    INFINITE,
    BOX,
    COULOMB_PYRAMID,
  };

  VectorXd ComputeError() const;

  // TODO: should contact and joint all be part of a constraint base class?
  void ComputeJ(MatrixXd* J_b0, MatrixXd* J_b1, ArrayXb* constraint_type,
                VectorXd* constraint_lo, VectorXd* constraint_hi) const;
  void ComputeJDot(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const;

  void Draw() const;

  std::string PrintInfo() const;

 private:
  const Body* b0_;
  const Body* b1_;
  const ContactGeometry cg_;
  const CollisionInfo ci_;

  const FrictionModel f_ = FrictionModel::NO_FRICTION;

  //==========================================================================
  // Different implementations of ComputeError, ComputeJ and ComputeJDot, varies
  // in friction handling.

  void ComputeJDot_NoFriction(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const;
  void ComputeJDot_InfiniteFriction(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const;

  // Box friction with dynamic limits
  void ComputeJ_BoxFriction(MatrixXd* J_b0, MatrixXd* J_b1) const;
  void ComputeJDot_BoxFriction(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const;
  // Models Coulomb friction cone as an N-sided pyramid.
  void ComputeJ_CoulombPyramid(MatrixXd* J_b0, MatrixXd* J_b1) const;
  void ComputeJDot_CoulombPyramid(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const;
};

std::ostream& operator<<(std::ostream* out, const Contact& c);

#endif
