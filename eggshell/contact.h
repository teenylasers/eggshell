#ifndef __CONTACTS_H__
#define __CONTACTS_H__

#include <Eigen/Dense>
#include <vector>

#include "collision.h"
#include "constraints.h"
#include "util.h"

class Contact : public Constraint {
 public:
  explicit Contact(const std::shared_ptr<Body> b, int index,
                   const ContactGeometry& cg)
      : Constraint(nullptr, -1, b, index), cg_(cg), ci_(CollisionInfo()) {}
  explicit Contact(const std::shared_ptr<Body> b0, int i0,
                   const std::shared_ptr<Body> b1, int i1,
                   const ContactGeometry& cg, const CollisionInfo& ci)
      : Constraint(b0, i0, b1, i1), cg_(cg), ci_(ci) {}

  enum struct FrictionModel {
    NO_FRICTION,
    INFINITE,
    BOX,
    COULOMB_PYRAMID,
  };

  VectorXd ComputeError() const override;
  void ComputeJ(MatrixXd* J_b0, MatrixXd* J_b1, ArrayXb* constraint_type,
                VectorXd* constraint_lo,
                VectorXd* constraint_hi) const override;
  void ComputeJDot(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const override;

  void Draw() const override;

  std::string PrintInfo() const override;

 private:
  const ContactGeometry cg_;
  const CollisionInfo ci_;
  const FrictionModel f_ = FrictionModel::NO_FRICTION;

  //==========================================================================
  // Different implementations of ComputeError, ComputeJ and ComputeJDot, varies
  // in friction handling.

  void ComputeJDot_NoFriction(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const;
  void ComputeJDot_InfiniteFriction(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const;
  void ComputeJDot_BoxFriction(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const;

  // Models Coulomb friction cone as an N-sided pyramid.
  void ComputeJ_CoulombPyramid(MatrixXd* J_b0, MatrixXd* J_b1) const;
  void ComputeJDot_CoulombPyramid(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const;
};

std::ostream& operator<<(std::ostream* out, const Contact& c);

#endif
