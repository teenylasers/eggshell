
#ifndef __CONTACTS_H__
#define __CONTACTS_H__

#include <Eigen/Dense>
#include <vector>

#include "body.h"
#include "collision.h"
#include "util.h"

class Contact {
 public:
  explicit Contact(const Body* b0, const ContactGeometry& cg)
      : b0_(b0), b1_(nullptr), cg_(cg), ci_(CollisionInfo()) {}
  explicit Contact(const Body* b0, const Body* b1, const ContactGeometry& cg,
                   const CollisionInfo& ci)
      : b0_(b0), b1_(b1), cg_(cg), ci_(ci) {}

  enum struct FrictionModel {
    NO_FRICTION,
    INFINITE_W_CFM,
    BOX_FRICTION,
    COULOMB_PYRAMID,
  };

  Eigen::VectorXd ComputeError() const;

  // TODO: should contact and joint all be part of a constraint base class?
  void ComputeJ(Eigen::MatrixXd& J_b0, Eigen::MatrixXd& J_b1,
                Eigen::Array<bool, Eigen::Dynamic, 1>& constraint_type,
                Eigen::VectorXd& constraint_lo, Eigen::VectorXd& constraint_hi) const;
  void ComputeJDot(Eigen::MatrixXd& Jdot_b0, Eigen::MatrixXd& Jdot_b1) const;

  void Draw() const;

  std::string PrintInfo() const;

 private:
  const Body* b0_;
  const Body* b1_;
  const ContactGeometry cg_;
  const CollisionInfo ci_;

  const FrictionModel f_ = FrictionModel::INFINITE_W_CFM;

  //==========================================================================
  // Different implementations of ComputeError, ComputeJ and ComputeJDot, varies
  // in friction handling.
  //
  Eigen::VectorXd ComputeError_NoFriction() const;
  void ComputeJ_NoFriction(Eigen::MatrixXd& J_b0, Eigen::MatrixXd& J_b1) const;
  void ComputeJDot_NoFriction(Eigen::MatrixXd& Jdot_b0,
                              Eigen::MatrixXd& Jdot_b1) const;
  // Infinite friction and add constraint force mixing
  Eigen::VectorXd ComputeError_ConstraintForceMixing() const;
  void ComputeJ_ConstraintForceMixing(Eigen::MatrixXd& J_b0,
                                      Eigen::MatrixXd& J_b1,
                                      Eigen::Array<bool, Eigen::Dynamic, 1>& C,
                                      Eigen::VectorXd& x_lo,
                                      Eigen::VectorXd& x_hi) const;
  void ComputeJDot_ConstraintForceMixing(Eigen::MatrixXd& Jdot_b0,
                                         Eigen::MatrixXd& Jdot_b1) const;
  // Box friction with dynamic limits
  void ComputeJ_BoxFriction(Eigen::MatrixXd& J_b0, Eigen::MatrixXd& J_b1) const;
  void ComputeJDot_BoxFriction(Eigen::MatrixXd& Jdot_b0,
                               Eigen::MatrixXd& Jdot_b1) const;
  // Models Coulomb friction cone as an N-sided pyramid.
  void ComputeJ_CoulombPyramid(Eigen::MatrixXd& J_b0,
                               Eigen::MatrixXd& J_b1) const;
  void ComputeJDot_CoulombPyramid(Eigen::MatrixXd& Jdot_b0,
                                  Eigen::MatrixXd& Jdot_b1) const;

  //==========================================================================
  // Helper functions for error checking and experimentation
  void CheckJ_ConstraintForceMixing() const;
};

std::ostream& operator<<(std::ostream& out, const Contact& c);

#endif
