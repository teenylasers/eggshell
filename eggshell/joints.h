// Library of all joint types

#ifndef __JOINTS_H__
#define __JOINTS_H__

#include "Eigen/Dense"
#include "constraints.h"
#include "model.h"
#include "utils.h"

// Base class
class Joint : public Constraint {
 public:
  // Joint that anchors b0 to c1 in global frame.
  explicit Joint(const std::shared_ptr<Body> b0, int i0, const Vector3d& c0,
                 const Vector3d& c1)
      : Constraint(b0, i0, nullptr, -1), c0_(c0), c1_(c1) {}
  // Joint between 2 bodies
  explicit Joint(const std::shared_ptr<Body> b0, int i0, const Vector3d& c0,
                 const std::shared_ptr<Body> b1, int i1, const Vector3d& c1)
      : Constraint(b0, i0, b1, i1), c0_(c0), c1_(c1) {}

 protected:
  // Joint locations for components b0_ and b1_, c0_ and c1_ are in the
  // components' respective local frame.
  Vector3d c0_;
  Vector3d c1_;
};

// A ball and socket joint
class BallAndSocketJoint : public Joint {
 public:
  // Joint that anchors b0 to c1 in global frame.
  explicit BallAndSocketJoint(const std::shared_ptr<Body> b0, int i0,
                              const Vector3d& c0, const Vector3d& c1)
      : Joint(b0, i0, c0, c1) {}
  explicit BallAndSocketJoint(const std::shared_ptr<Body> b0, int i0,
                              const Vector3d& c0,
                              const std::shared_ptr<Body> b1, int i1,
                              const Vector3d& c1)
      : Joint(b0, i0, c0, b1, i1, c1) {}
  VectorXd ComputeError() const override;
  void ComputeJ(MatrixXd* J_b0, MatrixXd* J_b1, ArrayXb* constraint_type,
                VectorXd* constraint_lo,
                VectorXd* constraint_hi) const override;
  void ComputeJDot(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const override;

  void Draw() const override;
};

#endif
