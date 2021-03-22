// Constraints, base class for joint constraints and contact constraints.

#ifndef __CONSTRAINTS_H__
#define __CONSTRAINTS_H__

#include <memory>

#include "Eigen/Dense"
#include "body.h"
#include "glog/logging.h"
#include "model.h"
#include "utils.h"

class Constraint {
 public:
  explicit Constraint(const std::shared_ptr<Body> b0, int i0,
                      const std::shared_ptr<Body> b1, int i1)
      : i0_(i0), i1_(i1), b0_(b0), b1_(b1) {}
  virtual ~Constraint() = default;

  virtual VectorXd ComputeError() const = 0;
  virtual void ComputeJ(MatrixXd* J_b0, MatrixXd* J_b1,
                        ArrayXb* constraint_type, VectorXd* constraint_lo,
                        VectorXd* constraint_hi) const = 0;
  virtual void ComputeJDot(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const = 0;

  // Visualize the joint implementation in EggshellView.
  virtual void Draw() const { LOG(ERROR) << "Constraint.Draw()"; };

  // Return a string description of this constraint.
  virtual std::string PrintInfo() const {
    LOG(ERROR) << "Constraint.PrintInfo()";
    return "Constraint.PrintInfo()";
  };

  // Get the global frame constraint position.
  virtual Vector3d GetConstraintPosition() const = 0;

  // The indices for b0_ and b1_ in the components_ list that contains this
  // Constraint.
  int i0_ = -1;
  int i1_ = -1;

 protected:
  // Components that are subject to this Constraint.
  const std::shared_ptr<Body> b0_;
  const std::shared_ptr<Body> b1_;
};

#endif
