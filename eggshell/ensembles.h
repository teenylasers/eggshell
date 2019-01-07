// An ensemble is a collection of rigid bodies (Body or Ensemble) held together
// by Joints.

#ifndef __ENSEMBLES_H__
#define __ENSEMBLES_H__

#include <vector>

#include "Eigen/Dense"
#include "joints.h"

// Base class.
class Ensemble {
 public:
  virtual ~Ensemble() = default;

  virtual Eigen::VectorXd ComputeJointError() const;
  virtual Eigen::MatrixXd ComputeJ() const;
  // Sanity check the dimensions of ComputeError and ComputeJ results.
  // TODO: may not need these, ComputeError and ComputeJ concatenations should
  // be correct by construction.
  virtual bool CheckErrorDims(Eigen::VectorXd error) const = 0;
  virtual bool CheckJDims(Eigen::MatrixXd j) const = 0;

  // Sanity check the ensemble's initial conditions, before any time stepping
  // has occurred.
  virtual bool CheckInitialConditions() const;

  // Advance one time step with step size dt
  enum Integrator { EXPLICIT_EULER, LINEARIZED_IMPLICIT_MIDPOINT };
  virtual void Step(double dt, Integrator g) = 0;

  virtual void Draw() const = 0;  // Render in EggshellView

 protected:
  int n_;                         // number of Bodies that make up this ensemble
  std::vector<Body> components_;  // the Bodies that make up this ensemble
  std::vector<std::shared_ptr<Joint>> joints_;  // Joints between Bodies.
};

class Chain : public Ensemble {
 public:
  Chain(int num_links);

  bool CheckErrorDims(Eigen::VectorXd) const override;
  bool CheckJDims(Eigen::MatrixXd) const override;

  void Step(double dt, Integrator g) override;
  void Draw() const override;

 private:
  void InitLinks();
  void InitJoints();

  // void Step_ExplicitEuler(double dt);
  // void Step_LinearizedImplicitMidpoint(double dt);
};

#endif
