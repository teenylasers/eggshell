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

  // Advance one time step with step size dt
  enum Integrator { EXPLICIT_EULER, LINEARIZED_IMPLICIT_MIDPOINT };
  virtual void Step(double dt, Integrator g) = 0;

  virtual void Draw() const = 0;  // Render in EggshellView

 protected:
  int n_;                         // number of Bodies that make up this ensemble
  std::vector<Body> components_;  // the Bodies that make up this ensemble
  std::vector<std::shared_ptr<Joint>> joints_;  // Joints between Bodies.
  // TODO: cannot use unique_ptr here, why?
};

class Chain : public Ensemble {
 public:
  Chain(int num_links);

  void Step(double dt, Integrator g) override;
  void Draw() const override;

 private:
  void InitLinks();
  void InitJoints();

  // void Step_ExplicitEuler(double dt);
  // void Step_LinearizedImplicitMidpoint(double dt);
};

#endif
