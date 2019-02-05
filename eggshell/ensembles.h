// An ensemble is a collection of rigid bodies (Body or Ensemble) held together
// by Joints.

#ifndef __ENSEMBLES_H__
#define __ENSEMBLES_H__

#include <vector>

#include "Eigen/Dense"
#include "body.h"
#include "joints.h"

// Base class.
class Ensemble {
 public:
  virtual ~Ensemble() = default;

  virtual void Init();
  virtual void SetAnchor() = 0;

  virtual Eigen::VectorXd ComputeJointError() const;
  virtual Eigen::MatrixXd ComputeJ() const = 0;
  virtual Eigen::MatrixXd ComputeJDot() const = 0;

  // Sanity check the dimensions of ComputeError and ComputeJ results.
  // TODO: may not need these, ComputeError and ComputeJ concatenations should
  // be correct by construction.
  virtual bool CheckErrorDims(Eigen::VectorXd error) const = 0;
  virtual bool CheckJDims(Eigen::MatrixXd j) const = 0;

  // Sanity check the ensemble's initial conditions, before any time stepping
  // has occurred.
  virtual bool CheckInitialConditions() const;

  // Advance one time step with step size dt
  enum struct Integrator { EXPLICIT_EULER = 0, LINEARIZED_IMPLICIT_MIDPOINT };
  virtual void Step(double dt, Integrator g = Integrator::EXPLICIT_EULER);

  virtual void Draw() const = 0;  // Render in EggshellView

 protected:
  int n_;                         // Number of Bodies that make up this ensemble
  std::vector<Body> components_;  // The Bodies that make up this ensemble
  std::vector<std::shared_ptr<Joint>> joints_;  // Joints between Bodies.
  // Anchor a component of this ensemble using a one-sided Joint.
  std::shared_ptr<Joint> anchor_;

  Eigen::MatrixXd M_inverse_;
  Eigen::VectorXd external_force_torque_;

 private:
  void ConstructMassInertiaMatrixInverse();
  // TODO: currently defaulting component 0 to be the anchor, i.e. it does not
  // experience any external force including gravity.
  // TODO: add support for non-zero external force and external torque.
  void InitializeExternalForceTorqueVector();

  // Create v vector, which contains p_dot and w for all bodies.
  Eigen::VectorXd GetCurrentVelocities();

  // Update p_dot and w in each Body in component_ using updated v (in global
  // frame).
  void UpdateComponentsVelocities(const Eigen::VectorXd& v);

  // Various versions of time steppers. Step() decided which ones to use.
  Eigen::VectorXd StepVelocities_ExplicitEuler(double dt,
                                               const Eigen::VectorXd& v);
  void StepPositions_ExplicitEuler(double dt, const Eigen::VectorXd& v);
  Eigen::VectorXd StepVelocities_LIM(double dt, const Eigen::VectorXd& v,
                                     double alpha = 0.5, double beta = 0.5);
  void StepPositions_LIM(double dt, const Eigen::VectorXd& v,
                         const Eigen::VectorXd& v_new, double alpha = 0.5,
                         double beta = 0.5);
};

class Chain : public Ensemble {
 public:
  Chain(int num_links);

  void SetAnchor() override;

  bool CheckErrorDims(Eigen::VectorXd) const override;

  // TODO: this implementation assumes joint i is between component i and i+1.
  // The assumption is used by ComputeJ. Make  general by adding integer label
  // to each component, std::vector<std::pair<std::vector<int>,
  // std::shared_ptr<Joint>>> joints_
  // Then, will be able to move ComputeJ() and ComputeJDot() to Ensemble class.
  Eigen::MatrixXd ComputeJ() const override;
  Eigen::MatrixXd ComputeJDot() const override;

  bool CheckJDims(Eigen::MatrixXd) const override;

  void Draw() const override;

 private:
  void InitLinks();
  // TODO: Default sets component_[0] to be an anchor. Add SetAnchor() to
  // Ensemble.
  void InitJoints();
};

#endif
