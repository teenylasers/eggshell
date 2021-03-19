// An ensemble is a collection of rigid bodies (Body or Ensemble) held together
// by Joints.

#ifndef __ENSEMBLES_H__
#define __ENSEMBLES_H__

#include <vector>

#include "Eigen/Dense"
#include "body.h"
#include "collision.h"
#include "contact.h"
#include "joints.h"
#include "utils.h"

// Base class.
class Ensemble {
 public:
  virtual ~Ensemble() = default;

  virtual void Init();

  // When there are only joint constraints and if the integrator does not
  // require constraint type info for mixed constraint solver, e.g. explicit
  // Euler, then we can use ComputeJ() without C, x_lo, x_hi.
  virtual MatrixXd ComputeJ() const;
  // For mixed constraints, C = constraint type, true for equality, false for
  // inequality. For inequality constraints, {x_lo, x_hi} indicates lower and
  // higher x bound in Ax = b+w.
  virtual MatrixXd ComputeJ(ArrayXb* C, VectorXd* x_lo, VectorXd* x_hi) const;

  // Compute the position error.
  virtual VectorXd ComputePositionConstraintError() const;

  // JDot is a sparse matrix, computing JDot then JDot * v is doing a lot of
  // multiplication with zero. Therefore compute JDotV directly.
  virtual VectorXd ComputeJDotV() const;

  // Sanity check the dimensions of ComputeError and ComputeJ results.
  // TODO: may not need these, ComputeError and ComputeJ concatenations should
  // be correct by construction.
  virtual bool CheckErrorDims(VectorXd error) const = 0;

  // Sanity check the ensemble's initial conditions, before any time stepping
  // has occurred. Specifically, check that position constraint error is no more
  // than reasonably allowed numerical error.
  virtual bool CheckInitialConditions() const;

  // Advance one time step with step size dt
  enum struct Integrator {
    EXPLICIT_EULER = 0,
    OPEN_DYNAMICS_ENGINE,
    IMPLICIT_MIDPOINT
  };
  virtual void Step(double dt, Integrator g = Integrator::OPEN_DYNAMICS_ENGINE);

  // Find all contacts between Bodies or between Body and ground, clear and
  // update contacts_
  void UpdateContacts();

  // When Ensemble is first initialized, check for position errors, correct them
  // if any using StepPositionRelaxation().
  void InitStabilize();

  // Takes Ensemble's state after an integrator Step, apply stabilization to
  // bring the state closer to or back to the constraint manifold in both
  // position and velocity.
  void PostStabilize(int max_steps = 500);

  // Render in EggshellView
  virtual void Draw() const;

  // TODO:
  // Check conservation of energy and impact, is the following even true?
  //
  // Check that the rotational kinetic energy is conserved. Linear kinetic
  // energy changes due to gravity, thus not checked in this function.
  bool CheckConservationOfEnergy();

 protected:
  // Number of Bodies that make up this ensemble
  int n_;

  // The Bodies that make up this ensemble. Use shared_ptr because Body is the
  // abstract base class.
  std::vector<std::shared_ptr<Body>> components_;
  // Joints between Bodies or anchor to global frame.
  std::vector<std::shared_ptr<Joint>> joints_;
  // Contacts between Bodies or between Body and ground.
  std::vector<std::shared_ptr<Contact>> contacts_;

  MatrixXd M_inverse_;
  VectorXd external_force_torque_;

 private:
  double total_rotational_ke_ = std::numeric_limits<double>::infinity();

  void ConstructMassInertiaMatrixInverse();
  void InitializeExternalForceTorqueVector();  // Gravity is applied here

  // Create and return the systems v vector, [p_dot_0; w_0; p_dot_1; w_1' ...]
  const VectorXd GetVelocities() const;

  // Update p_dot and w in each Body in component_ using updated v (in global
  // frame).
  void UpdateComponentsVelocities(const VectorXd& v);

  // Check whether J is singular or rank-deficient. For now, return false if
  // definitely singular and need to readdress the problem set-up.
  bool CheckJ(const MatrixXd& J) const;

  // Compute JDotV due to 1. joint constraints, 2. contact constraints
  VectorXd ComputeJDotV_Joints() const;
  VectorXd ComputeJDotV_Contacts() const;

  ////// Compute v_dot for stepping velocity //////
  // When there is only equality constraints, i.e. no contact constraints, used
  // when Integrator::EXPLICIT_EULER.
  VectorXd ComputeVDot(const MatrixXd& J, const VectorXd& rhs) const;
  // For mixed equality and inequality constraints, general case.
  VectorXd ComputeVDot(const MatrixXd& J, const VectorXd& rhs, const ArrayXb& C,
                       const VectorXd& x_lo, const VectorXd& x_hi) const;

  // Various versions of time steppers. StepVelocities() and StepPosition()
  // decide which ones to use.
  VectorXd StepVelocities_ExplicitEuler(double dt, const VectorXd& v);
  void StepPositions_ExplicitEuler(double dt, const VectorXd& v);
  VectorXd StepVelocities_ImplicitMidpoint(double dt, const VectorXd& v,
                                           double alpha = 0.5,
                                           double beta = 0.5);
  void StepPositions_ImplicitMidpoint(double dt, const VectorXd& v,
                                      const VectorXd& v_new, double alpha = 0.5,
                                      double beta = 0.5);
  VectorXd StepVelocities_ODE(double dt, const VectorXd& v,
                              double error_reduction_param = 0.2);
  void StepPositions_ODE(double dt, const VectorXd& v, const VectorXd& v_new);

  ////// Helper functions to InitStablize() and PostStablize(). //////

  // Stabilize position only. Bring the ensemble into a legal configuration.
  void StepPositionRelaxation(double dt, double step_scale = 0.2);
  // Calculate velocity correction for post-stabilization
  VectorXd CalculateVelocityRelaxation(double step_scale) const;
  // Update both position and velocity using velocity relaxation results.
  void StepPostStabilization(double dt, double step_scale = 0.2);
};

class Chain : public Ensemble {
 public:
  Chain(int num_links, const Vector3d& anchor_position);
  bool CheckErrorDims(VectorXd error) const override;

 private:
  void InitLinks(const Vector3d& anchor_position);
  void InitJoints();
  // TODO: Default sets component_[0] to be the anchor, make it customizable?
  void SetAnchor();
};

// TODO: handle mix of chain and cairn in a general way
class Cairn : public Ensemble {
 public:
  Cairn(int num_rocks, const std::array<double, 2>& x_bound,
        const std::array<double, 2>& y_bound,
        const std::array<double, 2>& z_bound);

  bool CheckErrorDims(VectorXd error) const override;

 private:
  const double max_init_v_ = 1;
  const double max_init_w_ = 1;
};

#endif
