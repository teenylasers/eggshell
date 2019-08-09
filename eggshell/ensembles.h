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

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
typedef Eigen::Array<bool, Eigen::Dynamic, 1> ArrayXb;

// Base class.
class Ensemble {
 public:
  virtual ~Ensemble() = default;

  virtual void Init();

  // Compute constraints Jacobian.
  // When only joint constraints, can use ComputeJ() without C, x_lo, x_hi.
  virtual MatrixXd ComputeJ() const;
  // For mixed constraints, C = constraint type, true for equality, false for
  // inequality. For inequality constraints, {x_lo, x_hi} indicates lower and
  // higher x bound in Ax = b+w.
  virtual MatrixXd ComputeJ(ArrayXb& C, VectorXd& x_lo, VectorXd& x_hi) const;
  virtual VectorXd ComputePositionConstraintError() const;

  // JDot is a sparse matrix, computing JDot then JDot * v is doing a lot of
  // multiplication with zero. Compute JDotV directly.
  virtual VectorXd ComputeJDotV() const;

  // Sanity check the dimensions of ComputeError and ComputeJ results.
  // TODO: may not need these, ComputeError and ComputeJ concatenations should
  // be correct by construction.
  virtual bool CheckErrorDims(VectorXd error) const = 0;

  // Sanity check the ensemble's initial conditions, before any time stepping
  // has occurred.
  virtual bool CheckInitialConditions() const;

  // Advance one time step with step size dt
  enum struct Integrator {
    EXPLICIT_EULER = 0,
    OPEN_DYNAMICS_ENGINE,
    IMPLICIT_MIDPOINT
  };
  virtual void Step(double dt, Integrator g = Integrator::OPEN_DYNAMICS_ENGINE);

  // Position and/or velocity stablization
  void Stablize(const int max_steps = 500);

  // Render in EggshellView
  virtual void Draw() const;

  // Post-stabilization: takes Ensemble's state after an integrator Step, apply
  // stabilization to bring the state closer to or back to the constraint
  // manifold.
  void StepPositionRelaxation(double dt, double step_scale = 0.2);

  // Apply naive post-stabilization
  void StepPostStabilization(double dt, double step_scale = 0.2);

 protected:
  // Number of Bodies that make up this ensemble
  int n_;

  struct JointBodies {
    // Use shared_ptr because Joint is the abstract base class.
    std::shared_ptr<Joint> j;
    int b0;  // body0 index in components_
    int b1;  // body1 index in components_

    JointBodies(const std::shared_ptr<Joint> _j, int _b)
        : j(_j), b0(_b), b1(-1) {}
    JointBodies(const std::shared_ptr<Joint> _j, int _b0, int _b1)
        : j(_j), b0(_b0), b1(_b1) {}
  };

  struct ContactingBodies {
    Contact c;
    int b0;  // body0 index in components_
    int b1;  // body1 index in components_

    ContactingBodies(const Contact _c, int _b) : c(_c), b0(_b), b1(-1) {}
    ContactingBodies(const Contact _c, int _b0, int _b1)
        : c(_c), b0(_b0), b1(_b1) {}
  };

  // The Bodies that make up this ensemble. Use shared_ptr because Body is the
  // abstract base class.
  std::vector<std::shared_ptr<Body>> components_;
  // Joints between Bodies or anchor to global frame.
  std::vector<JointBodies> joints_;
  // Contacts between Bodies or between Body and ground.
  // TODO: need to store contacts_ or just use them to create J and JDot on the
  // fly and discard
  std::vector<ContactingBodies> contacts_;

  MatrixXd M_inverse_;
  VectorXd external_force_torque_;

 private:
  void ConstructMassInertiaMatrixInverse();
  void InitializeExternalForceTorqueVector();  // Gravity is applied here

  // Create v vector, which contains p_dot and w for all bodies.
  VectorXd GetCurrentVelocities();

  // Update p_dot and w in each Body in component_ using updated v (in global
  // frame).
  void UpdateComponentsVelocities(const VectorXd& v);

  // Find all contacts between Bodies or between Body and ground, clear and
  // update contacts_
  void UpdateContacts();

  // TODO: joint and contact constraints can be treated more or less the same
  // way.
  // Compute J due to 1. joint constraints, 2. contact constraints
  MatrixXd ComputeJ_Joints() const;
  MatrixXd ComputeJ_Contacts(ArrayXb& C, VectorXd& x_lo, VectorXd& x_hi) const;
  // TODO: Pass argument to capture inequality constraints to separately treat
  // friction and non-penetrating contact. Is this the elegant thing to do?

  // Check whether J is singular or rank-deficient. For now, return false if
  // definitely singular and need to readdress the problem set-up.
  bool CheckJ(const MatrixXd& J) const;

  // Compute JDotV due to 1. joint constraints, 2. contact constraints
  VectorXd ComputeJDotV_Joints() const;
  VectorXd ComputeJDotV_Contacts() const;

  // Compute v_dot for stepping velocity
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

  // Post-stabilization: calculate velocity correction for post-stabilization
  VectorXd CalculateVelocityRelaxation(double step_scale);
};

class Chain : public Ensemble {
 public:
  Chain(int num_links, const Vector3d& anchor_position);

  // TODO: this implementation assumes joint i is between component i and i+1.
  // The assumption is used by ComputeJ. Make general by adding integer label
  // to each component, std::vector<std::pair<std::vector<int>,
  // std::shared_ptr<Joint>>> joints_
  // Then, will be able to move ComputeJ() and ComputeJDot() to Ensemble class.
  // MatrixXd ComputeJ() const override;
  // MatrixXd ComputeJDot() const override;

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
