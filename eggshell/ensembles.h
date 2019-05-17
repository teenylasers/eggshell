// An ensemble is a collection of rigid bodies (Body or Ensemble) held together
// by Joints.

#ifndef __ENSEMBLES_H__
#define __ENSEMBLES_H__

#include <vector>

#include "Eigen/Dense"
#include "body.h"
#include "collision.h"
#include "joints.h"

// Base class.
class Ensemble {
 public:
  virtual ~Ensemble() = default;

  virtual void Init();

  virtual Eigen::VectorXd ComputeJointError() const;
  virtual Eigen::MatrixXd ComputeJ() const;

  // JDot is a sparse matrix, computing JDot then JDot * v is doing a lot of
  // multiplication with zero. Compute JDotV directly.
  virtual Eigen::VectorXd ComputeJDotV() const;

  // Sanity check the dimensions of ComputeError and ComputeJ results.
  // TODO: may not need these, ComputeError and ComputeJ concatenations should
  // be correct by construction.
  virtual bool CheckErrorDims(Eigen::VectorXd error) const = 0;
  virtual bool CheckJDims(Eigen::MatrixXd j) const = 0;

  // Sanity check the ensemble's initial conditions, before any time stepping
  // has occurred.
  virtual bool CheckInitialConditions() const;

  // Advance one time step with step size dt
  enum struct Integrator {
    EXPLICIT_EULER = 0,
    IMPLICIT_MIDPOINT,
    OPEN_DYNAMICS_ENGINE
  };
  virtual void Step(double dt, Integrator g = Integrator::EXPLICIT_EULER);

  virtual void Draw() const;  // Render in EggshellView

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
    std::shared_ptr<Joint> j;
    int b0;  // body0 index in components_
    int b1;  // body1 index in components_

    JointBodies(const std::shared_ptr<Joint> _j, int _b)
        : j(_j), b0(_b), b1(-1) {}
    JointBodies(const std::shared_ptr<Joint> _j, int _b0, int _b1)
        : j(_j), b0(_b0), b1(_b1) {}
  };

  // The Bodies that make up this ensemble
  std::vector<Body> components_;
  // Joints between Bodies or anchor to global frame. Use share_ptr because
  // Joint is the abstract base class.
  std::vector<JointBodies> joints_;
  // Conacts between Bodies or between Body and ground.
  std::vector<ContactGeometry> contacts_;

  Eigen::MatrixXd M_inverse_;
  Eigen::VectorXd external_force_torque_;

 private:
  void ConstructMassInertiaMatrixInverse();
  void InitializeExternalForceTorqueVector();  // Gravity is applied here

  // Create v vector, which contains p_dot and w for all bodies.
  Eigen::VectorXd GetCurrentVelocities();

  // Update p_dot and w in each Body in component_ using updated v (in global
  // frame).
  void UpdateComponentsVelocities(const Eigen::VectorXd& v);

  // Various versions of time steppers. StepVelocities() and StepPosition()
  // decide which ones to use.
  Eigen::VectorXd StepVelocities_ExplicitEuler(double dt,
                                               const Eigen::VectorXd& v);
  void StepPositions_ExplicitEuler(double dt, const Eigen::VectorXd& v);
  Eigen::VectorXd StepVelocities_ImplicitMidpoint(double dt,
                                                  const Eigen::VectorXd& v,
                                                  double alpha = 0.5,
                                                  double beta = 0.5);
  void StepPositions_ImplicitMidpoint(double dt, const Eigen::VectorXd& v,
                                      const Eigen::VectorXd& v_new,
                                      double alpha = 0.5, double beta = 0.5);
  Eigen::VectorXd StepVelocities_ODE(double dt, const Eigen::VectorXd& v,
                                     double error_reduction_param = 0.2);
  void StepPositions_ODE(double dt, const Eigen::VectorXd& v,
                         const Eigen::VectorXd& v_new);

  // Post-stabilization: calculate velocity correction for post-stabilization
  Eigen::VectorXd CalculateVelocityRelaxation(double step_scale);
};

class Chain : public Ensemble {
 public:
  Chain(int num_links, const Eigen::Vector3d& anchor_position);

  // TODO: this implementation assumes joint i is between component i and i+1.
  // The assumption is used by ComputeJ. Make general by adding integer label
  // to each component, std::vector<std::pair<std::vector<int>,
  // std::shared_ptr<Joint>>> joints_
  // Then, will be able to move ComputeJ() and ComputeJDot() to Ensemble class.
  // Eigen::MatrixXd ComputeJ() const override;
  // Eigen::MatrixXd ComputeJDot() const override;

  bool CheckErrorDims(Eigen::VectorXd) const override;
  bool CheckJDims(Eigen::MatrixXd) const override;

 private:
  void InitLinks(const Eigen::Vector3d& anchor_position);
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

  bool CheckErrorDims(Eigen::VectorXd) const override;
  bool CheckJDims(Eigen::MatrixXd) const override;
};

#endif
