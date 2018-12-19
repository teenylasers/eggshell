#ifndef __MODEL_H__
#define __MODEL_H__

#include "Eigen/Dense"

// This is called one time at the start, to initialize things.
void SimulationInitialization();

// This is called to advance the simulation by one time step.
void SimulationStep();

// SimulationStep() can call these functions to draw output to the shell.

void DrawSphere(const Eigen::Vector3d& center, const Eigen::Matrix3d& rotation,
                double radius);

void DrawBox(const Eigen::Vector3d& center, const Eigen::Matrix3d& rotation,
             const Eigen::Vector3d& side_lengths);

void DrawCapsule(const Eigen::Vector3d& center, const Eigen::Matrix3d& rotation,
                 double radius, double length);

void DrawPoint(const Eigen::Vector3d& position);

void DrawLine(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);

// SimulationInitialization() or SimulationStep() can call these functions to
// indicate error conditions. Their arguments are the same as printf().

// A fatal error: print the error message and stop execution
void Panic(const char* message, ...);

// Generate the cross-product matrix for a vector a, so that a x b = a_bar
// b.
Eigen::Matrix3d CrossMat(const Eigen::Vector3d& a);

// Convert angular velocity w to equivalent quaternion
// q = [cos(|w|/2) 1/2*sinc(|w|/2) * w]
Eigen::Quaterniond WtoQ(const Eigen::Vector3d& w, double dt);

// Generate a random rotation matrix.
Eigen::Matrix3d RandomRotation();
Eigen::Matrix3d RandomRotationViaQuaternion();
Eigen::Matrix3d RandomRotationViaGramSchmidt();
Eigen::Matrix3d GramSchmidt(const Eigen::Matrix3d& m);

// Rotational state of a rigid body
class Body {
 public:
  Body()
      : R_(Eigen::Matrix3d::Identity()),
        w_(Eigen::Vector3d::Zero()),
        I_(Eigen::Matrix3d::Identity()) {}
  explicit Body(const Eigen::Matrix3d& R, const Eigen::Vector3d& w)
      : R_(R), w_(w), I_(Eigen::Matrix3d::Identity()) {}
  explicit Body(const Eigen::Matrix3d& R, const Eigen::Vector3d& w,
                const Eigen::Matrix3d& I)
      : R_(R), w_(w), I_(I) {}
  const Eigen::Matrix3d& R() const { return R_; };
  const Eigen::Vector3d w_b() const { return R_.transpose() * w_; };
  const Eigen::Vector3d& w_g() const { return w_; };
  const Eigen::Matrix3d& I_b() const { return I_; }
  const Eigen::Matrix3d I_g() const { return R_ * I_ * R_.transpose(); };
  void SetR(const Eigen::Matrix3d& R) { R_ = R; };
  void SetI(const Eigen::Matrix3d& I) { I_ = I; };

 private:
  Eigen::Matrix3d R_;  // rotation matrix R in global frame
  Eigen::Vector3d w_;  // angular velocity w (omega) in global frame
  Eigen::Matrix3d I_;  // inertia tensor I in body frame.
};

// Various integrators of different goodness
//
// Explicit Euler update on rotation matrix using addition:
// R_i+1 = R_i + dt * w0 x R_i
Eigen::Matrix3d ExplicitEulerRotationMatrix_Addition(const Eigen::Matrix3d& R,
                                                     const Eigen::Vector3d& w0,
                                                     double dt);
// Explicit Euler update on rotation matrix: R_i+1 = F(dt * w0) * R_i
Eigen::Matrix3d ExplicitEulerRotationMatrix(const Eigen::Matrix3d& R,
                                            const Eigen::Vector3d& w0,
                                            double dt);
Eigen::Matrix3d ExplicitEulerRotationMatrix(const Body& b, double dt);
Eigen::Matrix3d ExplicitEulerRotationMatrix_BodyFrameUpdate(const Body& b,
                                                            double dt);
// Explicit Euler update on angular velocity:
// w_i+1 = w_i + dt * I^-1 * (tau - w x I w)
// Default, use global frame
Eigen::Vector3d ExplicitEulerAngularVelocity(
    const Body& b, double dt,
    const Eigen::Vector3d& tau = Eigen::Vector3d::Zero());
// Uses body frame, returns w in body frame. Transformation to global frame
// requires knowing updated R instead of b.R().
Eigen::Vector3d ExplicitEulerAngularVelocity_BodyFrameUpate(
    const Body& b, double dt,
    const Eigen::Vector3d& tau = Eigen::Vector3d::Zero());
// Explicit Euler update on Body
// Default: use global frame
Body ExplicitEulerBodyRotation(
    const Body& b, double dt,
    const Eigen::Vector3d& tau = Eigen::Vector3d::Zero());
Body ExplicitEulerBodyRotation_BodyFrameUpdate(
    const Body& b, double dt,
    const Eigen::Vector3d& tau = Eigen::Vector3d::Zero());

// Calculate Body rotational kinetic energy
double GetRotationalKE(const Body& b);

#endif
