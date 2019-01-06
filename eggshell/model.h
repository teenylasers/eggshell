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

// State of a rigid body
class Body {
 public:
  Body()
      : p_(Eigen::Vector3d::Zero()),
        v_(Eigen::Vector3d::Zero()),
        m_(0.0),
        R_(Eigen::Matrix3d::Identity()),
        w_(Eigen::Vector3d::Zero()),
        I_(Eigen::Matrix3d::Identity()) {}
  explicit Body(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                const Eigen::Matrix3d& R, const Eigen::Vector3d& w)
      : p_(p), v_(v), m_(1.0), R_(R), w_(w), I_(Eigen::Matrix3d::Identity()) {}
  explicit Body(const Eigen::Vector3d& p, const Eigen::Vector3d& v, double m,
                const Eigen::Matrix3d& R, const Eigen::Vector3d& w,
                const Eigen::Matrix3d& I)
      : p_(p), v_(v), m_(m), R_(R), w_(w), I_(I) {}
  const Eigen::Vector3d& p() const { return p_; };
  const Eigen::Vector3d& v() const { return v_; };
  const double m() const { return m_; };
  const Eigen::Matrix3d& R() const { return R_; };
  const Eigen::Vector3d w_b() const { return R_.transpose() * w_; };
  const Eigen::Vector3d& w_g() const { return w_; };
  const Eigen::Matrix3d& I_b() const { return I_; }
  const Eigen::Matrix3d I_g() const { return R_ * I_ * R_.transpose(); };
  void SetR(const Eigen::Matrix3d& R) { R_ = R; };
  void SetI(const Eigen::Matrix3d& I) { I_ = I; };

  void Draw() const;

 private:
  Eigen::Vector3d p_;  // position p of center of mass in global frame
  Eigen::Vector3d v_;  // linear velocity of center of mass v in global frame
  double m_;           // mass
  Eigen::Matrix3d R_;  // rotation matrix R in global frame
  Eigen::Vector3d w_;  // angular velocity w (omega) in global frame
  Eigen::Matrix3d I_;  // inertia tensor I in body frame.

  // TODO: default Body to a box with sides_ 0.3 for now
  Eigen::Vector3d sides_{0.3, 0.3, 0.3};
};

/*********************************************
 Various integrators of different goodness
**********************************************/

/////////////  Explicit Euler  ////////////

// Update rotation matrix using addition: R_i+1 = R_i + dt * w0 x R_i
Eigen::Matrix3d ExplicitEulerRotationMatrix_Addition(const Eigen::Matrix3d& R,
                                                     const Eigen::Vector3d& w0,
                                                     double dt);
// Update rotation matrix: R_i+1 = F(dt * w0) * R_i
Eigen::Matrix3d ExplicitEulerRotationMatrix(const Eigen::Matrix3d& R,
                                            const Eigen::Vector3d& w0,
                                            double dt);
Eigen::Matrix3d ExplicitEulerRotationMatrix(const Body& b, double dt);
Eigen::Matrix3d ExplicitEulerRotationMatrix_BodyFrameUpdate(const Body& b,
                                                            double dt);
// Update angular velocity: w_i+1 = w_i + dt * I^-1 * (tau - w_i x I w_i)
// Default, use global frame
Eigen::Vector3d ExplicitEulerAngularVelocity(
    const Body& b, double dt,
    const Eigen::Vector3d& tau = Eigen::Vector3d::Zero());
// Uses body frame, returns w in body frame. Transformation to global frame
// requires knowing updated R instead of b.R().
Eigen::Vector3d ExplicitEulerAngularVelocity_BodyFrameUpate(
    const Body& b, double dt,
    const Eigen::Vector3d& tau = Eigen::Vector3d::Zero());
// Update Body
// Default, use global frame. The difference between ExplicitEulerBodyRotation
// and ExplicitEulerBodyRotation_BodyFrameUpdate is whether w_i+1 updated in
// body frame is converted to global frame by R_i or R_i+1, respectively.
Body ExplicitEulerBodyRotation(
    const Body& b, double dt,
    const Eigen::Vector3d& tau = Eigen::Vector3d::Zero());
Body ExplicitEulerBodyRotation_BodyFrameUpdate(
    const Body& b, double dt,
    const Eigen::Vector3d& tau = Eigen::Vector3d::Zero());

//////////////  Linearized implicit midpoint  ////////////

// Update angular velocity:
// First in body frame,
// w_i+1 = w_i - dt * (alpha + beta) * (Identity - dt * beta * I^-1 *
// (CrossMat(I * w_i) - CrossMat(w_i) * I))^-1 * I^-1 * (w x I w_i)
// Then w_i+1 (global) = R_i * w_i+1 (body)
Eigen::Vector3d LIMAngularVelocity(
    const Body& b, double dt,
    const Eigen::Vector3d& tau = Eigen::Vector3d::Zero(), double alpha = 0.5,
    double beta = 0.5);
// R_i+1 = F(dt * alpha * w_i + dt * beta * w_i+1) * R_i, w_i+1 is calculated
// from LIMAngularVelocity()
Eigen::Matrix3d LIMRotationMatrix(const Body& b, double dt,
                                  const Eigen::Vector3d& w_update,
                                  double alpha = 0.5, double beta = 0.5);
// Update Body
Body LIMBodyRotation(const Body& b, double dt,
                     const Eigen::Vector3d& tau = Eigen::Vector3d::Zero(),
                     double alpha = 0.5, double beta = 0.5);
// TODO: what happens if we kept LIMAngularVelocity() in body frame, then
// translated to global frame in LIMBodyRotation using R_i+1, in the manner of
// ExplicitEulerBodyRotation_BodyFrameUpdate()?

//////////////  Conservation of energy  /////////////

// Calculate Body rotational kinetic energy
double GetRotationalKE(const Body& b);

#endif
