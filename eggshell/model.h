#ifndef __MODEL_H__
#define __MODEL_H__

#include "Eigen/Dense"
#include "body.h"

// This is called one time at the start, to initialize things.
void SimulationInitialization();

// This is called to advance the simulation by one time step. Return true if the
// simulation is to continue or false otherwise.
bool SimulationStep();

// SimulationStep() can call these functions to draw output to the shell.

void DrawSphere(const Eigen::Vector3d &center, const Eigen::Matrix3d &rotation,
                double radius, int color = 0xffffff);

void DrawBox(const Eigen::Vector3d &center,
             const Eigen::Matrix3d &rotation,
             const Eigen::Vector3d &side_lengths, int color = 0xffffff);

void DrawCapsule(const Eigen::Vector3d &center,
                 const Eigen::Matrix3d &rotation,
                 double radius, double length, int color = 0xffffff);

void DrawPoint(const Eigen::Vector3d &position, int color = 0xffff00);

void DrawLine(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2,
              int color = 0xffff00);

// Make plots from columns of x.
void EggPlot(const Eigen::VectorXd &x, const Eigen::MatrixXd &data,
             const char *title = "");

// SimulationInitialization() or SimulationStep() can call these functions to
// indicate error conditions. Their arguments are the same as printf().

// A fatal error: print the error message and stop execution
void Panic(const char* message, ...);


/*****************************************************
 Various SimulationInitialization() and SimulationSteps()
******************************************************/

// Test case: single body integrators on boxes
void SimulationInitialization_BoxTests();
bool SimulationStep_BoxTests();

// Test case: a hanging chain with ball-and-socket joints
void SimulationInitialization_HangingChain();
bool SimulationStep_HangingChain();

// Test case: drop a pile of cubes
void SimulationInitialization_Cairn();
bool SimulationStep_Cairn();

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

//////////////  Error checking and visualization  /////////////

// TODO:
// 1. check KE and total energy progression
// 2. check joint error progression
// 3. check time change in joint error

#endif
