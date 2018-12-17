#ifndef __MODEL_H__
#define __MODEL_H__

#include "Eigen/Dense"

// This is called one time at the start, to initialize things.
void SimulationInitialization();

// This is called to advance the simulation by one time step.
void SimulationStep();

// SimulationStep() can call these functions to draw output to the shell.

void DrawSphere(const Eigen::Vector3d &center, const Eigen::Matrix3d &rotation,
                double radius);

void DrawBox(const Eigen::Vector3d &center, const Eigen::Matrix3d &rotation,
             const Eigen::Vector3d &side_lengths);

void DrawCapsule(const Eigen::Vector3d &center, const Eigen::Matrix3d &rotation,
                 double radius, double length);

void DrawPoint(const Eigen::Vector3d &position);

void DrawLine(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2);

// SimulationInitialization() or SimulationStep() can call these functions to
// indicate error conditions. Their arguments are the same as printf().

// A fatal error: print the error message and stop execution
void Panic(const char *message, ...);

// Generate the cross-product matrix for a vector a, so that a x b = a_bar b.
Eigen::Matrix3d CrossMat(const Eigen::Vector3d &a);

// Generate a random rotation matrix.
Eigen::Matrix3d RandomRotation();
Eigen::Matrix3d RandomRotationViaQuaternion();
Eigen::Matrix3d RandomRotationViaGramSchmidt();
Eigen::Matrix3d GramSchmidt(const Eigen::Matrix3d &m);

// Various integrators of different goodness
// Explicit Euler update on rotation matrix:  R_i+1 = R_i + dt * omega0 x R_i
Eigen::Matrix3d ExplicitEulerRotationUpdate(const Eigen::Matrix3d &R,
                                            const Eigen::Vector3d &omega0, double dt);
// Normalized update on rotation matrix: R_i+1 = F(dt * omega0) * R_i
Eigen::Matrix3d NormalizedRotationUpdate(const Eigen::Matrix3d &R,
					 const Eigen::Vector3d &omega0, double dt);

#endif
