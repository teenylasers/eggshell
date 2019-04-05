
#ifndef __MODEL_H__
#define __MODEL_H__

#include "Eigen/Dense"

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
void Panic(const char *message, ...);

#endif
