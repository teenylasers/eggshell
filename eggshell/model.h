
#ifndef __MODEL_H__
#define __MODEL_H__

#include "Eigen/Dense"

// This is called to advance the simulation by one time step.

void SimulationStep();

// SimulationStep() can call these functions to draw output to the shell.

void DrawSphere(const Eigen::Vector3d &center, const Eigen::Matrix3d &rotation,
                double radius);

void DrawBox(const Eigen::Vector3d &center,
             const Eigen::Matrix3d &rotation,
             const Eigen::Vector3d &side_lengths);

void DrawCapsule(const Eigen::Vector3d &center,
                 const Eigen::Matrix3d &rotation,
                 double radius, double length);

void DrawPoint(const Eigen::Vector3d &position);

void DrawLine(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2);

#endif
