
#ifndef __MODEL_H__
#define __MODEL_H__

#include "Eigen/Dense"

void DrawSphere(const Eigen::Vector3d &center, const Eigen::Matrix3d &rotation,
                double radius);

void DrawBox(const Eigen::Vector3d &center,
             const Eigen::Matrix3d &rotation,
             const Eigen::Vector3d &side_lengths);

void DrawCapsule(const Eigen::Vector3d &center,
                 const Eigen::Matrix3d &rotation,
                 const Eigen::Vector3d &axis_lengths);

void SimulationStep();

#endif

