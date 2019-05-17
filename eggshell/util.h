#ifndef __UTIL_H__
#define __UTIL_H__

#include <array>

#include "Eigen/Dense"

// Generate the cross-product matrix for a vector a, so that a x b = a_bar
// b.
Eigen::Matrix3d CrossMat(const Eigen::Vector3d& a);

// Convert angular velocity w to equivalent quaternion
// q = [cos(|w|/2) 1/2*sinc(|w|/2) * w]
Eigen::Quaterniond WtoQ(const Eigen::Vector3d& w, double dt);

// Generate a random position with the confines of x/y/z_bound.
Eigen::Vector3d RandomPosition(const std::array<double, 2>& x_bound,
                               const std::array<double, 2>& y_bound,
                               const std::array<double, 2>& z_bound);
// Generate a random velocity with a magnitude limit.
Eigen::Vector3d RandomVelocity(double limit);
// Generate a random angular velocity with a magnitude limit.
Eigen::Vector3d RandomAngularVelocity(double limit);
// Generate a random rotation matrix.
Eigen::Matrix3d RandomRotation();
Eigen::Matrix3d RandomRotationViaQuaternion();
Eigen::Matrix3d RandomRotationViaGramSchmidt();

Eigen::Matrix3d GramSchmidt(const Eigen::Matrix3d& m);

#endif
