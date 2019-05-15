#ifndef __UTIL_H__
#define __UTIL_H__

#include "Eigen/Dense"

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

#endif
