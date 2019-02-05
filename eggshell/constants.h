#include "Eigen/Dense"

namespace {
constexpr double kAllowNumericalError = 1e-9;
constexpr double kSimTimeStep = 0.001;                      // in msec
const Eigen::Vector3d kGravity(0, 0, -9.8);  // in m/s^2
}  // namespace
