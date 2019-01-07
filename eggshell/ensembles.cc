#include "ensembles.h"

#include <cmath>
#include <memory>

#include "constants.h"
#include "glog/logging.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

VectorXd Ensemble::ComputeJointError() const {
  VectorXd prev_errors(0, 0);
  for (const auto& joint : joints_) {
    const auto e = joint->ComputeError();
    VectorXd errors(prev_errors.rows() + e.rows(), e.cols());
    errors << prev_errors, e;
    prev_errors = errors;
  }
  CHECK(CheckErrorDims(prev_errors))
      << "Unexpected error vector dimensions: " << prev_errors.rows() << "x"
      << prev_errors.cols();
  return prev_errors;
}

MatrixXd Ensemble::ComputeJ() const {
  MatrixXd prev_j(0, 0);
  for (const auto& joint : joints_) {
    const auto j = joint->ComputeJ();
    MatrixXd new_j(prev_j.rows() + j.rows(), prev_j.cols() + j.cols());
    new_j << prev_j, MatrixXd::Zero(prev_j.rows(), j.cols()),
        MatrixXd::Zero(j.rows(), prev_j.cols()), j;
    prev_j = new_j;
  }
  CHECK(CheckJDims(prev_j))
      << "Unexpected J matrix dimensions: " << prev_j.rows() << "x"
      << prev_j.cols();
  return prev_j;
}

bool Ensemble::CheckInitialConditions() const {
  auto error = ComputeJointError();
  if (!error.isZero(kAllowNumericalError)) {
    LOG(ERROR) << "Initial error: " << std::endl << error;
    return false;
  } else {
    return true;
  }
}

Chain::Chain(int num_links) {
  // TODO: set max allowed n_
  CHECK(num_links > 0) << "Cannot create a Chain with " << num_links
                       << " links, must be >0.";
  n_ = num_links;

  InitLinks();
  InitJoints();
}

void Chain::InitLinks() {
  Vector3d v = Vector3d::Zero();
  Quaterniond q =
      Eigen::AngleAxisd(0.95531661812451, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX());
  Matrix3d R = q.matrix();
  Vector3d w = Vector3d::Zero();
  for (int i = 0; i < n_; ++i) {
    Vector3d p{sqrt(3.0) * 0.3 * i, 0, 6};
    Body b(p, v, R, w);
    components_.push_back(b);
  }
}

void Chain::InitJoints() {
  Vector3d c1{0.15, -0.15, 0.15};
  Vector3d c2{-0.15, 0.15, -0.15};
  for (int i = 0; i < n_ - 1; ++i) {
    joints_.push_back(std::shared_ptr<Joint>(new BallAndSocketJoint(
        components_.at(i), c1, components_.at(i + 1), c2)));
  }
}

bool Chain::CheckErrorDims(VectorXd error) const {
  // TODO: hardcoded for all BallAndSocketJoints, can be smarter or delete after
  // debug because Ensemble::ComputeJointError is correct by construction?
  return error.rows() == 3 * joints_.size() && error.cols() == 1;
}

bool Chain::CheckJDims(MatrixXd j) const {
  // TODO: hardcoded for all BallAndSocketJoints, can be smarter or delete after
  // debug because Ensemble::ComputeJointError is correct by construction?
  return j.rows() == 12 * joints_.size() && j.cols() == 12 * joints_.size();
}

void Chain::Step(double d, Ensemble::Integrator g) {}

void Chain::Draw() const {
  for (const auto& l : components_) {
    l.Draw();
  }
  for (const auto& j : joints_) {
    j->Draw();
  }
}
