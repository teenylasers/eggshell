#include "ensembles.h"

#include <cmath>
#include <memory>

#include "glog/logging.h"

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

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
  Quaterniond q = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX());
  Matrix3d R = q.matrix();
  Vector3d w = Vector3d::Zero();
  for (int i = 0; i < n_; ++i) {
    Vector3d p{sqrt(0.3) * i, 0, 6};
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

void Chain::Step(double d, Ensemble::Integrator g) {}

void Chain::Draw() const {
  for (const auto& l : components_) {
    l.Draw();
  }
  for (const auto& j : joints_) {
    j->Draw();
  }
}
