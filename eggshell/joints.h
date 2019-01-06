// Library of all joint types

#ifndef __JOINTS_H__
#define __JOINTS_H__

#include "Eigen/Dense"
#include "glog/logging.h"
#include "model.h"

// Base class
class Joint {
 public:
  virtual ~Joint() = default;

  virtual Eigen::Vector3d ComputeError() const;
  virtual Eigen::MatrixXd ComputeJ() const;

  // Visualize the joint implementation in EggshellView
  virtual void Draw() const { LOG(ERROR) << "Joint.Draw()"; };

 protected:
  int n_;  // number of Body connected to this joint
};

// A ball and socket joint
class BallAndSocketJoint : public Joint {
 public:
  explicit BallAndSocketJoint(const Body& b1, const Eigen::Vector3d& c1,
                              const Body& b2, const Eigen::Vector3d& c2)
      : b1_(&b1), c1_(c1), b2_(&b2), c2_(c2) {
    n_ = 2;
  }

  Eigen::Vector3d ComputeError() const override;
  Eigen::MatrixXd ComputeJ() const override;

  void Draw() const override;

 private:
  const Body* b1_;
  const Eigen::Vector3d c1_;
  const Body* b2_;
  const Eigen::Vector3d c2_;
};

#endif
