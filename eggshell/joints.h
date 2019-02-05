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
  virtual void ComputeJ(Eigen::MatrixXd& J_b1) const = 0;
  virtual void ComputeJ(Eigen::MatrixXd& J_b1, Eigen::MatrixXd& J_b2) const = 0;
  virtual void ComputeJDot(Eigen::MatrixXd& Jdot_b1) const = 0;
  virtual void ComputeJDot(Eigen::MatrixXd& Jdot_b1,
                           Eigen::MatrixXd& Jdot_b2) const = 0;

  // Visualize the joint implementation in EggshellView
  virtual void Draw() const { LOG(ERROR) << "Joint.Draw()"; };
};

// A ball and socket joint
class BallAndSocketJoint : public Joint {
 public:
  explicit BallAndSocketJoint(const Body& b1, const Eigen::Vector3d& c1)
      : b1_(&b1), c1_(c1), b2_(nullptr), c2_(Eigen::Vector3d::Zero()) {}
  explicit BallAndSocketJoint(const Body& b1, const Eigen::Vector3d& c1,
                              const Body& b2, const Eigen::Vector3d& c2)
      : b1_(&b1), c1_(c1), b2_(&b2), c2_(c2) {}

  Eigen::Vector3d ComputeError() const override;
  void ComputeJ(Eigen::MatrixXd& J_b1) const override;
  void ComputeJ(Eigen::MatrixXd& J_b1, Eigen::MatrixXd& J_b2) const override;
  void ComputeJDot(Eigen::MatrixXd& Jdot_b1) const override;
  void ComputeJDot(Eigen::MatrixXd& Jdot_b1,
                   Eigen::MatrixXd& Jdot_b2) const override;

  void Draw() const override;

 private:
  const Body* b1_;
  const Eigen::Vector3d c1_;
  const Body* b2_;
  const Eigen::Vector3d c2_;
};

#endif
