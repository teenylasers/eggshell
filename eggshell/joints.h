// Library of all joint types

#ifndef __JOINTS_H__
#define __JOINTS_H__

#include "Eigen/Dense"
#include "body.h"
#include "glog/logging.h"
#include "model.h"
#include "util.h"

// Base class
class Joint {
 public:
  virtual ~Joint() = default;

  virtual Vector3d ComputeError() const = 0;
  virtual void ComputeJ(MatrixXd* J_b0, MatrixXd* J_b1) const = 0;
  virtual void ComputeJDot(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const = 0;

  // Visualize the joint implementation in EggshellView
  virtual void Draw() const { LOG(ERROR) << "Joint.Draw()"; };
};

// A ball and socket joint
class BallAndSocketJoint : public Joint {
 public:
  // Joint that anchors b0 to c1 in global frame.
  explicit BallAndSocketJoint(const Body* b0, const Vector3d& c0,
                              const Vector3d& c1)
      : b0_(b0), c0_(c0), b1_(nullptr), c1_(c1) {}
  explicit BallAndSocketJoint(const Body* b0, const Vector3d& c0,
                              const Body* b1, const Vector3d& c1)
      : b0_(b0), c0_(c0), b1_(b1), c1_(c1) {}

  Vector3d ComputeError() const override;
  void ComputeJ(MatrixXd* J_b0, MatrixXd* J_b1) const override;
  void ComputeJDot(MatrixXd* Jdot_b0, MatrixXd* Jdot_b1) const override;

  void Draw() const override;

  // TODO: move these to Joint base class protected?
 private:
  const Body* b0_;
  const Vector3d c0_;
  const Body* b1_;
  const Vector3d c1_;
};

#endif
