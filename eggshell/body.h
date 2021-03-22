#ifndef __BODY_H__
#define __BODY_H__

#include "Eigen/Dense"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector3f;

// Base class for a rigid body
class Body {
 public:
  virtual ~Body() = default;
  Body()
      : p_(Vector3d::Zero()),
        v_(Vector3d::Zero()),
        m_(1.0),
        R_(Matrix3d::Identity()),
        w_(Vector3d::Zero()),
        I_(Matrix3d::Identity()) {
    I_ = CalculateInertia(m_);
  }
  explicit Body(const Vector3d& p, const Matrix3d& R)
      : p_(p),
        v_(Vector3d::Zero()),
        m_(1.0),
        R_(R),
        w_(Vector3d::Zero()),
        I_(Matrix3d::Identity()) {
    I_ = CalculateInertia(m_);
  }
  explicit Body(const Vector3d& p, const Vector3d& v, const Matrix3d& R,
                const Vector3d& w)
      : p_(p), v_(v), m_(1.0), R_(R), w_(w), I_(Matrix3d::Identity()) {
    I_ = CalculateInertia(m_);
  }
  explicit Body(const Vector3d& p, const Vector3d& v, double m,
                const Matrix3d& R, const Vector3d& w, const Matrix3d& I)
      : p_(p), v_(v), m_(m), R_(R), w_(w), I_(I) {}
  explicit Body(const Vector3d& p, const Vector3d& v, const Quaterniond& q,
                const Vector3d& w)
      : p_(p), v_(v), m_(1.0), R_(q.matrix()), w_(w), I_(Matrix3d::Identity()) {
    I_ = CalculateInertia(m_);
  }
  explicit Body(const Vector3d& p, const Vector3d& v, double m,
                const Quaterniond& q, const Vector3d& w, const Matrix3d& I)
      : p_(p), v_(v), m_(m), R_(q.matrix()), w_(w), I_(I) {}
  const Vector3d& p() const { return p_; };
  const Vector3d& v() const { return v_; };
  const double m() const { return m_; };
  const Quaterniond q() const { return Quaterniond(R_); };
  const Matrix3d& R() const { return R_; };
  const Vector3d w_b() const { return R().transpose() * w_; };
  const Vector3d& w_g() const { return w_; };
  const Matrix3d& I_b() const { return I_; };
  const Matrix3d I_g() const { return R() * I_ * R().transpose(); };

  enum struct BodyType { Box = 0 };

  void SetP(const Vector3d& p) { p_ = p; };
  void SetV(const Vector3d& v) { v_ = v; };
  void SetM(double m) { m_ = m; };
  void SetR(const Matrix3d& R) { R_ = R; };
  void SetR(const Quaterniond& q) { R_ = q.matrix(); };
  void SetW_GlobalFrame(const Vector3d& w) { w_ = w; };
  void SetW_BodyFrame(const Vector3d& w) { w_ = R() * w; };
  void SetI(const Matrix3d& I) { I_ = I; };

  void Rotate(const Matrix3d& R);
  void Rotate(const Quaterniond& q);

  double GetRotationalKE() const;

  // TODO: implement different types of Bodies
  void Draw() const;
  const Vector3d GetSideLengths() const { return side_lengths_; };

 private:
  Vector3d p_;  // position p of center of mass in global frame
  Vector3d v_;  // linear velocity of center of mass v in global frame
  double m_;    // mass
  Matrix3d R_;  // rotation matrix R in global frame
  Vector3d w_;  // angular velocity w (omega) in global frame
  Matrix3d I_;  // inertia tensor I in body frame.

  // TODO: move this to a subclass when there are different types of Bodies.
  // TODO: make side lengths settable, and constructible
  const BodyType type_ = BodyType::Box;
  const Vector3d side_lengths_ = {0.3, 0.3, 0.3};  // x, y, z

  // Calculate the inertia matrix from mass m, BodyType type_, and related body
  // geometry info.
  Matrix3d CalculateInertia(double m) const;
};

#endif
