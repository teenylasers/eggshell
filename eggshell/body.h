#ifndef __BODY_H__
#define __BODY_H__

#include "Eigen/Dense"

// State of a rigid body
class Body {
 public:
  Body()
      : p_(Eigen::Vector3d::Zero()),
        v_(Eigen::Vector3d::Zero()),
        m_(0.0),
        R_(Eigen::Matrix3d::Identity()),
        w_(Eigen::Vector3d::Zero()),
        I_(Eigen::Matrix3d::Identity()) {}
  explicit Body(const Eigen::Vector3d& p, const Eigen::Matrix3d& R)
      : p_(p),
        v_(Eigen::Vector3d::Zero()),
        m_(1.0),
        R_(R),
        w_(Eigen::Vector3d::Zero()),
        I_(Eigen::Matrix3d::Identity()) {}
  explicit Body(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                const Eigen::Matrix3d& R, const Eigen::Vector3d& w)
      : p_(p), v_(v), m_(1.0), R_(R), w_(w), I_(Eigen::Matrix3d::Identity()) {}
  explicit Body(const Eigen::Vector3d& p, const Eigen::Vector3d& v, double m,
                const Eigen::Matrix3d& R, const Eigen::Vector3d& w,
                const Eigen::Matrix3d& I)
      : p_(p), v_(v), m_(m), R_(R), w_(w), I_(I) {}
  explicit Body(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                const Eigen::Quaterniond& q, const Eigen::Vector3d& w)
      : p_(p),
        v_(v),
        m_(1.0),
        R_(q.matrix()),
        w_(w),
        I_(Eigen::Matrix3d::Identity()) {}
  explicit Body(const Eigen::Vector3d& p, const Eigen::Vector3d& v, double m,
                const Eigen::Quaterniond& q, const Eigen::Vector3d& w,
                const Eigen::Matrix3d& I)
      : p_(p), v_(v), m_(m), R_(q.matrix()), w_(w), I_(I) {}
  const Eigen::Vector3d& p() const { return p_; };
  const Eigen::Vector3d& v() const { return v_; };
  const double m() const { return m_; };
  const Eigen::Quaterniond q() const { return Eigen::Quaterniond(R_); };
  const Eigen::Matrix3d& R() const { return R_; };
  const Eigen::Vector3d w_b() const { return R().transpose() * w_; };
  const Eigen::Vector3d& w_g() const { return w_; };
  const Eigen::Matrix3d& I_b() const { return I_; };
  const Eigen::Matrix3d I_g() const { return R() * I_ * R().transpose(); };

  void SetP(const Eigen::Vector3d& p) { p_ = p; };
  void SetV(const Eigen::Vector3d& v) { v_ = v; };
  void SetM(double m) { m_ = m; };
  void SetR(const Eigen::Matrix3d& R) { R_ = R; };
  void SetR(const Eigen::Quaterniond& q) { R_ = q.matrix(); };
  void SetW_GlobalFrame(const Eigen::Vector3d& w) { w_ = w; };
  void SetW_BodyFrame(const Eigen::Vector3d& w) { w_ = R() * w; };
  void SetI(const Eigen::Matrix3d& I) { I_ = I; };

  void Rotate(const Eigen::Matrix3d& R);
  void Rotate(const Eigen::Quaterniond& q);

  double GetRotationalKE() const;

  void Draw() const;

 private:
  Eigen::Vector3d p_;  // position p of center of mass in global frame
  Eigen::Vector3d v_;  // linear velocity of center of mass v in global frame
  double m_;           // mass
  Eigen::Matrix3d R_;  // rotation matrix R in global frame
  Eigen::Vector3d w_;  // angular velocity w (omega) in global frame
  Eigen::Matrix3d I_;  // inertia tensor I in body frame.

  // TODO: default Body to a box with sides_ 0.3 for now
  Eigen::Vector3d sides_{0.3, 0.3, 0.3};
};

#endif
