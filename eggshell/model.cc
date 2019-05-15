#include "model.h"

#include "collision.h"
#include "constants.h"
#include "ensembles.h"
#include "glog/logging.h"
#include "joints.h"
#include "util.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector3f;

static double angle1 = 0.5;
static double angle2 = 0;

// BoxTest
static Matrix3d random_R = Matrix3d::Identity();
static Matrix3d R_exp_euler = Matrix3d::Identity();
static Matrix3d R_exp_euler_add = Matrix3d::Identity();
static Vector3d w0{10, 20, 15};  // fixed angular velocity, global frame
static Body b0(Vector3d(0, 0, 3), Vector3d::Zero(), Matrix3d::Identity(), w0);
static Body b1(Vector3d(1.5, 0, 3), Vector3d::Zero(), Matrix3d::Identity(), w0);
static Body b2(Vector3d(3, 0, 3), Vector3d::Zero(), Matrix3d::Identity(), w0);

// HangingChain
static Chain ch0 = Chain(10);
static Chain ch1 = Chain(10);

// Cairn

void SimulationInitialization(){};

bool SimulationStep() {
  Eigen::AngleAxisd Raa1(angle1, Vector3d(0, 0, 1));
  Eigen::AngleAxisd Raa2(angle2, Vector3d(0, 1, 0));
  Quaterniond q = Raa1 * Raa2;
  Matrix3d R = q.matrix();
  DrawBox(Vector3d(1, 0, 0.5), R, Vector3d(1, 0.5, 0.5));
  DrawBox(Vector3d(1.65, 0, 0.5), Matrix3d::Identity(), Vector3d(0.3, 1, 1));

  DrawCapsule(Vector3d(-1, 0, 0.5), R, 0.2, 0.6);
  DrawSphere(Vector3d(0, 0, 0.5), R, 0.3);

  DrawPoint(Vector3d(0, 0, 0));
  DrawLine(Vector3d(0, 0, 0), Vector3d(0, 0, 1));

  // Generate contact points and normals from a box and render them.
  std::vector<ContactGeometry> contacts;
  CollideBoxAndGround(Vector3d(1, 0, 0.5), R, Vector3d(1, 0.5, 0.5), &contacts);
  CollideBoxes(Vector3d(1, 0, 0.5), R, Vector3d(1, 0.5, 0.5),
               Vector3d(1.65, 0, 0.5), Matrix3d::Identity(),
               Vector3d(0.3, 1, 1), 0, &contacts);
  for (int i = 0; i < contacts.size(); i++) {
    DrawPoint(contacts[i].position);
    DrawLine(contacts[i].position,
             contacts[i].position + contacts[i].normal * 0.1);
  }

  angle1 += 0.01;
  angle2 += 0.002;

  return true;
}

bool SimulationStep_Cairn() {return true;}

void SimulationInitialization_HangingChain() {
  ch0.Init();
  ch1.Init();
}

bool SimulationStep_HangingChain() {
  ch0.Draw();
  ch0.Step(kSimTimeStep);

  // Post-stabilization
  Eigen::VectorXd err = ch0.ComputeJointError();
  double err_sq = (err * err.transpose()).sum();
  // LOG(INFO) << "Pre-stabilization error sum : " << err_sq;
  int max_stabilization_steps = 500;
  int step_counter = 0;
  while (err_sq > kAllowNumericalError &&
         step_counter < max_stabilization_steps) {
    // ch0.StepPositionRelaxation(kSimTimeStep*1000);
    ch0.StepPostStabilization(kSimTimeStep * 100);
    err = ch0.ComputeJointError();
    err_sq = (err * err.transpose()).sum();
    ++step_counter;
  }
  // LOG(INFO) << "Post-stabilization steps count : " << step_counter;
  LOG(INFO) << "Explicit Euler post-stabilization error_sq sum : " << err_sq;

  ch1.Draw();
  ch1.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
  err = ch1.ComputeJointError();
  err_sq = (err * err.transpose()).sum();
  LOG(INFO) << "ODE step error_sq sum " << err_sq;

  return true;
}

void SimulationInitialization_BoxTests() {
  srand(time(0));
  random_R = RandomRotation();
  R_exp_euler = random_R;
  R_exp_euler_add = random_R;

  Matrix3d inertia;
  // clang-format off
  inertia << 1, 0, 1,
             0, 2, 2,
             2, 0, 3;
  // clang-format on
  b0.SetR(random_R);
  b0.SetI(inertia);
  b1.SetR(random_R);
  b1.SetI(inertia);
  b2.SetR(random_R);
  b2.SetI(inertia);
}

bool SimulationStep_BoxTests() {
  // Draw boxes that rotates with angular velocity w0, start with identical
  // boxes, use different integrators
  Vector3d box_dims{0.3, 0.3, 0.3};
  Vector3d point_on_box_body = box_dims / 2.0;
  Vector3d exp_euler_box_origin{1.5, 0, 1.5};
  Vector3d exp_euler_add_box_origin{3, 0, 1.5};
  DrawBox(exp_euler_box_origin, R_exp_euler, box_dims);
  DrawBox(exp_euler_add_box_origin, R_exp_euler_add, box_dims);
  DrawPoint(exp_euler_box_origin + R_exp_euler * point_on_box_body);
  DrawPoint(exp_euler_add_box_origin + R_exp_euler_add * point_on_box_body);
  R_exp_euler = ExplicitEulerRotationMatrix(R_exp_euler, w0, kSimTimeStep);
  R_exp_euler_add =
      ExplicitEulerRotationMatrix_Addition(R_exp_euler_add, w0, kSimTimeStep);

  // Explicit Euler single rigid body rotation
  b0.Draw();
  b0 = ExplicitEulerBodyRotation(b0, kSimTimeStep);
  // For comparison, update in the body frame
  b1.Draw();
  b1 = ExplicitEulerBodyRotation_BodyFrameUpdate(b1, kSimTimeStep);
  // For comparison, linearized implicit midpoint update
  b2.Draw();
  b2 = LIMBodyRotation(b2, kSimTimeStep);
  // Log angular velocity and kinetic energies
  LOG(INFO) << GetRotationalKE(b0) << ", " << GetRotationalKE(b1) << ", "
            << GetRotationalKE(b2);
  // TODO: SINGLE BODY ROTATION
  // 1. Test that kinetic energy progression makes sense with multiple different
  // random initial R
  //  => with some initial R, kinetic energy jumps. Global frame update
  //  recovers,
  //     but body frame update does not.
  // 2. Why does kinetic energy converge at the end, write out w after each
  // update.

  return true;
}

Matrix3d ExplicitEulerRotationMatrix_Addition(const Matrix3d& R,
                                              const Vector3d& w0, double dt) {
  Matrix3d R_update = GramSchmidt(R + dt * CrossMat(w0) * R);
  return R_update;
}

Matrix3d ExplicitEulerRotationMatrix(const Matrix3d& R, const Vector3d& w0,
                                     double dt) {
  Quaterniond q = WtoQ(w0, dt);
  Matrix3d R_update = q.matrix() * R;
  return R_update;
}

Matrix3d ExplicitEulerRotationMatrix(const Body& b, double dt) {
  Quaterniond q = WtoQ(b.w_g(), dt);
  Matrix3d R_update = q.matrix() * b.R();
  return R_update;
}

Matrix3d ExplicitEulerRotationMatrix_BodyFrameUpdate(const Body& b, double dt) {
  Quaterniond q = WtoQ(b.R() * b.w_b(), dt);
  Matrix3d R_update = q.matrix() * b.R();
  return R_update;
}

Vector3d ExplicitEulerAngularVelocity(const Body& b, double dt,
                                      const Vector3d& tau) {
  Vector3d w_update =
      b.w_g() +
      dt * b.I_g().inverse() * (tau - CrossMat(b.w_g()) * b.I_g() * b.w_g());
  return w_update;
}

Vector3d ExplicitEulerAngularVelocity_BodyFrameUpate(const Body& b, double dt,
                                                     const Vector3d& tau) {
  Vector3d w_update = b.w_b() + dt * b.I_b().inverse() *
                                    (b.R().transpose() * tau -
                                     CrossMat(b.w_b()) * b.I_b() * b.w_b());
  return w_update;
}

Body ExplicitEulerBodyRotation(const Body& b, double dt, const Vector3d& tau) {
  Body b_update(b.p(), b.v(), b.m(), ExplicitEulerRotationMatrix(b, dt),
                ExplicitEulerAngularVelocity(b, dt, tau), b.I_b());
  return b_update;
}

Body ExplicitEulerBodyRotation_BodyFrameUpdate(const Body& b, double dt,
                                               const Vector3d& tau) {
  // get update in body frame
  Body b_update_b(
      b.p(), b.v(), b.m(), ExplicitEulerRotationMatrix_BodyFrameUpdate(b, dt),
      ExplicitEulerAngularVelocity_BodyFrameUpate(b, dt, tau), b.I_b());
  // convert to global frame using updated R
  Body b_update_g(b.p(), b.v(), b.m(), b_update_b.R(),
                  b_update_b.R() * b_update_b.w_g(), b_update_b.I_b());
  return b_update_g;
}

Vector3d LIMAngularVelocity(const Body& b, double dt,
                            const Eigen::Vector3d& tau, double alpha,
                            double beta) {
  Vector3d w_update_b =
      b.w_b() - dt * (alpha + beta) *
                    (Matrix3d::Identity() - dt * beta * b.I_b().inverse() *
                                                (CrossMat(b.I_b() * b.w_b()) -
                                                 CrossMat(b.w_b()) * b.I_b()))
                        .inverse() *
                    b.I_b().inverse() * (CrossMat(b.w_b()) * b.I_b() * b.w_b());
  Vector3d w_update_g = b.R() * w_update_b;
  return w_update_g;
}

Matrix3d LIMRotationMatrix(const Body& b, double dt,
                           const Eigen::Vector3d& w_update, double alpha,
                           double beta) {
  Vector3d w_LIM = alpha * b.w_g() + beta * w_update;
  Quaterniond q = WtoQ(w_LIM, dt);
  Matrix3d R_update = q.matrix() * b.R();
  return R_update;
}

Body LIMBodyRotation(const Body& b, double dt, const Vector3d& tau,
                     double alpha, double beta) {
  Vector3d w_update = LIMAngularVelocity(b, dt, tau, alpha, beta);
  Matrix3d R_update = LIMRotationMatrix(b, dt, w_update, alpha, beta);
  Body b_update(b.p(), b.v(), b.m(), R_update, w_update, b.I_b());
  return b_update;
}
