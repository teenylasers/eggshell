
#include "model.h"

using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;

static double angle1 = 0.5;
static double angle2 = 0;

void SimulationStep() {
  Eigen::AngleAxisd Raa1(angle1, Vector3d(0, 0, 1));
  Eigen::AngleAxisd Raa2(angle2, Vector3d(0, 1, 0));
  Quaterniond q = Raa1 * Raa2;
  Matrix3d R = q.matrix();
  DrawBox(Vector3d(1, 0, 0.5), R, Vector3d(1, 0.5, 0.5));

  DrawCapsule(Vector3d(-1, 0, 0.5), R, 0.2, 0.6);

  DrawSphere(Vector3d(0, 0, 0.5), R, 0.3);

  DrawPoint(Vector3d(0, 0, 0));
  DrawLine(Vector3d(0, 0, 0), Vector3d(0, 0, 1));

  angle1 += 0.01;
  angle2 += 0.002;
}
