#include "Eigen/Dense"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "model.h"
#include "gtest/gtest.h"
#include <iostream>

DEFINE_bool(verbose, false, "verbose output [false]");

using namespace Eigen;

namespace {
constexpr int kNumTestInsts = 50; // num instances to run in each test
constexpr double kAllowNumericalError = 1e-9;
} // namespace

bool IsOrthonormal(const Matrix3d &R) {
  auto m = R.transpose() * R;
  return m.isIdentity(kAllowNumericalError);
}

TEST(CrossMatTest, ) {
  for (int i = 0; i < kNumTestInsts; i++) {
    Vector3d v = Vector3d::Random();
    Vector3d w = Vector3d::Random();
    Matrix3d vbar = CrossMat(v);
    if (FLAGS_verbose) {
      LOG(INFO) << "--- Test " << i << " ---";
      LOG(INFO) << "v: \n" << v;
      LOG(INFO) << "w: \n" << w;
      LOG(INFO) << "vbar: \n" << vbar;
      LOG(INFO) << "vbar w: \n" << vbar * w;
      LOG(INFO) << "v x w: \n" << v.cross(w);
    }
    EXPECT_LT((vbar * w - v.cross(w)).norm(), kAllowNumericalError);
  }
}

TEST(RandomRotationTest, DefaultOrthonormal) {
  for (int i = 0; i < kNumTestInsts; i++) {
    auto R = RandomRotation();
    if (FLAGS_verbose) {
      LOG(INFO) << "--- Test " << i << " ---";
      LOG(INFO) << "random R: \n" << R;
    }
    EXPECT_TRUE(IsOrthonormal(R));
  }
}

TEST(RandomRotationTest, DefaultRandom) {
  Matrix3d prev_R = Matrix3d::Zero();
  for (int i = 0; i < kNumTestInsts; i++) {
    auto R = RandomRotation();
    if (FLAGS_verbose) {
      LOG(INFO) << "--- Test " << i << " ---";
      LOG(INFO) << "random R: \n" << R;
    }
    EXPECT_GT((prev_R - R).norm(), kAllowNumericalError);
    prev_R = R;
  }
}

TEST(RandomRotationTest, Quaternion) {
  Matrix3d prev_R = Matrix3d::Zero();
  for (int i = 0; i < kNumTestInsts; i++) {
    auto R = RandomRotationViaQuaternion();
    if (FLAGS_verbose) {
      LOG(INFO) << "--- Test " << i << " ---";
      LOG(INFO) << "random R: \n" << R;
    }
    EXPECT_TRUE(IsOrthonormal(R));
    EXPECT_GT((prev_R - R).norm(), kAllowNumericalError);
    prev_R = R;
  }
}

TEST(RandomRotationTest, GramSchmidt) {
  Matrix3d prev_R = Matrix3d::Zero();
  for (int i = 0; i < kNumTestInsts; i++) {
    auto R = RandomRotationViaGramSchmidt();
    if (FLAGS_verbose) {
      LOG(INFO) << "--- Test " << i << " ---";
      LOG(INFO) << "random R: \n" << R;
    }
    EXPECT_TRUE(IsOrthonormal(R));
    EXPECT_GT((prev_R - R).norm(), kAllowNumericalError);
    prev_R = R;
  }
}

GTEST_API_ int main(int argc, char **argv) {
  srand(time(0));
  std::cout << "Run " << __FILE__;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
