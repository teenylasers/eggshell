#include "utils.h"

#include <cmath>
#include <iostream>

#include "constants.h"
#include "error.h"

// TODO: put all this in a namespace or no?

bool IsOrthonormal(const Matrix3d& R) {
  auto m = R.transpose() * R;
  return m.isIdentity(kAllowNumericalError);
}

Matrix3d CrossMat(const Vector3d& a) {
  Matrix3d m;
  // clang-format off
  m << 0, -a(2), a(1),
       a(2), 0, -a(0),
       -a(1), a(0), 0;
  // clang-format on
  return m;
}

Vector3d RandomPosition(const std::array<double, 2>& x_bound,
                        const std::array<double, 2>& y_bound,
                        const std::array<double, 2>& z_bound) {
  // Generate a random Vector3d, whose coeffients are <0, 1>;
  Vector3d v = (Vector3d::Random() + Vector3d::Ones()) / 2;
  v(0) = v(0) * std::abs(x_bound.at(1) - x_bound.at(0)) +
         std::min(x_bound.at(0), x_bound.at(1));
  v(1) = v(1) * std::abs(y_bound.at(1) - y_bound.at(0)) +
         std::min(y_bound.at(0), y_bound.at(1));
  v(2) = v(2) * std::abs(z_bound.at(1) - z_bound.at(0)) +
         std::min(z_bound.at(0), z_bound.at(1));
  return v;
}

Vector3d RandomVelocity(double limit) {
  Vector3d v = Vector3d::Random() * limit;
  return v;
}

Vector3d RandomAngularVelocity(double limit) {
  Vector3d v = Vector3d::Random() * limit;
  return v;
}

Matrix3d RandomRotation() { return RandomRotationViaQuaternion(); }

Matrix3d RandomRotationViaQuaternion() {
  Quaterniond q = Quaterniond::UnitRandom();
  return q.matrix();
}

Matrix3d RandomRotationViaGramSchmidt() {
  Matrix3d m = GramSchmidt(Matrix3d::Random());
  // TODO: Matrix3d m = Matrix3d::Random().householderQr().householderQ();??
  if ((m.col(0).cross(m.col(1)) - m.col(2)).norm() > kAllowNumericalError) {
    m.col(2) = -1.0 * m.col(2);
  }
  return m;
}

Matrix3d GramSchmidt(const Matrix3d& m) {
  // u0, u1, u2 are normalized bases, orthogonal to each other

  // Naive Gram-Schmidt
  Vector3d u0 = m.col(0) / m.col(0).norm();
  Vector3d u1 = m.col(1) - m.col(1).dot(u0) * u0;
  u1 = u1 / u1.norm();
  Vector3d u2 = m.col(2) - m.col(2).dot(u0) * u0 - m.col(2).dot(u1) * u1;
  u2 = u2 / u2.norm();
  Matrix3d m_normalized;
  m_normalized << u0, u1, u2;

  // TODO: stablized Gram-Schmidt
  return m_normalized;
}

Quaterniond WtoQ(const Vector3d& w, double dt) {
  // TODO: this implementation handles zero angular velocity correctly because
  // Eigen normalizes near-zero vectors to (0,0,0). However, normalize() uses
  // Pythagorean magnitude and thus overflows earlier than is optimal.
  Eigen::AngleAxisd aa(w.norm() * dt, w.normalized());
  Quaterniond q(aa);
  return q;
}

Matrix3d AlignVectors(const Vector3d& a, const Vector3d& b) {
  Quaterniond q = Quaterniond::FromTwoVectors(a, b);
  Matrix3d R = q.toRotationMatrix();
  return R;

  // Method 2, equivalent to above.
  // 1. a x b to find the axis of rotation
  // 2. a . b to find rotation angle
  // 3. build angle-axis or quarternion
  // 4. transform into a rotation matrix
  // Vector3d rotation_axis = a.cross(b);
  // rotation_axis.normalize();
  // double rotation_angle = acos(a.dot(b) / a.norm() / b.norm());
  // AngleAxisd aa(rotation_angle, rotation_axis);
  // Matrix3d R2 = aa.toRotationMatrix();
  // std::cout << "\nQuaterniond::FromTwoVectors R = \n" << R1;
  // std::cout << "\nRotation axis = \n" << rotation_axis;
  // std::cout << "\nRotation angle = \n" << rotation_angle;
  // std::cout << "\nAngleAxisd::toRorationMatrix R = \n" << R2;
  // std::cout << "\n(R1 - R2).norm() = " << (R1 - R2).norm() << "\n";
  // CHECK((R1 - R2).norm() < kAllowNumericalError);
}

double GetConditionNumber(const Eigen::MatrixXd& A) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
  double cond = svd.singularValues()(0) /
                svd.singularValues()(svd.singularValues().size() - 1);
  return cond;
}

bool CheckMatrixCondition(const MatrixXd& A) {
  if (A.rows() > A.cols()) {
    // std::cout << "ERROR: A has " << A.rows() << " rows and " << A.cols()
    //           << " cols => singular." << std::endl;
    return false;
  } else {
    bool good = GetConditionNumber(A) < kGoodConditionNumber;
    // if (!good) {
    //   std::cout << "ERROR: GetConditionNumber(A) [" << GetConditionNumber(A)
    //             << "] > kGoodConditionNumber [" << kGoodConditionNumber
    //             << "]\n.";
    // }
    return good;
  }
}

double GetMatrixSparsity(const MatrixXd& A) {
  return (A.array() == 0).count() * 1.0 / A.rows() / A.cols();
}

double GetMatrixBlockSparsity(const MatrixXd& A, int block_width) {
  CHECK(block_width <= A.cols());
  CHECK(A.cols() % block_width == 0);
  const int num_blocks = A.rows() * (A.cols() / block_width);
  int num_zero_blocks = 0;
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < A.cols() / block_width; ++j) {
      if ((A.block(i, j * block_width, 1, block_width).array() != 0).count() ==
          0) {
        ++num_zero_blocks;
      }
    }
  }
  return num_zero_blocks * 1.0 / num_blocks;
}

//***************************************************************************
// Testing.

#include <stdio.h>

#include "testing.h"

// TODO: why anonymous namespace here?
namespace {
constexpr bool kVerbose = false;
constexpr int kNumTestInsts = 10;  // num instances to run in each test

TEST_FUNCTION(CrossMat) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    Vector3d v = Vector3d::Random();
    Vector3d w = Vector3d::Random();
    Matrix3d vbar = CrossMat(v);
    if (kVerbose) {
      std::cout << "--- Test " << i << " ---";
      std::cout << "v: \n" << v;
      std::cout << "w: \n" << w;
      std::cout << "vbar: \n" << vbar;
      std::cout << "vbar w: \n" << vbar * w;
      std::cout << "v x w: \n" << v.cross(w);
    }
    CHECK((vbar * w - v.cross(w)).isZero(kAllowNumericalError));
  }
}

TEST_FUNCTION(RandomRotationTest_DefaultOrthonormal) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    auto R = RandomRotation();
    if (kVerbose) {
      std::cout << "--- Test " << i << " ---";
      std::cout << "random R: \n" << R;
    }
    CHECK(IsOrthonormal(R));
  }
}

TEST_FUNCTION(RandomRotationTest_DefaultRandom) {
  Matrix3d prev_R = Matrix3d::Zero();
  for (int i = 0; i < kNumTestInsts; ++i) {
    auto R = RandomRotation();
    if (kVerbose) {
      std::cout << "\n--- Test " << i << " ---";
      std::cout << "\nrandom R: \n" << R;
    }
    CHECK((prev_R - R).norm() >= kAllowNumericalError);
    prev_R = R;
  }
}

TEST_FUNCTION(RandomRotationTest_Quaternion) {
  Matrix3d prev_R = Matrix3d::Zero();
  for (int i = 0; i < kNumTestInsts; ++i) {
    auto R = RandomRotationViaQuaternion();
    if (kVerbose) {
      std::cout << "\n--- Test " << i << " ---";
      std::cout << "\nrandom R: \n" << R;
    }
    CHECK(IsOrthonormal(R));
    CHECK((prev_R - R).norm() >= kAllowNumericalError);
    prev_R = R;
  }
}

TEST_FUNCTION(RandomRotationTest_GramSchmidt) {
  Matrix3d prev_R = Matrix3d::Zero();
  for (int i = 0; i < kNumTestInsts; ++i) {
    auto R = RandomRotationViaGramSchmidt();
    if (kVerbose) {
      std::cout << "\n--- Test " << i << " ---";
      std::cout << "\nrandom R: \n" << R;
    }
    CHECK(IsOrthonormal(R));
    CHECK((prev_R - R).norm() >= kAllowNumericalError);
    prev_R = R;
  }
}

TEST_FUNCTION(AlignVectors) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    Vector3d a = Vector3d::Random();
    Vector3d b = Vector3d::Random();
    Matrix3d R = AlignVectors(a, b);
    if (kVerbose) {
      std::cout << "\n--- Test " << i << " ---";
      std::cout << "\na = \n" << a;
      std::cout << "\nb = \n" << b;
      std::cout << "\nR = \n" << R;
      std::cout << "\na.norm() = " << a.norm() << "\n";
      std::cout << "\n(R*b).dot(a.normalized()) = "
                << (R * b).dot(a.normalized()) << "\n";
    }
    CHECK(b.normalized().dot(R * a) - a.norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(GetMatrixSparsity) {
  const int A_rows = 16;
  const int A_cols = 60;
  for (int i = 0; i < kNumTestInsts; ++i) {
    const MatrixXd A = MatrixXd::Random(A_rows, A_cols);
    const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> B =
        MatrixXd::Random(A_rows, A_cols).array() < 0;
    const MatrixXd test_matrix = A.cwiseProduct(B.cast<double>());
    const double sparsity = 1 - B.count() * 1.0 / A_rows / A_cols;
    CHECK(abs(GetMatrixSparsity(test_matrix) - sparsity) <=
          kAllowNumericalError);
  }
}

TEST_FUNCTION(GetMatrixBlockSparsity) {
  const int A_rows = 16;
  const int A_cols = 60;

  auto check_block_sparsity = [&](int block_width) {
    MatrixXd A = MatrixXd::Random(A_rows, A_cols);
    const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> B =
        MatrixXd::Random(A_rows, A_cols / block_width).array() < 0;
    // Set blocks of A to zero using boolean matrix B.
    const MatrixXd zero_block = MatrixXd::Zero(1, block_width);
    for (int i = 0; i < A_rows; ++i) {
      for (int j = 0; j < A_cols / block_width; ++j) {
        if (!B(i, j)) {
          A.block(i, j * block_width, 1, block_width) << zero_block;
        }
      }
    }
    // Check block sparsity calculation
    const double sparsity =
        1 - B.count() * 1.0 / (A_rows * A_cols / block_width);
    CHECK(abs(GetMatrixBlockSparsity(A, block_width) - sparsity) <=
          kAllowNumericalError);
  };

  for (int i = 0; i < kNumTestInsts; ++i) {
    check_block_sparsity(/*block_width=*/3);
    check_block_sparsity(/*block_width=*/6);
  }
}

}  // namespace
