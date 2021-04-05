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

MatrixXd GenerateSPDMatrix(int dimension) {
  auto gen_matrix = [&]() {
    const MatrixXd m = MatrixXd::Random(dimension, dimension);
    const MatrixXd A = m.transpose() * m;
    return A;
  };

  MatrixXd M = gen_matrix();
  while (GetConditionNumber(M) > kGoodConditionNumber) {
    M = gen_matrix();
  }
  return M;
}

MatrixXd GenerateDiagonalDominantMatrix(int dimension) {
  auto gen_matrix = [&]() {
    MatrixXd A = MatrixXd::Random(dimension, dimension);
    double scale_diagonal =
        A.array().abs().maxCoeff() / A.array().abs().minCoeff();
    A.diagonal() = A.diagonal().array() * (scale_diagonal);
    return A;
  };

  MatrixXd M = gen_matrix();
  while (GetConditionNumber(M) > kGoodConditionNumber) {
    M = gen_matrix();
  }
  return M;
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

double GetSpectralRadius(const Eigen::MatrixXd& A) {
  Eigen::EigenSolver<Eigen::MatrixXd> es(A);
  auto lambda = es.eigenvalues();
  auto lambda_sqr = lambda.array() * lambda.array();
  double radius = sqrt(lambda_sqr.real().maxCoeff());
  return radius;
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

MatrixXd InvertDiagonalMatrix(const MatrixXd& D) {
  CHECK(D.rows() == D.cols());
  CHECK_MSG(D.diagonal().prod() != 0, "det(D)==0, D is not invertible.");
  MatrixXd inv_D = MatrixXd::Zero(D.rows(), D.cols());
  inv_D.diagonal() = 1.0 / D.diagonal().array();
  return inv_D;
}

VectorXd MatrixSolveDiagonal(const MatrixXd& D, const VectorXd& rhs) {
  CHECK(D.rows() == D.cols());
  CHECK_MSG(D.diagonal().prod() != 0, "det(D)==0, D is not invertible.");
  VectorXd x = 1.0 / D.diagonal().array() * rhs.array();
  return x;
}

VectorXd MatrixSolveLowerTriangle(const MatrixXd& L, const VectorXd& rhs) {
  CHECK(L.rows() == L.cols() && L.rows() == rhs.size());
  CHECK_MSG(L.diagonal().prod() != 0, "L is not invertible.");
  const int dim = rhs.size();
  VectorXd x = VectorXd::Zero(dim);
  for (int i = 0; i < dim; ++i) {
    double substitutions = 0;
    for (int j = 0; j < i; ++j) {
      substitutions += L(i, j) * x(j);
    }
    x(i) = (rhs(i) - substitutions) / L(i, i);
  }
  return x;
}

VectorXd MatrixSolveUpperTriangle(const MatrixXd& U, const VectorXd& rhs) {
  CHECK(U.rows() == U.cols() && U.rows() == rhs.size());
  CHECK_MSG(U.diagonal().prod() != 0, "U is not invertible.");
  const int dim = rhs.size();
  VectorXd x = VectorXd::Zero(dim);
  for (int i = dim - 1; i >= 0; --i) {
    double substitutions = 0;
    for (int j = i + 1; j < dim; ++j) {
      substitutions += U(i, j) * x(j);
    }
    x(i) = (rhs(i) - substitutions) / U(i, i);
  }
  return x;
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

TEST_FUNCTION(InvertDiagonalMatrix) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate a matrix with dimension between 3 and 100.
    int dim = rand() % 98 + 3;
    // Construct a random diagonal matrix
    MatrixXd A = MatrixXd::Zero(dim, dim);
    A.diagonal() = VectorXd::Random(dim);
    CHECK((A * InvertDiagonalMatrix(A) - MatrixXd::Identity(dim, dim)).norm() <
          kAllowNumericalError);
  }
}

TEST_FUNCTION(MatrixSolveDiagonal) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate a matrix with dimension between 3 and 100.
    int dim = rand() % 98 + 3;
    // Construct a random diagonal matrix
    MatrixXd A = MatrixXd::Zero(dim, dim);
    A.diagonal() = VectorXd::Random(dim);
    // rhs
    const VectorXd b = VectorXd::Random(dim);
    // Solve
    auto x = MatrixSolveDiagonal(A, b);
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(MatrixSolveLowerTriangle) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate a matrix with dimension between 3 and 100.
    int dim = rand() % 98 + 3;
    // Construct a well-conditioned lower triangle matrix
    MatrixXd A = MatrixXd::Random(dim, dim).triangularView<Eigen::Lower>();
    if (GetConditionNumber(A) > kGoodConditionNumber) {
      --i;
      continue;
    }
    const VectorXd b = VectorXd::Random(dim);
    auto x = MatrixSolveLowerTriangle(A, b);
    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A = " << GetConditionNumber(A)
                << std::endl;
      std::cout << "(Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(MatrixSolveUpperTriangle) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate a matrix with dimension between 3 and 100.
    int dim = rand() % 98 + 3;
    // Construct a well-conditioned upper triangle matrix
    MatrixXd A = MatrixXd::Random(dim, dim).triangularView<Eigen::Upper>();
    if (GetConditionNumber(A) > kGoodConditionNumber) {
      --i;
      continue;
    }
    const VectorXd b = VectorXd::Random(dim);
    auto x = MatrixSolveUpperTriangle(A, b);
    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A = " << GetConditionNumber(A)
                << std::endl;
      std::cout << "(Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

}  // namespace
