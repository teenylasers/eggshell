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

/////////////////////////////////////////////////////////////////////////


MatrixXd SelectSubmatrix(const MatrixXd& A, const ArrayXb& row_ind,
                              const ArrayXb& col_ind) {
  // Check input argument dimensions
  const auto dim = A.rows();
  CHECK(A.cols() == dim);        // "A must be a square matrix.\n");
  CHECK(row_ind.rows() == dim);  //, "A and ind dimensions do not match.\n");
  CHECK(col_ind.rows() == dim);  //, "A and ind dimensions do not match.\n");

  // Form submatrix
  // TODO: what is a more efficient way?
  const int sr = row_ind.count();  // submatrix num rows
  const int sc = col_ind.count();  // submatrix num cols
  int si = 0;  // advancing submatrix row index as we fill content
  int sj = 0;  // advancing submatrix col index
  MatrixXd submatrix = MatrixXd::Zero(sr, sc);
  for (int i = 0; i < dim; ++i) {
    for (int j = 0; j < dim; ++j) {
      if (col_ind(j)) {
        if (row_ind(i)) {
          submatrix(si, sj) = A(i, j);
        }
        ++sj;
      }
    }
    sj = 0;
    if (row_ind(i)) {
      ++si;
    }
  }
  return submatrix;
}

VectorXd SelectSubvector(const VectorXd& v, const ArrayXb& ind) {
  if (v.rows() != ind.rows()) {
    std::cout << "v.rows() = " << v.rows() << ", ind.rows() = " << ind.rows()
              << ". Must be the same dimensions.\n";
    CHECK(v.rows() == ind.rows());
  }
  VectorXd subvector = VectorXd::Zero(ind.count());
  int si = 0;
  for (int i = 0; i < v.rows(); ++i) {
    if (ind(i)) {
      subvector(si) = v(i);
      ++si;
    }
  }
  return subvector;
}

void UpdateSubmatrix(MatrixXd& A, const ArrayXb& row_ind,
                          const ArrayXb& col_ind, const MatrixXd& m) {
  // Check input argument dimensions
  const int dim = A.rows();
  CHECK(A.cols() == dim);
  CHECK(row_ind.rows() == dim);
  CHECK(col_ind.rows() == dim);
  const int sr = m.rows();  // submatrix num rows
  const int sc = m.cols();  // submatric num cols
  CHECK(row_ind.count() == sr);
  CHECK(col_ind.count() == sc);
  CHECK(m.rows() == sr);
  CHECK(m.cols() == sc);

  // advancing submatrix row and col indices as we read m content to update A
  int si = 0;
  int sj = 0;
  for (int i = 0; i < dim; ++i) {
    for (int j = 0; j < dim; ++j) {
      if (col_ind(j)) {
        if (row_ind(i)) {
          A(i, j) = m(si, sj);
        }
        ++sj;
      }
    }
    sj = 0;
    if (row_ind(i)) {
      ++si;
    }
  }
}

void UpdateSubvector(VectorXd& v, const ArrayXb& ind, const VectorXd& n) {
  // Check input argument dimensions
  const int dim = v.rows();
  CHECK(ind.rows() == dim);
  const int sd = n.rows();
  CHECK(ind.count() == sd);
  int si = 0;
  for (int i = 0; i < dim; ++i) {
    if (ind(i)) {
      v(i) = n(si);
      ++si;
    }
  }
}

void UpdateSubvector(VectorXd& v, const ArrayXb& ind, double d) {
  // Check input argument dimensions
  const int dim = v.rows();
  CHECK(ind.rows() == dim);
  for (int i = 0; i < dim; ++i) {
    if (ind(i)) {
      v(i) = d;
    }
  }
}

/////////////////////////////////////////////////////////////////////////

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
  CHECK(A.rows() == A.cols() && A.rows() > 0);
  Eigen::EigenSolver<Eigen::MatrixXd> es(A);
  double radius = es.eigenvalues().cwiseAbs().maxCoeff();
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

TEST_FUNCTION(SelectSubmatrix) {
  // Test case
  ArrayXb S(8);
  S << 1, 1, 0, 0, 1, 0, 1, 1;
  MatrixXd A(8, 8);
  A << 44, 23, 81, 97, 37, 34, 72, 51, 12, 12, 3, 55, 99, 68, 91, 48, 26, 30,
      93, 53, 4, 14, 90, 91, 41, 32, 74, 24, 89, 73, 34, 61, 60, 43, 49, 49, 92,
      11, 70, 62, 27, 51, 58, 63, 80, 66, 20, 86, 61, 9, 24, 68, 10, 50, 4, 81,
      72, 27, 46, 40, 27, 78, 75, 58;
  MatrixXd A_SS(5, 5);
  A_SS << 44, 23, 37, 72, 51, 12, 12, 99, 91, 48, 60, 43, 92, 70, 62, 61, 9, 10,
      4, 81, 72, 27, 27, 75, 58;
  MatrixXd A_ScS(3, 5);
  A_ScS << 26, 30, 4, 90, 91, 41, 32, 89, 34, 61, 27, 51, 80, 20, 86;
  MatrixXd A_SSc(5, 3);
  A_SSc << 81, 97, 34, 3, 55, 68, 49, 49, 11, 24, 68, 50, 46, 40, 78;
  MatrixXd A_ScSc(3, 3);
  A_ScSc << 93, 53, 14, 74, 24, 73, 58, 63, 66;
  // Run test
  const ArrayXb S_complement = !S;
  CHECK(A_SS == SelectSubmatrix(A, S, S));
  CHECK(A_ScS == SelectSubmatrix(A, S_complement, S));
  CHECK(A_SSc == SelectSubmatrix(A, S, S_complement));
  CHECK(A_ScSc == SelectSubmatrix(A, S_complement, S_complement));
}

TEST_FUNCTION(UpdateSubmatrix) {
  ArrayXb S(8);
  S << 1, 1, 0, 0, 1, 0, 1, 1;
  MatrixXd A(8, 8);
  A << 90, 82, 36, 39, 57, 17, 23, 11, 96, 25, 84, 57, 47, 61, 92, 97, 55, 93,
      59, 8, 2, 27, 16, 1, 14, 35, 55, 6, 34, 66, 83, 78, 15, 20, 92, 54, 17,
      69, 54, 82, 26, 26, 29, 78, 80, 75, 100, 87, 85, 62, 76, 94, 32, 46, 8, 9,
      26, 48, 76, 13, 53, 9, 45, 40;
  MatrixXd m0(5, 5), m1(5, 3), m2(3, 5), m3(3, 3);
  m0 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
      21, 22, 23, 24, 25;
  m1 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
  m2 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
  m3 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  // Run test
  const ArrayXb S_complement = !S;
  UpdateSubmatrix(A, S, S, m0);
  MatrixXd res(8, 8);
  res << 1, 2, 36, 39, 3, 17, 4, 5, 6, 7, 84, 57, 8, 61, 9, 10, 55, 93, 59, 8,
      2, 27, 16, 1, 14, 35, 55, 6, 34, 66, 83, 78, 11, 12, 92, 54, 13, 69, 14,
      15, 26, 26, 29, 78, 80, 75, 100, 87, 16, 17, 76, 94, 18, 46, 19, 20, 21,
      22, 76, 13, 23, 9, 24, 25;
  CHECK(A == res);
  UpdateSubmatrix(A, S, S_complement, m1);
  res << 1, 2, 1, 2, 3, 3, 4, 5, 6, 7, 4, 5, 8, 6, 9, 10, 55, 93, 59, 8, 2, 27,
      16, 1, 14, 35, 55, 6, 34, 66, 83, 78, 11, 12, 7, 8, 13, 9, 14, 15, 26, 26,
      29, 78, 80, 75, 100, 87, 16, 17, 10, 11, 18, 12, 19, 20, 21, 22, 13, 14,
      23, 15, 24, 25;
  CHECK(A == res);
  UpdateSubmatrix(A, S_complement, S, m2);
  res << 1, 2, 1, 2, 3, 3, 4, 5, 6, 7, 4, 5, 8, 6, 9, 10, 1, 2, 59, 8, 3, 27, 4,
      5, 6, 7, 55, 6, 8, 66, 9, 10, 11, 12, 7, 8, 13, 9, 14, 15, 11, 12, 29, 78,
      13, 75, 14, 15, 16, 17, 10, 11, 18, 12, 19, 20, 21, 22, 13, 14, 23, 15,
      24, 25;
  CHECK(A == res);
  UpdateSubmatrix(A, S_complement, S_complement, m3);
  res << 1, 2, 1, 2, 3, 3, 4, 5, 6, 7, 4, 5, 8, 6, 9, 10, 1, 2, 1, 2, 3, 3, 4,
      5, 6, 7, 4, 5, 8, 6, 9, 10, 11, 12, 7, 8, 13, 9, 14, 15, 11, 12, 7, 8, 13,
      9, 14, 15, 16, 17, 10, 11, 18, 12, 19, 20, 21, 22, 13, 14, 23, 15, 24, 25;
  CHECK(A == res);
}

TEST_FUNCTION(SelectSubvector) {
  VectorXd v(20);
  v << 79, 9, 93, 78, 49, 44, 45, 31, 51, 52, 82, 80, 65, 38, 82, 54, 36, 94,
      88, 56;
  ArrayXb S(20);
  S << 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1;
  VectorXd res(10);
  res << 79, 9, 49, 51, 52, 82, 38, 36, 88, 56;
  CHECK(res == SelectSubvector(v, S));
  res << 93, 78, 44, 45, 31, 80, 65, 82, 54, 94;
  CHECK(res == SelectSubvector(v, !S));
}

TEST_FUNCTION(UpdateSubvector) {
  VectorXd v(20);
  v << 79, 9, 93, 78, 49, 44, 45, 31, 51, 52, 82, 80, 65, 38, 82, 54, 36, 94,
      88, 56;
  ArrayXb S(20);
  S << 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1;
  VectorXd n(10);
  n << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  UpdateSubvector(v, S, n);
  VectorXd res(20);
  res << 1, 2, 93, 78, 3, 44, 45, 31, 4, 5, 6, 80, 65, 7, 82, 54, 8, 94, 9, 10;
  CHECK(res == v);
  UpdateSubvector(v, !S, n);
  res << 1, 2, 1, 2, 3, 3, 4, 5, 4, 5, 6, 6, 7, 7, 8, 9, 8, 10, 9, 10;
  CHECK(res == v);
  UpdateSubvector(v, S, 0);
  res << 0, 0, 1, 2, 0, 3, 4, 5, 0, 0, 0, 6, 7, 0, 8, 9, 0, 10, 0, 0;
  CHECK(res == v);
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

}  // namespace
