#include "sparse_iterations.h"

#include <iostream>

#include "constants.h"
#include "error.h"

namespace {

// For successive-over-relaxation (SOR),
//  - forward impl: M = (1/omega) * D + U, N = (1/omega - 1) * D + U.
//  - backward impl: M = (1/omega) * D + L, N = (1/omega - 1) * D + L.
// kSOR = 1/omega.
constexpr double omega = 1;  // 0 < omega < 2 for convergence.
constexpr double kSOR = 1 / omega;

// Default number of iterations, if none specified by the user.
constexpr int kNumIterations = 1000;

enum struct IterationType {
  JACOBI,
  GAUSS_SEIDEL,
  SOR,
  // SSOR,
};

// Base iteration algorithm. n_iter is the number of iterations.
VectorXd BaseIteration(const MatrixXd& A, const VectorXd& b,
                       IterationType type) {
  // Dimensions of lhs
  CHECK(A.rows() == A.cols() && A.rows() == b.size());
  const int dim = A.rows();
  MatrixXd M = MatrixXd::Zero(dim, dim);
  MatrixXd N = MatrixXd::Zero(dim, dim);

  // Pointer to the function that solves Ax=b for a special A, implementation
  // depends on whether A is diagonal, lower, or upper triangular.
  VectorXd (*matrix_solve)(const MatrixXd&, const VectorXd&);

  // x0 to start the iterations
  VectorXd x = VectorXd::Random(dim);

  switch (type) {
    case IterationType::JACOBI:
      M.diagonal() = A.diagonal();
      N = -1 * (MatrixXd(A.triangularView<Eigen::StrictlyLower>()) +
                MatrixXd(A.triangularView<Eigen::StrictlyUpper>()));
      matrix_solve = &MatrixSolveDiagonal;
      break;
    case IterationType::GAUSS_SEIDEL:
      M = A.triangularView<Eigen::Lower>();
      N = -1 * MatrixXd(A.triangularView<Eigen::StrictlyUpper>());
      matrix_solve = &MatrixSolveLowerTriangle;
      break;
    case IterationType::SOR:
      // Using backward SOR.
      M = A.triangularView<Eigen::Upper>();
      M.diagonal() = kSOR * A.diagonal();
      N = A.triangularView<Eigen::Lower>();
      N.diagonal() = (kSOR - 1) * A.diagonal();
      matrix_solve = &MatrixSolveUpperTriangle;
      break;
    default:
      Panic("Unknown iteration type %d.", type);
  }

  // Check spectral radius to determine whether the iterative methods will
  // converge.
  const double rho = GetSpectralRadius(M.inverse() * N);
  std::cout << "INFO: spectral radius of M^(-1)*N = " << rho << std::endl;
  CHECK(rho < 1);

  // Carry out iterations to find x.
  int i = 0;
  double err = (A * x - b).norm();
  // std::cout << "Step -1: (Ax-b).norm() = " << err << std::endl;
  while (err > kAllowNumericalError && i < kNumIterations) {
    VectorXd rhs = N * x + b;
    x = matrix_solve(M, rhs);
    double new_err = (A * x - b).norm();
    // std::cout << "Step " << i << ": (Ax-b).norm() = " << new_err <<
    // std::endl; CHECK_MSG(new_err < err,
    //           "Error increased with this iteration, divergent trend.");
    err = new_err;
    ++i;
  }
  std::cout << "num iterations for convergence = " << i << std::endl;
  return x;
}

}  // namespace

VectorXd sparse::JacobiIteration(const MatrixXd& lhs, const VectorXd& rhs) {
  return BaseIteration(lhs, rhs, IterationType::JACOBI);
}

VectorXd sparse::GaussSeidelIteration(const MatrixXd& lhs,
                                      const VectorXd& rhs) {
  return BaseIteration(lhs, rhs, IterationType::GAUSS_SEIDEL);
}

VectorXd sparse::SORIteration(const MatrixXd& lhs, const VectorXd& rhs) {
  return BaseIteration(lhs, rhs, IterationType::SOR);
}

//***************************************************************************
// Testing.

#include <stdio.h>

#include "testing.h"

namespace {

// Number of instances to run in each TEST_FUNCTION.
constexpr int kNumTestInsts = 10;

TEST_FUNCTION(JacobiIteration) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate matrix dimension, between 3 and 50.
    const int dim = rand() % 48 + 3;
    const MatrixXd A = GenerateDiagonalDominantMatrix(dim);
    const VectorXd b = VectorXd::Random(dim);
    auto x = sparse::JacobiIteration(A, b);
    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A = " << GetConditionNumber(A)
                << "(Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(GaussSeidelIteration_diagonalDominant) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate matrix dimension, between 3 and 50.
    const int dim = rand() % 48 + 3;
    const MatrixXd A = GenerateDiagonalDominantMatrix(dim);
    const VectorXd b = VectorXd::Random(dim);
    auto x = sparse::GaussSeidelIteration(A, b);
    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A = " << GetConditionNumber(A)
                << ", (Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(GaussSeidelIteration_spd) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate matrix dimension, between 3 and 50.
    const int dim = rand() % 48 + 3;
    // Generate an SPD matrix, add a little diagonal dominance to decrease the
    // spectral radius of the iteration matrix. Otherwise, the spectral radius
    // is often >0.99, resulting in slow convergence.
    const MatrixXd A =
        GenerateSPDMatrix(dim) + MatrixXd::Identity(dim, dim) * 2.0;
    const VectorXd b = VectorXd::Random(dim);
    auto x = sparse::GaussSeidelIteration(A, b);
    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A (" << A.rows() << "x" << A.cols()
                << ") = " << GetConditionNumber(A)
                << ", (Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(SORIteration_diagonalDominant) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate matrix dimension, between 3 and 50.
    const int dim = rand() % 98 + 3;
    // Generate a well-conditioned diagonal-dominant matrix A.
    MatrixXd A = MatrixXd::Random(dim, dim);
    double scale_diagonal =
        A.array().abs().maxCoeff() / A.array().abs().minCoeff() * 2;
    A.diagonal() = A.diagonal().array() * (scale_diagonal);
    if (GetConditionNumber(A) > kGoodConditionNumber) {
      --i;
      continue;
    }
    const VectorXd b = VectorXd::Random(dim);
    auto x = sparse::SORIteration(A, b);
    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A = " << GetConditionNumber(A)
                << ", (Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

}  // namespace
