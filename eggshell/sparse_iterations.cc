#include "sparse_iterations.h"

#include <iostream>

#include "constants.h"
#include "error.h"
#include "sparse_iterations_utils.h"

namespace {

// For successive-over-relaxation (SOR),
//  - forward impl: M = (1/omega) * D + U, N = (1/omega - 1) * D + U.
//  - backward impl: M = (1/omega) * D + L, N = (1/omega - 1) * D + L.
// kSOR = 1/omega.
constexpr double omega = 1.5;  // 0 < omega < 2 for convergence.
constexpr double kSOR = 1 / omega;

// Default number of iterations, if none specified by the user.
constexpr int kNumIterations = 500;

enum struct IterationType {
  JACOBI,
  GAUSS_SEIDEL,
  SOR,
  // SSOR,
};

// Base iteration algorithm for Ax=b.
VectorXd BaseIteration(const MatrixXd& A, const VectorXd& b,
                       IterationType type) {
  // Dimensions of lhs
  CHECK(A.rows() == A.cols() && A.rows() == b.size());
  const int dim = A.rows();
  MatrixXd M = MatrixXd::Zero(dim, dim);
  MatrixXd N = MatrixXd::Zero(dim, dim);

  // Function pointer to the matrix solver, depending on whether the matrix
  // structure is diagonal, lower, or upper triangular.
  VectorXd (*matrix_solver)(const MatrixXd&, const VectorXd&);

  switch (type) {
    case IterationType::JACOBI:
      M.diagonal() = A.diagonal();
      N = -1 * (MatrixXd(A.triangularView<Eigen::StrictlyLower>()) +
                MatrixXd(A.triangularView<Eigen::StrictlyUpper>()));
      matrix_solver = &sparse::MatrixSolveDiagonal;
      break;
    case IterationType::GAUSS_SEIDEL:
      M = A.triangularView<Eigen::Lower>();
      N = -1 * MatrixXd(A.triangularView<Eigen::StrictlyUpper>());
      matrix_solver = &sparse::MatrixSolveLowerTriangle;
      break;
    case IterationType::SOR:
      // Using backward SOR.
      M = A.triangularView<Eigen::Upper>();
      M.diagonal() = kSOR * A.diagonal();
      N = -1 * MatrixXd(A.triangularView<Eigen::Lower>());
      N.diagonal() = (kSOR - 1) * A.diagonal();
      matrix_solver = &sparse::MatrixSolveUpperTriangle;
      break;
    default:
      Panic("Unknown iteration type %d.", type);
  }

  // Check spectral radius to determine whether the iterative methods will
  // converge.
  const double rho = GetSpectralRadius(M.inverse() * N);
  std::cout << "INFO: spectral radius of M^(-1)*N = " << rho << std::endl;
  CHECK(rho < 1);

  // x0 to start the iterations
  VectorXd x = VectorXd::Random(dim);

  // Carry out iterations to find x.
  int i = 0;
  double err = (A * x - b).norm();
  // std::cout << "Step -1: (Ax-b).norm() = " << err << std::endl;
  while (err > kAllowNumericalError && i < kNumIterations) {
    VectorXd rhs = N * x + b;
    x = matrix_solver(M, rhs);
    double new_err = (A * x - b).norm();
    // std::cout << "Step " << i << ": (Ax-b).norm() = " << new_err <<
    // std::endl;
    // CHECK_MSG(new_err < err,
    //           "Error increased with this iteration, divergent trend.");
    err = new_err;
    ++i;
  }
  std::cout << "num iterations for convergence = " << i << std::endl;
  return x;
}

// Base iteration algorithm for an ensemble, without explicitly forming the
// systems matrix JMJt.
VectorXd BaseIteration(const ConstraintsList& constraints,
                       const MatrixXd& M_inverse, const VectorXd& rhs,
                       IterationType type) {
  // Iteratively solve for x(i+1) in Mx(i+1) = Nx(i) + b

  // Function pointer to the solver for x(i+1) in each iterative step, depending
  // on whether the equivalent matrix structure is diagonal, lower, or upper
  // triangular. Function arguments are:
  //  1. ConstraintsList&, MatrixXd& M_inverse - to form the equivalent of M
  //     matrix
  //  2. VectorXd& rhs = Nx(i) + b
  //  3. double Mx_solver_scale
  VectorXd (*Mx_solver)(const ConstraintsList&, const MatrixXd&,
                        const VectorXd&, double);
  double Mx_solver_scale = 1.0;

  // Function pointer to get Nx(i). Function arguments are:
  //  1. ConstraintsList&, MatrixXd& M_inverse - to form the equivalent of N
  //     matrix
  //  2. VectorXd& x(i)
  //  3. double get_Nx_scale
  VectorXd (*get_Nx)(const ConstraintsList&, const MatrixXd&, const VectorXd&,
                     double);
  double get_Nx_scale = 1.0;

  // Set Mx_solver and get_Nx for each iteration type.
  switch (type) {
    case IterationType::JACOBI:
      get_Nx = &sparse::CalculateBlockLxUx;
      Mx_solver = &sparse::MatrixSolveBlockDiagonal;
      break;
    case IterationType::GAUSS_SEIDEL:
      get_Nx = &sparse::CalculateBlockUx;
      Mx_solver = &sparse::MatrixSolveBlockLowerTriangle;
      break;
    case IterationType::SOR:
      get_Nx = &sparse::CalculateBlockLxDx;
      get_Nx_scale = 1 - kSOR;
      Mx_solver = &sparse::MatrixSolveBlockUpperTriangle;
      Mx_solver_scale = kSOR;
      break;
    default:
      Panic("Unknown iteration type %d.", type);
  }

  // x0 to start the iterations
  VectorXd x = VectorXd::Random(rhs.size());

  // Carry out iterations to find x.
  int i = 0;
  double err =
      (sparse::CalculateBlockJMJtX(constraints, M_inverse, x) - rhs).norm();
  // std::cout << "Step -1: (Ax-b).norm() = " << err << std::endl;
  while (err > kAllowNumericalError && i < kNumIterations) {
    // iteration_rhs = Nx(i) + b = get_Nx() + rhs.
    VectorXd iteration_rhs =
        -1 * get_Nx(constraints, M_inverse, x, get_Nx_scale) + rhs;
    x = Mx_solver(constraints, M_inverse, iteration_rhs, Mx_solver_scale);
    double new_err =
        (sparse::CalculateBlockJMJtX(constraints, M_inverse, x) - rhs).norm();
    // std::cout << "Step " << i << ": (Ax-b).norm() = " << new_err <<
    // std::endl;
    // CHECK_MSG(new_err < err,
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

VectorXd sparse::JacobiIteration(const ConstraintsList& constraints,
                                 const MatrixXd& M_inverse,
                                 const VectorXd& rhs) {
  return BaseIteration(constraints, M_inverse, rhs, IterationType::JACOBI);
}

VectorXd sparse::GaussSeidelIteration(const ConstraintsList& constraints,
                                      const MatrixXd& M_inverse,
                                      const VectorXd& rhs) {
  return BaseIteration(constraints, M_inverse, rhs,
                       IterationType::GAUSS_SEIDEL);
}

VectorXd sparse::SORIteration(const ConstraintsList& constraints,
                              const MatrixXd& M_inverse, const VectorXd& rhs) {
  return BaseIteration(constraints, M_inverse, rhs, IterationType::SOR);
}

//***************************************************************************
// Testing.

#include <stdio.h>

#include "testing.h"

namespace {

// Number of instances to run in each TEST_FUNCTION.
constexpr int kNumTestInsts = 10;

// Number of eggshell simulation steps to run in each relevant TEST_FUNCTION.
constexpr int kNumSimSteps = 20;

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
    const int dim = rand() % 48 + 3;
    const MatrixXd A = GenerateDiagonalDominantMatrix(dim);
    const VectorXd b = VectorXd::Random(dim);
    auto x = sparse::SORIteration(A, b);
    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A = " << GetConditionNumber(A)
                << ", (Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(SORIteration_spd) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate matrix dimension, between 3 and 50.
    const int dim = rand() % 48 + 3;
    const MatrixXd A =
        GenerateSPDMatrix(dim) + MatrixXd::Identity(dim, dim) * 2.0;
    const VectorXd b = VectorXd::Random(dim);
    auto x = sparse::SORIteration(A, b);
    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A = " << GetConditionNumber(A)
                << ", (Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(JacobiIteration_ensemble) {
  auto check_jacobi = [](const Ensemble& en) {
    const MatrixXd J = en.ComputeJ();
    const MatrixXd M = en.M_inverse();

    // Use a randomly generated rhs
    const VectorXd rhs = VectorXd::Random(J.rows());

    // Solve (JMJt * x = rhs) using Gause-Seidel iteration
    const VectorXd x =
        sparse::JacobiIteration(en.constraints(), en.M_inverse(), rhs);

    // Check x
    return (J * M * J.transpose() * x - rhs).norm() < kAllowNumericalError;
  };

  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(check_jacobi(chain));
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(check_jacobi(chain));
  }

  Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
  cairn.Init();
  cairn.InitStabilize();
  CHECK(check_jacobi(cairn));
  for (int i = 0; i < kNumSimSteps; ++i) {
    cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(check_jacobi(cairn));
  }
}

TEST_FUNCTION(GaussSeidelIteration_ensemble) {
  auto check_gauss_seidel = [](const Ensemble& en) {
    const MatrixXd J = en.ComputeJ();
    const MatrixXd M = en.M_inverse();

    // Use a randomly generated rhs
    const VectorXd rhs = VectorXd::Random(J.rows());

    // Solve (JMJt * x = rhs) using Gause-Seidel iteration
    const VectorXd x =
        sparse::GaussSeidelIteration(en.constraints(), en.M_inverse(), rhs);

    // Check x
    return (J * M * J.transpose() * x - rhs).norm() < kAllowNumericalError;
  };

  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(check_gauss_seidel(chain));
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(check_gauss_seidel(chain));
  }

  Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
  cairn.Init();
  cairn.InitStabilize();
  CHECK(check_gauss_seidel(cairn));
  for (int i = 0; i < kNumSimSteps; ++i) {
    cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(check_gauss_seidel(cairn));
  }
}

TEST_FUNCTION(SORIteration_ensemble) {
  auto check_sor = [](const Ensemble& en) {
    const MatrixXd J = en.ComputeJ();
    const MatrixXd M = en.M_inverse();

    // Use a randomly generated rhs
    const VectorXd rhs = VectorXd::Random(J.rows());

    // Solve (JMJt * x = rhs) using Gause-Seidel iteration
    const VectorXd x =
        sparse::SORIteration(en.constraints(), en.M_inverse(), rhs);

    // Check x
    return (J * M * J.transpose() * x - rhs).norm() < kAllowNumericalError;
  };

  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(check_sor(chain));
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(check_sor(chain));
  }

  Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
  cairn.Init();
  cairn.InitStabilize();
  CHECK(check_sor(cairn));
  for (int i = 0; i < kNumSimSteps; ++i) {
    cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(check_sor(cairn));
  }
}

}  // namespace
