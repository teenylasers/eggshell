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

// Helper function to return an indication of residual error. For example, to
// check whether the iterations are converging.
// double GetResidualError(const MatrixXd& A, const VectorXd& b,
//                         const VectorXd& x) {
//   return (A * x - b).norm();
// }

double GetResidualError(const MatrixXd& A, const VectorXd& b, const VectorXd& x,
                        const ArrayXb& C, const VectorXd& x_lo,
                        const VectorXd& x_hi) {
  VectorXd w = A * x - b;
  double equality_error = SelectSubvector(w, C).norm();
  double inequality_error =
      SelectSubvector(w, !C && x.array() == x_lo.array() && w.array() < 0)
          .norm() +
      SelectSubvector(w, !C && x.array() == x_hi.array() && w.array() > 0)
          .norm() +
      SelectSubvector(
          w, !C && x.array() > x_lo.array() && x.array() < x_hi.array())
          .norm();
  return equality_error + inequality_error;
}

double GetResidualError(const ConstraintsList& constraints,
                        const MatrixXd& M_inverse, const VectorXd& rhs,
                        const VectorXd& x, double cfm_coeff) {
  ArrayXb C;
  VectorXd x_lo, x_hi;
  sparse::ConstructMixedConstraints(constraints, &C, &x_lo, &x_hi);
  VectorXd w =
      sparse::CalculateBlockJMJtX(constraints, M_inverse, x, cfm_coeff) - rhs;
  double equality_error = SelectSubvector(w, C).norm();
  double inequality_error =
      SelectSubvector(w, !C && x.array() == x_lo.array() && w.array() < 0)
          .norm() +
      SelectSubvector(w, !C && x.array() == x_hi.array() && w.array() > 0)
          .norm() +
      SelectSubvector(
          w, !C && x.array() > x_lo.array() && x.array() < x_hi.array())
          .norm();
  return equality_error + inequality_error;
}

// Base iteration algorithm for Ax=b.
VectorXd BaseIteration(const MatrixXd& A, const VectorXd& b, IterationType type,
                       const ArrayXb& C, const VectorXd& x_lo,
                       const VectorXd& x_hi) {
  // Dimensions of lhs
  CHECK(A.rows() == A.cols() && A.rows() == b.size());
  const int dim = A.rows();
  if (dim == 0) {
    return VectorXd(0);
  }

  MatrixXd M = MatrixXd::Zero(dim, dim);
  MatrixXd N = MatrixXd::Zero(dim, dim);

  // Function pointer to the matrix solver, depending on whether the matrix
  // structure is diagonal, lower, or upper triangular.
  VectorXd (*matrix_solver)(const MatrixXd&, const VectorXd&, const ArrayXb&,
                            const VectorXd&, const VectorXd&);

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
  if (rho >= 1) {
    std::cout << "M.inverse() * N = \n" << M.inverse() * N << std::endl;
    CHECK(rho < 1);
  }

  // x0 to start the iterations
  VectorXd x = b;

  // Carry out iterations to find x.
  int i = 0;
  double err = GetResidualError(A, b, x, C, x_lo, x_hi);
  // std::cout << "Step -1: (Ax-b).norm() = " << err << std::endl;
  while (err > kAllowNumericalError && i < kNumIterations) {
    VectorXd rhs = N * x + b;
    x = matrix_solver(M, rhs, C, x_lo, x_hi);
    double new_err = GetResidualError(A, b, x, C, x_lo, x_hi);
    // if (i % 50 == 0 || new_err > err || err < kAllowNumericalError) {
    //   std::cout << "Step " << i << ": residual error = " << new_err
    //             << std::endl;
    // }
    err = new_err;
    ++i;
  }
  std::cout << "num iterations for convergence (dense algs) = " << i
            << std::endl;
  return x;
}

// Base iteration algorithm for an ensemble, without explicitly forming the
// systems matrix JMJt.
VectorXd BaseIteration(const ConstraintsList& constraints,
                       const MatrixXd& M_inverse, const VectorXd& rhs,
                       IterationType type, double cfm_coeff) {
  // If constraints.size() == 0, then the systems matrix JMJt is empty.
  if (constraints.size() == 0) {
    return VectorXd(0);
  }

  // Construct mixed constraints
  ArrayXb C;
  VectorXd x_lo, x_hi;
  sparse::ConstructMixedConstraints(constraints, &C, &x_lo, &x_hi);

  // Function pointer to the solver for x(i+1) in each iterative step, depending
  // on whether the equivalent matrix structure is diagonal, lower, or upper
  // triangular. Function arguments are:
  //  1. ConstraintsList&, MatrixXd& M_inverse - to form the equivalent of M
  //     matrix
  //  2. VectorXd& rhs = Nx(i) + b
  //  3. double Mx_solver_scale, for SOR.
  VectorXd (*Mx_solver)(const ConstraintsList&, const MatrixXd&,
                        const VectorXd&, double, double);
  double Mx_solver_scale = 1.0;

  // Function pointer to get Nx(i). Function arguments are:
  //  1. ConstraintsList&, MatrixXd& M_inverse - to form the equivalent of N
  //     matrix
  //  2. VectorXd& x(i)
  //  3. double get_Nx_scale
  VectorXd (*get_Nx)(const ConstraintsList&, const MatrixXd&, const VectorXd&,
                     double, double);
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
  VectorXd x = rhs;

  // Carry out iterations to find x.
  int i = 0;
  double err = GetResidualError(constraints, M_inverse, rhs, x, cfm_coeff);
  // std::cout << "Step -1: (Ax-b).norm() = " << err << std::endl;
  while (err > kAllowNumericalError && i < kNumIterations) {
    // iteration_rhs = Nx(i) + b = get_Nx() + rhs.
    const VectorXd iteration_rhs =
        -1 * get_Nx(constraints, M_inverse, x, cfm_coeff, get_Nx_scale) + rhs;
    x = Mx_solver(constraints, M_inverse, iteration_rhs, cfm_coeff,
                  Mx_solver_scale);
    double new_err =
        GetResidualError(constraints, M_inverse, rhs, x, cfm_coeff);
    // if (i % 50 == 0 || new_err > err || err < kAllowNumericalError) {
    //   std::cout << "Step " << i << ": residual error = " << new_err
    //             << std::endl;
    // }
    err = new_err;
    ++i;
  }
  std::cout << "num iterations for convergence (sparse algs) = " << i
            << std::endl;
  return x;
}

}  // namespace

VectorXd sparse::JacobiIteration(const MatrixXd& A, const VectorXd& b) {
  return BaseIteration(A, b, IterationType::JACOBI,
                       /*C = */ ArrayXb::Constant(b.size(), true),
                       /*x_lo = */ VectorXd::Zero(b.size()),
                       /*x_hi = */ VectorXd::Zero(b.size()));
}

VectorXd sparse::JacobiIteration(const MatrixXd& A, const VectorXd& b,
                                 const ArrayXb& C, const VectorXd& x_lo,
                                 const VectorXd& x_hi) {
  return BaseIteration(A, b, IterationType::JACOBI, C, x_lo, x_hi);
}

VectorXd sparse::GaussSeidelIteration(const MatrixXd& A, const VectorXd& b) {
  return BaseIteration(A, b, IterationType::GAUSS_SEIDEL,
                       /*C = */ ArrayXb::Constant(b.size(), true),
                       /*x_lo = */ VectorXd::Zero(b.size()),
                       /*x_hi = */ VectorXd::Zero(b.size()));
}

VectorXd sparse::GaussSeidelIteration(const MatrixXd& A, const VectorXd& b,
                                      const ArrayXb& C, const VectorXd& x_lo,
                                      const VectorXd& x_hi) {
  return BaseIteration(A, b, IterationType::GAUSS_SEIDEL, C, x_lo, x_hi);
}

VectorXd sparse::SORIteration(const MatrixXd& A, const VectorXd& b) {
  return BaseIteration(A, b, IterationType::SOR,
                       /*C = */ ArrayXb::Constant(b.size(), true),
                       /*x_lo = */ VectorXd::Zero(b.size()),
                       /*x_hi = */ VectorXd::Zero(b.size()));
}

VectorXd sparse::SORIteration(const MatrixXd& A, const VectorXd& b,
                              const ArrayXb& C, const VectorXd& x_lo,
                              const VectorXd& x_hi) {
  return BaseIteration(A, b, IterationType::SOR, C, x_lo, x_hi);
}

VectorXd sparse::JacobiIteration(const ConstraintsList& constraints,
                                 const MatrixXd& M_inverse, const VectorXd& rhs,
                                 double cfm) {
  return BaseIteration(constraints, M_inverse, rhs, IterationType::JACOBI, cfm);
}

VectorXd sparse::GaussSeidelIteration(const ConstraintsList& constraints,
                                      const MatrixXd& M_inverse,
                                      const VectorXd& rhs, double cfm) {
  return BaseIteration(constraints, M_inverse, rhs, IterationType::GAUSS_SEIDEL,
                       cfm);
}

VectorXd sparse::SORIteration(const ConstraintsList& constraints,
                              const MatrixXd& M_inverse, const VectorXd& rhs,
                              double cfm) {
  return BaseIteration(constraints, M_inverse, rhs, IterationType::SOR, cfm);
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

// Add constraint force mixing (CFM) to ensemble test cases. The value is high
// so that the test cases will converge within 500 steps.
constexpr double kCfmCoeff = 0.1;

// Helper function to check mixed constraint solutions.
bool CheckMixedConstraintSolutions(const MatrixXd& A, const VectorXd& b,
                                   const VectorXd& x, const ArrayXb& C,
                                   const VectorXd& x_lo, const VectorXd& x_hi) {
  // Check Ax = b for the equality constraints
  const bool check_equality_constraints =
      SelectSubvector(A * x - b, C).norm() < kAllowNumericalError;
  if (!check_equality_constraints) {
    std::cout << "CheckMixedConstraintSolutions: equality check error = "
              << SelectSubvector(A * x - b, C).norm() << std::endl;
    std::cout << "SelectSubvector(A * x - b, C) = \n"
              << SelectSubvector(A * x - b, C) << std::endl;
  }

  // Check Ax <= b (since w >= 0), x > x_lo, and x < x_hi for the ineuqality
  // constraints.
  const VectorXd x_inequality = SelectSubvector(x, !C);
  const VectorXd x_lo_subset = SelectSubvector(x_lo, !C);
  const VectorXd x_hi_subset = SelectSubvector(x_hi, !C);
  const VectorXd w = SelectSubvector(A * x - b, !C);
  bool check_inequality_constraints = true;
  for (int i = 0; i < (!C).count(); ++i) {
    if (x_inequality(i) > x_lo_subset(i) && x_inequality(i) < x_hi(i)) {
      // If x_lo < x < x_hi, then w = 0
      check_inequality_constraints &= std::abs(w(i)) < kAllowNumericalError;
    } else if (x_inequality(i) == x_lo_subset(i)) {
      // If x == x_lo, then w > 0
      check_inequality_constraints &= w(i) > -1.0 * kAllowNumericalError;
    } else if (x_inequality(i) == x_hi_subset(i)) {
      // If x == x_hi, then w < 0
      check_inequality_constraints &= w(i) < kAllowNumericalError;
    } else {
      check_inequality_constraints &= false;
      break;
    }
  }
  if (!check_inequality_constraints) {
    std::cout << "CheckMixedConstraintSolutions: inequality check error."
              << std::endl;
    // std::cout << "x_lo = \n" << SelectSubvector(x_lo, !C) << std::endl;
    // std::cout << "x_hi = \n" << SelectSubvector(x_hi, !C) << std::endl;
    // std::cout << "w = \n" << SelectSubvector(A * x - b, !C) << std::endl;
    // std::cout << "inequality x subvector = \n" << x_inequality << std::endl;
  }

  return check_equality_constraints && check_inequality_constraints;
}

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
        GenerateSPDMatrix(dim) + MatrixXd::Identity(dim, dim) * 1.0;
    const VectorXd b = VectorXd::Random(dim);
    auto x = sparse::GaussSeidelIteration(A, b);
    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A (" << A.rows() << "x" << A.cols()
                << ") = " << GetConditionNumber(A) << std::endl;
    }
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(GaussSeidelIteration_mixed_constraints_no_bounds) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate matrix dimension, between 3 and 50.
    const int dim = rand() % 48 + 3;
    // Generate an SPD matrix, add a little diagonal dominance to decrease the
    // spectral radius of the iteration matrix. Otherwise, the spectral radius
    // is often >0.99, resulting in slow convergence.
    const MatrixXd A =
        GenerateSPDMatrix(dim) + MatrixXd::Identity(dim, dim) * 0.5;
    const VectorXd b = VectorXd::Random(dim);
    const ArrayXb C = ArrayXb::Random(dim);
    // const VectorXd x_lo = VectorXd::Zero(dim);
    const VectorXd x_lo =
        VectorXd::Constant(dim, -1.0 * std::numeric_limits<double>::infinity());
    const VectorXd x_hi =
        VectorXd::Constant(dim, std::numeric_limits<double>::infinity());
    auto x = sparse::GaussSeidelIteration(A, b, C, x_lo, x_hi);

    // Check solutions
    const bool check = CheckMixedConstraintSolutions(A, b, x, C, x_lo, x_hi);
    if (!check) {
      std::cout << "Condition number of A (" << A.rows() << "x" << A.cols()
                << ") = " << GetConditionNumber(A) << std::endl;
    }
    CHECK(check);
  }
}

TEST_FUNCTION(GaussSeidelIteration_mixed_constraints_with_bounds) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate matrix dimension, between 3 and 50.
    const int dim = rand() % 48 + 3;
    // Generate an SPD matrix, add a little diagonal dominance to decrease the
    // spectral radius of the iteration matrix. Otherwise, the spectral radius
    // is often >0.99, resulting in slow convergence.
    const MatrixXd A =
        GenerateSPDMatrix(dim) + MatrixXd::Identity(dim, dim) * 0.5;
    const VectorXd b = VectorXd::Random(dim);
    const ArrayXb C = ArrayXb::Random(dim);
    const VectorXd x_lo = VectorXd::Constant(dim, -0.5);
    const VectorXd x_hi = VectorXd::Constant(dim, 0.5);
    auto x = sparse::GaussSeidelIteration(A, b, C, x_lo, x_hi);

    // Check solutions
    const bool check = CheckMixedConstraintSolutions(A, b, x, C, x_lo, x_hi);
    if (!check) {
      std::cout << "Condition number of A (" << A.rows() << "x" << A.cols()
                << ") = " << GetConditionNumber(A) << std::endl;
    }
    CHECK(check);
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

TEST_FUNCTION(SORIteration_spd_mixed_constraints) {
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate matrix dimension, between 3 and 50.
    const int dim = rand() % 48 + 3;
    // Generate an SPD matrix, add a little diagonal dominance to decrease the
    // spectral radius of the iteration matrix. Otherwise, the spectral radius
    // is often >0.99, resulting in slow convergence.
    const MatrixXd A =
        GenerateSPDMatrix(dim) + MatrixXd::Identity(dim, dim) * 2.0;
    const VectorXd b = VectorXd::Random(dim);
    const ArrayXb C = ArrayXb::Random(dim);
    const VectorXd x_lo = VectorXd::Constant(dim, -10);
    const VectorXd x_hi = VectorXd::Constant(dim, 10);
    auto x = sparse::SORIteration(A, b, C, x_lo, x_hi);

    // Check solutions
    // TODO: all of the above cases also pass Ax=b, when they shouldn't. Why?
    const bool check = CheckMixedConstraintSolutions(A, b, x, C, x_lo, x_hi);
    if (!check) {
      std::cout << "Condition number of A (" << A.rows() << "x" << A.cols()
                << ") = " << GetConditionNumber(A)
                << ", (Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }
    CHECK(check);
  }
}

TEST_FUNCTION(JacobiIteration_ensemble) {
  auto check_jacobi = [](const Ensemble& en, bool sparse = true) {
    const MatrixXd J = en.ComputeJ();
    const MatrixXd M = en.M_inverse();
    MatrixXd JMJt = J * M * J.transpose();
    JMJt += kCfmCoeff * MatrixXd::Identity(JMJt.rows(), JMJt.cols());

    // Use a randomly generated rhs
    const VectorXd rhs = VectorXd::Random(J.rows());

    // Get constraints vectors
    ArrayXb C;
    VectorXd x_lo, x_hi;
    sparse::ConstructMixedConstraints(en.constraints(), &C, &x_lo, &x_hi);

    // Solve (JMJt * x = rhs) using Gause-Seidel iteration
    VectorXd x = VectorXd::Zero(rhs.size());
    if (sparse) {
      x = sparse::JacobiIteration(en.constraints(), en.M_inverse(), rhs,
                                  kCfmCoeff);
    } else {
      x = sparse::JacobiIteration(JMJt, rhs, C, x_lo, x_hi);
    }

    // Check x
    const bool check =
        CheckMixedConstraintSolutions(JMJt, rhs, x, C, x_lo, x_hi);
    return check;
  };

  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  bool sparse_check, dense_check;
  sparse_check = check_jacobi(chain, /*sparse = */ true);
  dense_check = check_jacobi(chain, /*sparse = */ false);
  if (!sparse_check || !dense_check) {
    std::cout << "sparse_check = " << sparse_check << std::endl;
    std::cout << "dense_check = " << dense_check << std::endl;
    CHECK(false);
  }
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    sparse_check = check_jacobi(chain, /*sparse = */
                                true);
    dense_check = check_jacobi(chain, /*sparse = */ false);
    if (!sparse_check || !dense_check) {
      std::cout << "sparse_check = " << sparse_check << std::endl;
      std::cout << "dense_check = " << dense_check << std::endl;
      CHECK(false);
    }
  }

  // Jacobi method does not converge well for mixed constraints, therefore we
  // will not use cairn test with collision constraints.
  // for (int n = 0; n < kNumTestInsts; ++n) {
  //   std::cout << "Test cairns #" << n << std::endl;
  //   Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
  //   cairn.Init();
  //   cairn.InitStabilize();
  //   CHECK(check_jacobi(cairn, /*sparse = */ true));
  //   CHECK(check_jacobi(cairn, /*sparse = */ false));
  //   for (int i = 0; i < kNumSimSteps; ++i) {
  //     cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
  //     CHECK(check_jacobi(cairn, /*sparse = */ true));
  //     CHECK(check_jacobi(cairn, /*sparse = */ false));
  //   }
  // }
}

TEST_FUNCTION(GaussSeidelIteration_ensemble) {
  auto check_gauss_seidel = [](const Ensemble& en, const VectorXd& rhs,
                               bool sparse) {
    const MatrixXd J = en.ComputeJ();
    const MatrixXd M = en.M_inverse();
    const MatrixXd JMJt = J * M * J.transpose() +
                          kCfmCoeff * MatrixXd::Identity(J.rows(), J.rows());

    // Get constraints vectors
    ArrayXb C;
    VectorXd x_lo, x_hi;
    sparse::ConstructMixedConstraints(en.constraints(), &C, &x_lo, &x_hi);

    // Solve (JMJt * x = rhs) using Gause-Seidel iteration
    VectorXd x = VectorXd::Zero(rhs.size());
    if (sparse) {
      x = sparse::GaussSeidelIteration(en.constraints(), en.M_inverse(), rhs,
                                       kCfmCoeff);
    } else {
      x = sparse::GaussSeidelIteration(JMJt, rhs, C, x_lo, x_hi);
    }

    // Check x
    const bool check =
        CheckMixedConstraintSolutions(JMJt, rhs, x, C, x_lo, x_hi);
    if (!check) {
      std::cout << "Condition number of JMJt = " << GetConditionNumber(JMJt)
                << std::endl;
    }
    return check;
  };

  bool sparse_check, dense_check;
  VectorXd rhs;

  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();

  // Use a randomly generated rhs
  rhs = VectorXd::Random(chain.ComputeJ().rows());
  sparse_check = check_gauss_seidel(chain, rhs, /*sparse = */ true);
  dense_check = check_gauss_seidel(chain, rhs, /*sparse = */ false);
  if (!sparse_check || !dense_check) {
    std::cout << "sparse_check = " << sparse_check << std::endl;
    std::cout << "dense_check = " << dense_check << std::endl;
    CHECK(false);
  }
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    rhs = VectorXd::Random(chain.ComputeJ().rows());
    sparse_check = check_gauss_seidel(chain, rhs, /*sparse = */ true);
    dense_check = check_gauss_seidel(chain, rhs, /*sparse = */ false);
    if (!sparse_check || !dense_check) {
      std::cout << "sparse_check = " << sparse_check << std::endl;
      std::cout << "dense_check = " << dense_check << std::endl;
      CHECK(false);
    }
  }

  for (int n = 0; n < kNumTestInsts; ++n) {
    std::cout << "Gauss-Seidel cairn test " << n << std::endl;
    Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
    cairn.Init();
    cairn.InitStabilize();
    rhs = VectorXd::Random(cairn.ComputeJ().rows());
    sparse_check = check_gauss_seidel(cairn, rhs, /*sparse = */ true);
    dense_check = check_gauss_seidel(cairn, rhs, /*sparse = */ false);
    if (!sparse_check || !dense_check) {
      std::cout << "sparse_check = " << sparse_check << std::endl;
      std::cout << "dense_check = " << dense_check << std::endl;
      CHECK(false);
    }
    for (int i = 0; i < kNumSimSteps; ++i) {
      cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
      rhs = VectorXd::Random(cairn.ComputeJ().rows());
      sparse_check = check_gauss_seidel(cairn, rhs, /*sparse = */ true);
      dense_check = check_gauss_seidel(cairn, rhs, /*sparse = */ false);
      if (!sparse_check || !dense_check) {
        std::cout << "sparse_check = " << sparse_check << std::endl;
        std::cout << "dense_check = " << dense_check << std::endl;
        CHECK(false);
      }
    }
  }
}

TEST_FUNCTION(SORIteration_ensemble) {
  auto check_sor = [](const Ensemble& en, const VectorXd& rhs, bool sparse) {
    const MatrixXd J = en.ComputeJ();
    const MatrixXd M = en.M_inverse();
    const MatrixXd JMJt = J * M * J.transpose() +
                          kCfmCoeff * MatrixXd::Identity(J.rows(), J.rows());

    // Get constraints vectors
    ArrayXb C;
    VectorXd x_lo, x_hi;
    sparse::ConstructMixedConstraints(en.constraints(), &C, &x_lo, &x_hi);

    // Solve (JMJt * x = rhs) using SOR iteration
    VectorXd x = VectorXd::Zero(rhs.size());
    if (sparse) {
      x = sparse::SORIteration(en.constraints(), en.M_inverse(), rhs,
                               kCfmCoeff);
    } else {
      x = sparse::SORIteration(JMJt, rhs, C, x_lo, x_hi);
    }

    // Check x
    const bool check =
        CheckMixedConstraintSolutions(JMJt, rhs, x, C, x_lo, x_hi);
    return check;
  };

  bool sparse_check, dense_check;
  VectorXd rhs;

  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();

  rhs = VectorXd::Random(chain.ComputeJ().rows());
  sparse_check = check_sor(chain, rhs, /*sparse = */ true);
  dense_check = check_sor(chain, rhs, /*sparse = */ false);
  if (!sparse_check || !dense_check) {
    std::cout << "sparse_check = " << sparse_check << std::endl;
    std::cout << "dense_check = " << dense_check << std::endl;
    CHECK(false);
  }
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    rhs = VectorXd::Random(chain.ComputeJ().rows());
    sparse_check = check_sor(chain, rhs, /*sparse = */ true);
    dense_check = check_sor(chain, rhs, /*sparse = */ false);
    if (!sparse_check || !dense_check) {
      std::cout << "sparse_check = " << sparse_check << std::endl;
      std::cout << "dense_check = " << dense_check << std::endl;
      CHECK(false);
    }
  }

  for (int n = 0; n < kNumTestInsts; ++n) {
    std::cout << "SOR cairn test " << n << std::endl;
    Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
    cairn.Init();
    cairn.InitStabilize();
    rhs = VectorXd::Random(cairn.ComputeJ().rows());
    sparse_check = check_sor(cairn, rhs, /*sparse = */ true);
    dense_check = check_sor(cairn, rhs, /*sparse = */ false);
    if (!sparse_check || !dense_check) {
      std::cout << "sparse_check = " << sparse_check << std::endl;
      std::cout << "dense_check = " << dense_check << std::endl;
      CHECK(false);
    }
    for (int i = 0; i < kNumSimSteps; ++i) {
      cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
      rhs = VectorXd::Random(cairn.ComputeJ().rows());
      sparse_check = check_sor(cairn, rhs, /*sparse = */ true);
      dense_check = check_sor(cairn, rhs, /*sparse = */ false);
      if (!sparse_check || !dense_check) {
        std::cout << "sparse_check = " << sparse_check << std::endl;
        std::cout << "dense_check = " << dense_check << std::endl;
        CHECK(false);
      }
    }
  }
}

}  // namespace
