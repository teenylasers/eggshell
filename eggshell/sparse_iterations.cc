#include "sparse_iterations.h"

#include <iostream>

#include "constants.h"
#include "error.h"

namespace {

// For successive-over-relaxation (SOR),
//  - forward impl: M = (1/omega) * D + U, N = (1/omega - 1) * D + U.
//  - backward impl: M = (1/omega) * D + L, N = (1/omega - 1) * D + L.
// kSOR = 1/omega.
constexpr double omega = 1.5;  // 0 < omega < 2 for convergence.
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

  // Function pointer to the matrix solver, depending on whether the matrix
  // structure is diagonal, lower, or upper triangular.
  sparse::matrix_solver solver;

  // x0 to start the iterations
  VectorXd x = VectorXd::Random(dim);

  switch (type) {
    case IterationType::JACOBI:
      M.diagonal() = A.diagonal();
      N = -1 * (MatrixXd(A.triangularView<Eigen::StrictlyLower>()) +
                MatrixXd(A.triangularView<Eigen::StrictlyUpper>()));
      solver = &MatrixSolveDiagonal;
      break;
    case IterationType::GAUSS_SEIDEL:
      M = A.triangularView<Eigen::Lower>();
      N = -1 * MatrixXd(A.triangularView<Eigen::StrictlyUpper>());
      solver = &MatrixSolveLowerTriangle;
      break;
    case IterationType::SOR:
      // Using backward SOR.
      M = A.triangularView<Eigen::Upper>();
      M.diagonal() = kSOR * A.diagonal();
      N = -1 * MatrixXd(A.triangularView<Eigen::Lower>());
      N.diagonal() = (kSOR - 1) * A.diagonal();
      solver = &MatrixSolveUpperTriangle;
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
    x = solver(M, rhs);
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

VectorXd sparse::CalculateBlockLx(const ConstraintsList& constraints,
                                  const MatrixXd& M_inverse,
                                  const VectorXd& x) {
  VectorXd Lx = VectorXd::Zero(x.size());
  int Lx_row_index = 0;

  for (int i = 1; i < constraints.size(); ++i) {
    MatrixXd j_i0, j_i1;  // jacobians for the 2 component of this constraint
    ArrayXb ct;           // unused
    VectorXd c_lo, c_hi;  // unused
    constraints.at(i)->ComputeJ(&j_i0, &j_i1, &ct, &c_lo, &c_hi);
    int i0 = constraints.at(i)->i0_;  // component 0's global index
    int i1 = constraints.at(i)->i1_;  // component 1's global index

    Lx_row_index += j_i0.rows();
    int x_row_index = 0;

    for (int j = 0; j < i; ++j) {
      MatrixXd j_j0, j_j1;  // jacobians for the 2 components in the constraint
      constraints.at(j)->ComputeJ(&j_j0, &j_j1, &ct, &c_lo, &c_hi);
      int j0 = constraints.at(j)->i0_;  // component 0's global index
      int j1 = constraints.at(j)->i1_;  // component 1's global index

      // Form the (i,j)-th subblock of the JMJt systems matrix
      MatrixXd JMJt = MatrixXd::Zero(j_i0.rows(), j_j0.rows());
      if (i0 == j0 && i0 >= 0) {
        JMJt += j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_j0.transpose();
      } else if (i0 == j1 && i0 >= 0) {
        JMJt += j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_j1.transpose();
      }
      if (i1 == j0 && i1 >= 0) {
        JMJt += j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_j0.transpose();
      } else if (i1 == j1 && i1 >= 0) {
        JMJt += j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_j1.transpose();
      }

      /*
      std::cout << "Constraints indices i, j = " << i << ", " << j << std::endl;
      std::cout << "Components indices = (i0, i1), (j0, j1) = (" << i0 << ", "
                << i1 << "), (" << j0 << ", " << j1 << ")\n";
      std::cout << "JMJt = \n" << JMJt << std::endl;
      std::cout << "Lx_row_index = " << Lx_row_index << std::endl;
      std::cout << "x_row_index" << x_row_index << std::endl;
      */

      // Accumulate the i-th subblock of Lx
      Lx.block(Lx_row_index, 0, j_i0.rows(), 1) +=
          JMJt * x.block(x_row_index, 0, JMJt.cols(), 1);

      // Update x_row_index for subvector multiplication with this JMJt
      // submatrix.
      x_row_index += j_i0.rows();
    }
  }

  return Lx;
}

VectorXd sparse::CalculateBlockUx(const ConstraintsList& constraints,
                                  const MatrixXd& M_inverse,
                                  const VectorXd& x) {
  VectorXd Ux = VectorXd::Zero(x.size());
  int Ux_row_index = 0;

  for (int i = 0; i < constraints.size(); ++i) {
    MatrixXd j_i0, j_i1;  // jacobians for the 2 component of this constraint
    ArrayXb ct;           // unused
    VectorXd c_lo, c_hi;  // unused
    constraints.at(i)->ComputeJ(&j_i0, &j_i1, &ct, &c_lo, &c_hi);
    int i0 = constraints.at(i)->i0_;  // component 0's global index
    int i1 = constraints.at(i)->i1_;  // component 1's global index

    int x_row_index = Ux_row_index;

    for (int j = i + 1; j < constraints.size(); ++j) {
      MatrixXd j_j0, j_j1;  // jacobians for the 2 components in the constraint
      constraints.at(j)->ComputeJ(&j_j0, &j_j1, &ct, &c_lo, &c_hi);
      int j0 = constraints.at(j)->i0_;  // component 0's global index
      int j1 = constraints.at(j)->i1_;  // component 1's global index

      // Form the (i,j)-th subblock of the JMJt systems matrix
      MatrixXd JMJt = MatrixXd::Zero(j_i0.rows(), j_j0.rows());
      if (i0 == j0 && i0 >= 0) {
        JMJt += j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_j0.transpose();
      } else if (i0 == j1 && i0 >= 0) {
        JMJt += j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_j1.transpose();
      }
      if (i1 == j0 && i1 >= 0) {
        JMJt += j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_j0.transpose();
      } else if (i1 == j1 && i1 >= 0) {
        JMJt += j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_j1.transpose();
      }

      /*
      std::cout << "Constraints indices i, j = " << i << ", " << j << std::endl;
      std::cout << "Components indices = (i0, i1), (j0, j1) = (" << i0 << ", "
                << i1 << "), (" << j0 << ", " << j1 << ")\n";
      std::cout << "JMJt = \n" << JMJt << std::endl;
      */

      // Update x_row_index for subvector multiplication with this JMJt
      // submatrix.
      x_row_index += j_i0.rows();

      // Accumulate the i-th subblock of Ux
      Ux.block(Ux_row_index, 0, j_i0.rows(), 1) +=
          JMJt * x.block(x_row_index, 0, JMJt.cols(), 1);
    }

    // Update Dx_row_index for the next constraint
    Ux_row_index += j_i0.rows();
  }

  return Ux;
}

VectorXd sparse::CalculateBlockDx(const ConstraintsList& constraints,
                                  const MatrixXd& M_inverse,
                                  const VectorXd& x) {
  VectorXd Dx = VectorXd::Zero(x.size());
  int Dx_row_index = 0;

  for (int i = 0; i < constraints.size(); ++i) {
    MatrixXd j0, j1;
    ArrayXb ct;           // unused
    VectorXd c_lo, c_hi;  // unused
    constraints.at(i)->ComputeJ(&j0, &j1, &ct, &c_lo, &c_hi);
    int i0 = constraints.at(i)->i0_;
    int i1 = constraints.at(i)->i1_;

    // Accumulate JMJt is i0 or i1 is not -1. Negative index indicates joint
    // constraint with an anchor or contact constraint with ground.
    MatrixXd JMJt = MatrixXd::Zero(j0.rows(), j0.rows());
    if (i0 >= 0) {
      JMJt = JMJt + j0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j0.transpose();
    }
    if (i1 >= 0) {
      JMJt = JMJt + j1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j1.transpose();
    }
    Dx.block(Dx_row_index, 0, j0.rows(), 1) =
        JMJt * x.block(Dx_row_index, 0, j0.rows(), 1);

    // Update Dx_row_index for the next constraint
    Dx_row_index += j0.rows();
  }

  return Dx;
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

// Compare (JMJT_block_diagonal * x) when calculated using CalculateBlockDx()
// versus forming the JMJt systems matrix.
bool CompareBlockDx(const Ensemble& en, int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();

  // Use a randomly generated x
  const VectorXd x = VectorXd::Random(J.rows());

  // Calculate block Dx using CalculateBlockDx()
  const VectorXd Dx = sparse::CalculateBlockDx(en.constraints(), M, x);

  // Calculate block Dx by forming JMJt the systems matrix.
  const MatrixXd JMJt = J * M * J.transpose();
  MatrixXd JMJt_block_diagonal = MatrixXd::Zero(JMJt.rows(), JMJt.cols());
  for (int i = 0; i < JMJt.rows() / dof_per_constraint; ++i) {
    JMJt_block_diagonal.block(i * dof_per_constraint, i * dof_per_constraint,
                              dof_per_constraint, dof_per_constraint) =
        JMJt.block(i * dof_per_constraint, i * dof_per_constraint,
                   dof_per_constraint, dof_per_constraint);
  }
  const VectorXd JMJt_Dx = JMJt_block_diagonal * x;

  // std::cout << "x =\n" << x << std::endl;
  // std::cout << "JMJt_block_diagonal = \n" << JMJt_block_diagonal <<
  // std::endl; std::cout << "Dx = \n" << Dx << std::endl; std::cout << "JMJt_Dx
  // = \n" << JMJt_Dx << std::endl;
  std::cout << "(Dx - JMJt_Dx).norm() = " << (Dx - JMJt_Dx).norm() << std::endl;
  return (Dx - JMJt_Dx).norm() < kAllowNumericalError;
}

// Compare (JMJt_block_upper_triangle * x) when calculated using
// CalculateBlockUx() versus forming the JMJt systems matrix.
bool CompareBlockUx(const Ensemble& en, int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();

  // Use a randomly generated x
  const VectorXd x = VectorXd::Random(J.rows());

  // Calculate block Ux using CalculateBlockUx()
  const VectorXd Ux = sparse::CalculateBlockUx(en.constraints(), M, x);

  // Calculate block Ux by forming JMJt the systems matrix.
  const MatrixXd JMJt = J * M * J.transpose();
  MatrixXd JMJt_block_upper_triangle = JMJt.triangularView<Eigen::Upper>();
  for (int i = 0; i < JMJt.rows() / dof_per_constraint; ++i) {
    JMJt_block_upper_triangle.block(i * dof_per_constraint,
                                    i * dof_per_constraint, dof_per_constraint,
                                    dof_per_constraint) =
        MatrixXd::Zero(dof_per_constraint, dof_per_constraint);
  }
  const VectorXd JMJt_Ux = JMJt_block_upper_triangle * x;

  // std::cout << "x = \n" << x << std::endl;
  // std::cout << "JMJt_block_upper_triangle = \n"
  //           << JMJt_block_upper_triangle << std::endl;
  // std::cout << "Ux = \n" << Ux << std::endl;
  // std::cout << "JMJt_Ux = \n" << JMJt_Ux << std::endl;
  std::cout << "(Ux - JMJt_Ux).norm() = " << (Ux - JMJt_Ux).norm() << std::endl;
  return (Ux - JMJt_Ux).norm() < kAllowNumericalError;
}

// Compare (JMJt_block_lower_triangle * x) when calculated using
// CalculateBlockLx() versus forming the JMJt systems matrix.
bool CompareBlockLx(const Ensemble& en, int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();

  // Use a randomly generated x
  const VectorXd x = VectorXd::Random(J.rows());

  // Calculate block Lx using CalculateBlockLx()
  const VectorXd Lx = sparse::CalculateBlockLx(en.constraints(), M, x);

  // Calculate block Lx by forming JMJt the systems matrix.
  const MatrixXd JMJt = J * M * J.transpose();
  MatrixXd JMJt_block_upper_triangle = JMJt.triangularView<Eigen::Lower>();
  for (int i = 0; i < JMJt.rows() / dof_per_constraint; ++i) {
    JMJt_block_upper_triangle.block(i * dof_per_constraint,
                                    i * dof_per_constraint, dof_per_constraint,
                                    dof_per_constraint) =
        MatrixXd::Zero(dof_per_constraint, dof_per_constraint);
  }
  const VectorXd JMJt_Lx = JMJt_block_upper_triangle * x;

  // std::cout << "x = \n" << x << std::endl;
  // std::cout << "JMJt_block_upper_triangle = \n"
  //           << JMJt_block_upper_triangle << std::endl;
  // std::cout << "Lx = \n" << Lx << std::endl;
  // std::cout << "JMJt_Lx = \n" << JMJt_Lx << std::endl;
  std::cout << "(Lx - JMJt_Lx).norm() = " << (Lx - JMJt_Lx).norm() << std::endl;
  return (Lx - JMJt_Lx).norm() < kAllowNumericalError;
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

// Test CalculateBlockDx on a hanging chain ensemble.
TEST_FUNCTION(CalculateBlockDx_chain) {
  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(CompareBlockDx(chain, 3));

  // Step the ensemble, check sparse::Dx calculation against using a fully
  // formed JMJt systems matrix.
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockDx(chain, 3));
  }
}

// Test CalculateBlockDx on a falling cairn ensemble.
TEST_FUNCTION(CalculateBlockDx_cairn) {
  Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
  cairn.Init();
  cairn.InitStabilize();
  CHECK(CompareBlockDx(cairn, 3));

  // Step the ensemble, check sparse::Dx calculation against using a fully
  // formed JMJt systems matrix
  for (int i = 0; i < kNumSimSteps; ++i) {
    cairn.Step(kSimTimeStep * 5, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockDx(cairn, 3));
  }
}

// Test CalculateBlockUx on a hanging chain ensemble.
TEST_FUNCTION(CalculateBlockUx_chain) {
  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(CompareBlockUx(chain, 3));

  // Step the ensemble, check sparse::Dx calculation against using a fully
  // formed JMJt systems matrix.
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockUx(chain, 3));
  }
}

// Test CalculateBlockUx on a falling cairn ensemble.
TEST_FUNCTION(CalculateBlockUx_cairn) {
  Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
  cairn.Init();
  cairn.InitStabilize();
  CHECK(CompareBlockUx(cairn, 3));

  // Step the ensemble, check sparse::Dx calculation against using a fully
  // formed JMJt systems matrix.
  for (int i = 0; i < kNumSimSteps; ++i) {
    cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockUx(cairn, 3));
  }
}

// Test CalculateBlockLx on a hanging chain ensemble.
TEST_FUNCTION(CalculateBlockLx_chain) {
  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(CompareBlockLx(chain, 3));

  // Step the ensemble, check sparse::Dx calculation against using a fully
  // formed JMJt systems matrix.
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockLx(chain, 3));
  }
}

// Test CalculateBlockLx on a falling cairn ensemble.
TEST_FUNCTION(CalculateBlockLx_cairn) {
  Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
  cairn.Init();
  cairn.InitStabilize();
  CHECK(CompareBlockLx(cairn, 3));

  // Step the ensemble, check sparse::Dx calculation against using a fully
  // formed JMJt systems matrix.
  for (int i = 0; i < kNumSimSteps; ++i) {
    cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockLx(cairn, 3));
  }
}

}  // namespace
