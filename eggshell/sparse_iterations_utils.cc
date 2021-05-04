#include "sparse_iterations_utils.h"

#include <iostream>

#include "constants.h"
#include "error.h"

namespace {

// If inequality constraint applies (i.e. !C) and x is outside of the
// x_lo/x_hi limits, project x onto x_lo or x_hi, whichever is nearer.
double ApplyProjection(double x, bool C, double x_lo, double x_hi) {
  if (!C) {
    if (x < x_lo) {
      return x_lo;
    } else if (x > x_hi) {
      return x_hi;
    }
  }
  return x;
}

}  // namespace

VectorXd sparse::MatrixSolveDiagonal(const MatrixXd& D, const VectorXd& rhs,
                                     const ArrayXb& C, const VectorXd& x_lo,
                                     const VectorXd& x_hi) {
  CHECK(D.rows() == D.cols());
  CHECK_MSG(D.diagonal().prod() != 0, "det(D)==0, D is not invertible.");
  VectorXd x = 1.0 / D.diagonal().array() * rhs.array();
  // Project x for inequality constraints
  for (int i = 0; i < rhs.size(); ++i) {
    x(i) = ApplyProjection(x(i), C(i), x_lo(i), x_hi(i));
  }
  return x;
}

VectorXd sparse::MatrixSolveBlockDiagonal(const MatrixXd& D,
                                          const VectorXd& rhs, const ArrayXb& C,
                                          const VectorXd& x_lo,
                                          const VectorXd& x_hi, int block_dim) {
  CHECK(rhs.size() % block_dim == 0);
  CHECK(block_dim <= 3 && block_dim > 0);
  VectorXd x = VectorXd::Zero(rhs.size());
  int x_row_index = 0;

  for (int i = 0; i < rhs.size() / block_dim; ++i) {
    MatrixXd subD = D.block(i * block_dim, i * block_dim, block_dim, block_dim);
    // Use subD.inverse() instead of factorization, because we make no
    // assumption about whether D is symmetric or its definiteness.
    x.block(x_row_index, 0, block_dim, 1) =
        subD.inverse() * rhs.block(x_row_index, 0, block_dim, 1);

    // Project x for inequality constraints
    for (int k = 0; k < block_dim; ++k) {
      int x_index = x_row_index + k;
      x(x_index) =
          ApplyProjection(x(x_index), C(x_index), x_lo(x_index), x_hi(x_index));
    }

    x_row_index += block_dim;
  }

  return x;
}

VectorXd sparse::MatrixSolveBlockDiagonal(const ConstraintsList& constraints,
                                          const MatrixXd& M_inverse,
                                          const VectorXd& rhs, double epsilon,
                                          double scale) {
  VectorXd x = VectorXd::Zero(rhs.size());
  int x_row_index = 0;

  for (int i = 0; i < constraints.size(); ++i) {
    MatrixXd j0, j1;
    ArrayXb ct;
    VectorXd c_lo, c_hi;
    constraints.at(i)->ComputeJ(&j0, &j1, &ct, &c_lo, &c_hi);
    int i0 = constraints.at(i)->i0_;
    int i1 = constraints.at(i)->i1_;

    // Accumulate subD, the diagonal subblock, if i0 or i1 is not -1. Negative
    // index indicates joint constraint with an anchor or contact constraint
    // with ground.
    MatrixXd subD = MatrixXd::Zero(j0.rows(), j0.rows());
    if (i0 >= 0) {
      subD += j0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j0.transpose();
    }
    if (i1 >= 0) {
      subD += j1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j1.transpose();
    }
    subD += epsilon * MatrixXd::Identity(subD.rows(), subD.cols());
    subD.diagonal() = subD.diagonal().array() * scale;

    x.block(x_row_index, 0, j0.rows(), 1) =
        1.0 / subD.diagonal().array() *
        rhs.block(x_row_index, 0, j0.rows(), 1).array();
    for (int k = 0; k < ct.size(); ++k) {
      x(x_row_index + k) =
          ApplyProjection(x(x_row_index + k), ct(k), c_lo(k), c_hi(k));
    }

    // Update Dx_row_index for the next constraint
    x_row_index += j0.rows();
  }

  return x;
}

VectorXd sparse::MatrixSolveLowerTriangle(const MatrixXd& L,
                                          const VectorXd& rhs, const ArrayXb& C,
                                          const VectorXd& x_lo,
                                          const VectorXd& x_hi) {
  CHECK(L.rows() == L.cols() && L.rows() == rhs.size());
  CHECK_MSG(L.diagonal().prod() != 0, "L is not invertible.");
  const int dim = rhs.size();
  VectorXd x = VectorXd::Zero(dim);
  for (int i = 0; i < dim; ++i) {
    double substitutions = 0;
    for (int j = 0; j < i; ++j) {
      substitutions += L(i, j) * x(j);
    }
    x(i) = ApplyProjection((rhs(i) - substitutions) / L(i, i), C(i), x_lo(i),
                           x_hi(i));
  }

  return x;
}

VectorXd sparse::MatrixSolveBlockLowerTriangle(
    const MatrixXd& L, const VectorXd& rhs, const ArrayXb& C,
    const VectorXd& x_lo, const VectorXd& x_hi, int block_dim) {
  CHECK(rhs.size() % block_dim == 0);
  VectorXd x = VectorXd::Zero(rhs.size());

  for (int i = 0; i < rhs.size() / block_dim; ++i) {
    VectorXd substitutions = VectorXd::Zero(block_dim);
    for (int j = 0; j < i; ++j) {
      MatrixXd subL =
          L.block(i * block_dim, j * block_dim, block_dim, block_dim);
      substitutions += subL * x.block(j * block_dim, 0, block_dim, 1);
    }
    MatrixXd subD = L.block(i * block_dim, i * block_dim, block_dim, block_dim);
    x.block(i * block_dim, 0, block_dim, 1) =
        subD.inverse() *
        (rhs.block(i * block_dim, 0, block_dim, 1) - substitutions);

    // Project x for inequality constraints
    for (int k = 0; k < block_dim; ++k) {
      int x_index = i * block_dim + k;
      x(x_index) =
          ApplyProjection(x(x_index), C(x_index), x_lo(x_index), x_hi(x_index));
    }
  }

  return x;
}

VectorXd sparse::MatrixSolveBlockLowerTriangle(
    const ConstraintsList& constraints, const MatrixXd& M_inverse,
    const VectorXd& rhs, double epsilon_diagonal, double scale_diagonal) {
  VectorXd x = VectorXd::Zero(rhs.size());
  int x_row_index = 0;

  for (int i = 0; i < constraints.size(); ++i) {
    MatrixXd j_i0, j_i1;  // jacobians for the 2 component of this constraint
    ArrayXb ct;
    VectorXd c_lo, c_hi;
    constraints.at(i)->ComputeJ(&j_i0, &j_i1, &ct, &c_lo, &c_hi);
    int i0 = constraints.at(i)->i0_;  // component 0's global index
    int i1 = constraints.at(i)->i1_;  // component 1's global index

    // Accumulate substitutions from off-diagonal lower-triangle blocks.
    int rhs_row_index = 0;
    VectorXd substitutions = VectorXd::Zero(j_i0.rows());

    for (int j = 0; j < i; ++j) {
      MatrixXd j_j0,
          j_j1;  // jacobians for the 2 components in the constraint
      constraints.at(j)->ComputeJ(&j_j0, &j_j1, &ct, &c_lo, &c_hi);
      int j0 = constraints.at(j)->i0_;  // component 0's global index
      int j1 = constraints.at(j)->i1_;  // component 1's global index

      // Form the (i,j)-th subblock of the JMJt systems matrix
      MatrixXd subblock = MatrixXd::Zero(j_i0.rows(), j_j0.rows());
      if (i0 == j0 && i0 >= 0) {
        subblock +=
            j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_j0.transpose();
      } else if (i0 == j1 && i0 >= 0) {
        subblock +=
            j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_j1.transpose();
      }
      if (i1 == j0 && i1 >= 0) {
        subblock +=
            j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_j0.transpose();
      } else if (i1 == j1 && i1 >= 0) {
        subblock +=
            j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_j1.transpose();
      }

      // Accumulate substitutions
      substitutions += subblock * x.block(rhs_row_index, 0, subblock.cols(), 1);

      // Update rhs_row_index for subvector multiplication with this JMJt
      // submatrix.
      rhs_row_index += j_j0.rows();
    }

    // Accumulate diagonal_block if i0 or i1 is not -1. Negative index
    // indicates joint constraint with an anchor or contact constraint with
    // ground.
    const int diagonal_dim = i0 >= i1 ? j_i0.rows() : j_i1.rows();
    MatrixXd diagonal_block = MatrixXd::Zero(diagonal_dim, diagonal_dim);
    if (i0 >= 0) {
      diagonal_block +=
          j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_i0.transpose();
    }
    if (i1 >= 0) {
      diagonal_block +=
          j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_i1.transpose();
    }
    diagonal_block +=
        epsilon_diagonal *
        MatrixXd::Identity(diagonal_block.rows(), diagonal_block.cols());
    diagonal_block.diagonal() =
        diagonal_block.diagonal().array() * scale_diagonal;

    // Do substitution line by line
    for (int k = 0; k < ct.size(); ++k) {
      for (int l = 0; l < k; ++l) {
        substitutions(k) += diagonal_block(k, l) * x(x_row_index + l);
      }
      x(x_row_index + k) = ApplyProjection(
          (rhs(x_row_index + k) - substitutions(k)) / diagonal_block(k, k),
          ct(k), c_lo(k), c_hi(k));
    }

    // Update x_row_index
    x_row_index += j_i0.rows();
  }

  return x;
}

VectorXd sparse::MatrixSolveUpperTriangle(const MatrixXd& U,
                                          const VectorXd& rhs, const ArrayXb& C,
                                          const VectorXd& x_lo,
                                          const VectorXd& x_hi) {
  CHECK(U.rows() == U.cols() && U.rows() == rhs.size());
  CHECK_MSG(U.diagonal().prod() != 0, "U is not invertible.");
  const int dim = rhs.size();
  VectorXd x = VectorXd::Zero(dim);
  for (int i = dim - 1; i >= 0; --i) {
    double substitutions = 0;
    for (int j = i + 1; j < dim; ++j) {
      substitutions += U(i, j) * x(j);
    }
    x(i) = ApplyProjection((rhs(i) - substitutions) / U(i, i), C(i), x_lo(i),
                           x_hi(i));
  }
  return x;
}

VectorXd sparse::MatrixSolveBlockUpperTriangle(
    const MatrixXd& U, const VectorXd& rhs, const ArrayXb& C,
    const VectorXd& x_lo, const VectorXd& x_hi, int block_dim) {
  CHECK(rhs.size() % block_dim == 0);
  VectorXd x = VectorXd::Zero(rhs.size());

  for (int i = rhs.size() / block_dim - 1; i >= 0; --i) {
    VectorXd substitutions = VectorXd::Zero(block_dim);
    for (int j = i + 1; j < rhs.size() / block_dim; ++j) {
      MatrixXd subU =
          U.block(i * block_dim, j * block_dim, block_dim, block_dim);
      substitutions += subU * x.block(j * block_dim, 0, block_dim, 1);
    }
    MatrixXd subD = U.block(i * block_dim, i * block_dim, block_dim, block_dim);
    x.block(i * block_dim, 0, block_dim, 1) =
        subD.inverse() *
        (rhs.block(i * block_dim, 0, block_dim, 1) - substitutions);

    // Project x for inequality constraints
    for (int k = 0; k < block_dim; ++k) {
      int x_index = i * block_dim + k;
      x(x_index) =
          ApplyProjection(x(x_index), C(x_index), x_lo(x_index), x_hi(x_index));
    }
  }
  return x;
}

VectorXd sparse::MatrixSolveBlockUpperTriangle(
    const ConstraintsList& constraints, const MatrixXd& M_inverse,
    const VectorXd& rhs, double epsilon_diagonal, double scale_diagonal) {
  VectorXd x = VectorXd::Zero(rhs.size());
  int x_row_index = rhs.size();

  for (int i = constraints.size() - 1; i >= 0; --i) {
    MatrixXd j_i0, j_i1;  // jacobians for the 2 component of this constraint
    ArrayXb ct;
    VectorXd c_lo, c_hi;
    constraints.at(i)->ComputeJ(&j_i0, &j_i1, &ct, &c_lo, &c_hi);
    int i0 = constraints.at(i)->i0_;  // component 0's global index
    int i1 = constraints.at(i)->i1_;  // component 1's global index

    x_row_index -= j_i0.rows();

    // Accumulate substitutions from off-diagonal lower-triangle blocks.
    VectorXd substitutions = VectorXd::Zero(j_i0.rows());
    int rhs_row_index = rhs.size();

    for (int j = constraints.size() - 1; j > i; --j) {
      MatrixXd j_j0,
          j_j1;  // jacobians for the 2 components in the constraint
      constraints.at(j)->ComputeJ(&j_j0, &j_j1, &ct, &c_lo, &c_hi);
      int j0 = constraints.at(j)->i0_;  // component 0's global index
      int j1 = constraints.at(j)->i1_;  // component 1's global index

      // Update rhs_row_index for subvector multiplication with this JMJt
      // submatrix.
      rhs_row_index -= j_j0.rows();

      // Form the (i,j)-th subblock of the JMJt systems matrix
      MatrixXd subblock = MatrixXd::Zero(j_i0.rows(), j_j0.rows());
      if (i0 == j0 && i0 >= 0) {
        subblock +=
            j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_j0.transpose();
      } else if (i0 == j1 && i0 >= 0) {
        subblock +=
            j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_j1.transpose();
      }
      if (i1 == j0 && i1 >= 0) {
        subblock +=
            j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_j0.transpose();
      } else if (i1 == j1 && i1 >= 0) {
        subblock +=
            j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_j1.transpose();
      }
      // Accumulate substitutions
      substitutions += subblock * x.block(rhs_row_index, 0, subblock.cols(), 1);
    }

    // Accumulate diagonal_block if i0 or i1 is not -1. Negative index
    // indicates joint constraint with an anchor or contact constraint with
    // ground.
    MatrixXd diagonal_block = MatrixXd::Zero(j_i0.rows(), j_i0.rows());
    if (i0 >= 0) {
      diagonal_block +=
          j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_i0.transpose();
    }
    if (i1 >= 0) {
      diagonal_block +=
          j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_i1.transpose();
    }
    diagonal_block +=
        epsilon_diagonal *
        MatrixXd::Identity(diagonal_block.rows(), diagonal_block.cols());
    diagonal_block.diagonal() =
        diagonal_block.diagonal().array() * scale_diagonal;

    // Do substitution line by line
    for (int k = ct.size() - 1; k >= 0; --k) {
      for (int l = k + 1; l < ct.size(); ++l) {
        substitutions(k) += diagonal_block(k, l) * x(x_row_index + l);
      }
      x(x_row_index + k) = ApplyProjection(
          (rhs(x_row_index + k) - substitutions(k)) / diagonal_block(k, k),
          ct(k), c_lo(k), c_hi(k));
    }
  }

  return x;
}

VectorXd sparse::MatrixSolveDiagonal(const MatrixXd& D, const VectorXd& rhs) {
  return MatrixSolveDiagonal(D, rhs,
                             /*C = */ ArrayXb::Constant(rhs.size(), true),
                             /*x_lo = */ VectorXd::Zero(rhs.size()),
                             /*x_hi = */ VectorXd::Zero(rhs.size()));
}

VectorXd sparse::MatrixSolveBlockDiagonal(const MatrixXd& D,
                                          const VectorXd& rhs, int block_dim) {
  return MatrixSolveBlockDiagonal(D, rhs,
                                  /*C = */ ArrayXb::Constant(rhs.size(), true),
                                  /*x_lo = */ VectorXd::Zero(rhs.size()),
                                  /*x_hi = */ VectorXd::Zero(rhs.size()),
                                  block_dim);
}

VectorXd sparse::MatrixSolveLowerTriangle(const MatrixXd& D,
                                          const VectorXd& rhs) {
  return MatrixSolveLowerTriangle(D, rhs,
                                  /*C = */ ArrayXb::Constant(rhs.size(), true),
                                  /*x_lo = */ VectorXd::Zero(rhs.size()),
                                  /*x_hi = */ VectorXd::Zero(rhs.size()));
}

VectorXd sparse::MatrixSolveBlockLowerTriangle(const MatrixXd& D,
                                               const VectorXd& rhs,
                                               int block_dim) {
  return MatrixSolveBlockLowerTriangle(
      D, rhs, /*C = */ ArrayXb::Constant(rhs.size(), true),
      /*x_lo = */ VectorXd::Zero(rhs.size()),
      /*x_hi = */ VectorXd::Zero(rhs.size()), block_dim);
}

VectorXd sparse::MatrixSolveUpperTriangle(const MatrixXd& D,
                                          const VectorXd& rhs) {
  return MatrixSolveUpperTriangle(D, rhs,
                                  /*C = */ ArrayXb::Constant(rhs.size(), true),
                                  /*x_lo = */ VectorXd::Zero(rhs.size()),
                                  /*x_hi = */ VectorXd::Zero(rhs.size()));
}

VectorXd sparse::MatrixSolveBlockUpperTriangle(const MatrixXd& D,
                                               const VectorXd& rhs,
                                               int block_dim) {
  return MatrixSolveBlockUpperTriangle(
      D, rhs, /*C = */ ArrayXb::Constant(rhs.size(), true),
      /*x_lo = */ VectorXd::Zero(rhs.size()),
      /*x_hi = */ VectorXd::Zero(rhs.size()), block_dim);
}

/////////////////////////////////////////////////////////////////////////////////

VectorXd sparse::CalculateBlockLx(const ConstraintsList& constraints,
                                  const MatrixXd& M_inverse, const VectorXd& x,
                                  double epsilon_diagonal,
                                  double scale_diagonal) {
  VectorXd Lx = VectorXd::Zero(x.size());
  int Lx_row_index = 0;

  for (int i = 0; i < constraints.size(); ++i) {
    MatrixXd j_i0, j_i1;  // jacobians for the 2 component of this constraint
    ArrayXb ct;           // unused
    VectorXd c_lo, c_hi;  // unused
    constraints.at(i)->ComputeJ(&j_i0, &j_i1, &ct, &c_lo, &c_hi);
    int i0 = constraints.at(i)->i0_;  // component 0's global index
    int i1 = constraints.at(i)->i1_;  // component 1's global index

    int x_row_index = 0;

    // Lower triangle blocks
    for (int j = 0; j < i; ++j) {
      MatrixXd j_j0,
          j_j1;  // jacobians for the 2 components in the constraint
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

      // Accumulate the i-th subblock of Lx
      Lx.block(Lx_row_index, 0, j_i0.rows(), 1) +=
          JMJt * x.block(x_row_index, 0, JMJt.cols(), 1);

      // Update x_row_index for subvector multiplication with this JMJt
      // submatrix.
      x_row_index += j_j0.rows();
    }

    // Lower triangle of the diagonal blocks
    MatrixXd JMJtDiag = MatrixXd::Zero(j_i0.rows(), j_i0.rows());
    if (i0 >= 0) {
      JMJtDiag +=
          j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_i0.transpose();
    }
    if (i1 >= 0) {
      JMJtDiag +=
          j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_i1.transpose();
    }
    Lx.block(Lx_row_index, 0, j_i0.rows(), 1) +=
        JMJtDiag.triangularView<Eigen::StrictlyLower>() *
        x.block(x_row_index, 0, JMJtDiag.cols(), 1);

    // Update Lx_row_index for the next iteration.
    Lx_row_index += j_i0.rows();
  }

  return Lx;
}

VectorXd sparse::CalculateBlockUx(const ConstraintsList& constraints,
                                  const MatrixXd& M_inverse, const VectorXd& x,
                                  double epsilon_diagonal,
                                  double scale_diagonal) {
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

    // Upper triangle of the diagonal blocks
    MatrixXd JMJtDiag = MatrixXd::Zero(j_i0.rows(), j_i0.rows());
    if (i0 >= 0) {
      JMJtDiag +=
          j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_i0.transpose();
    }
    if (i1 >= 0) {
      JMJtDiag +=
          j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_i1.transpose();
    }
    Ux.block(Ux_row_index, 0, j_i0.rows(), 1) +=
        JMJtDiag.triangularView<Eigen::StrictlyUpper>() *
        x.block(x_row_index, 0, JMJtDiag.cols(), 1);

    // Upper triangle blocks
    for (int j = i + 1; j < constraints.size(); ++j) {
      MatrixXd j_j0,
          j_j1;  // jacobians for the 2 components in the constraint
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

      // Update x_row_index for subvector multiplication with this JMJt
      // submatrix.
      x_row_index += j_i0.rows();

      // Accumulate the i-th subblock of Ux
      Ux.block(Ux_row_index, 0, j_i0.rows(), 1) +=
          JMJt * x.block(x_row_index, 0, JMJt.cols(), 1);
    }

    // Update Ux_row_index for the next constraint
    Ux_row_index += j_i0.rows();
  }

  return Ux;
}

VectorXd sparse::CalculateBlockLxUx(const ConstraintsList& constraints,
                                    const MatrixXd& M_inverse,
                                    const VectorXd& x, double epsilon_diagonal,
                                    double scale_diagonal) {
  return CalculateBlockLx(constraints, M_inverse, x) +
         CalculateBlockUx(constraints, M_inverse, x);
}

VectorXd sparse::CalculateBlockDx(const ConstraintsList& constraints,
                                  const MatrixXd& M_inverse, const VectorXd& x,
                                  double epsilon, double scale) {
  VectorXd Dx = VectorXd::Zero(x.size());
  int Dx_row_index = 0;

  for (int i = 0; i < constraints.size(); ++i) {
    MatrixXd j0, j1;
    ArrayXb ct;           // unused
    VectorXd c_lo, c_hi;  // unused
    constraints.at(i)->ComputeJ(&j0, &j1, &ct, &c_lo, &c_hi);
    int i0 = constraints.at(i)->i0_;
    int i1 = constraints.at(i)->i1_;

    // Accumulate JMJt if i0 or i1 is not -1. Negative index indicates joint
    // constraint with an anchor or contact constraint with ground.
    MatrixXd JMJt = MatrixXd::Zero(j0.rows(), j0.rows());
    if (i0 >= 0) {
      JMJt += j0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j0.transpose();
    }
    if (i1 >= 0) {
      JMJt += j1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j1.transpose();
    }
    JMJt.diagonal() = (JMJt.diagonal().array() + epsilon) * scale;

    Dx.block(Dx_row_index, 0, j0.rows(), 1) =
        JMJt.diagonal().asDiagonal() * x.block(Dx_row_index, 0, j0.rows(), 1);

    // Update Dx_row_index for the next constraint
    Dx_row_index += j0.rows();
  }

  return Dx;
}

VectorXd sparse::CalculateBlockUxDx(const ConstraintsList& constraints,
                                    const MatrixXd& M_inverse,
                                    const VectorXd& x, double epsilon_diagonal,
                                    double scale_diagonal) {
  return CalculateBlockUx(constraints, M_inverse, x) +
         CalculateBlockDx(constraints, M_inverse, x, epsilon_diagonal,
                          scale_diagonal);
}

VectorXd sparse::CalculateBlockLxDx(const ConstraintsList& constraints,
                                    const MatrixXd& M_inverse,
                                    const VectorXd& x, double epsilon_diagonal,
                                    double scale_diagonal) {
  return CalculateBlockLx(constraints, M_inverse, x) +
         CalculateBlockDx(constraints, M_inverse, x, epsilon_diagonal,
                          scale_diagonal);
}

VectorXd sparse::CalculateBlockJMJtX(const ConstraintsList& constraints,
                                     const MatrixXd& M_inverse,
                                     const VectorXd& x,
                                     double epsilon_diagonal) {
  VectorXd JMJtX = VectorXd::Zero(x.size());
  int JMJtX_row_index = 0;

  for (int i = 0; i < constraints.size(); ++i) {
    MatrixXd j_i0, j_i1;
    ArrayXb ct;           // unused
    VectorXd c_lo, c_hi;  // unused
    constraints.at(i)->ComputeJ(&j_i0, &j_i1, &ct, &c_lo, &c_hi);
    int i0 = constraints.at(i)->i0_;
    int i1 = constraints.at(i)->i1_;

    int x_row_index = 0;

    // Accumulate JMJt subblock if i0 or i1 is not -1. Negative index
    // indicates joint constraint with an anchor or contact constraint with
    // ground.
    for (int j = 0; j < constraints.size(); ++j) {
      MatrixXd j_j0,
          j_j1;  // jacobians for the 2 components in the constraint
      constraints.at(j)->ComputeJ(&j_j0, &j_j1, &ct, &c_lo, &c_hi);
      int j0 = constraints.at(j)->i0_;  // component 0's global index
      int j1 = constraints.at(j)->i1_;  // component 1's global index

      if (i == j) {
        // Diagonal blocks
        MatrixXd subblock = MatrixXd::Zero(j_i0.rows(), j_i0.rows());
        if (i0 >= 0) {
          subblock +=
              j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_i0.transpose();
        }
        if (i1 >= 0) {
          subblock +=
              j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_i1.transpose();
        }
        JMJtX.block(JMJtX_row_index, 0, j_i0.rows(), 1) +=
            (subblock +
             epsilon_diagonal *
                 MatrixXd::Identity(subblock.rows(), subblock.cols())) *
            x.block(x_row_index, 0, subblock.cols(), 1);
        x_row_index += j_i0.rows();
      } else {
        // Off-diagonal blocks
        MatrixXd subblock = MatrixXd::Zero(j_i0.rows(), j_j0.rows());
        if (i0 == j0 && i0 >= 0) {
          subblock +=
              j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_j0.transpose();
        } else if (i0 == j1 && i0 >= 0) {
          subblock +=
              j_i0 * M_inverse.block<6, 6>(i0 * 6, i0 * 6) * j_j1.transpose();
        }
        if (i1 == j0 && i1 >= 0) {
          subblock +=
              j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_j0.transpose();
        } else if (i1 == j1 && i1 >= 0) {
          subblock +=
              j_i1 * M_inverse.block<6, 6>(i1 * 6, i1 * 6) * j_j1.transpose();
        }
        JMJtX.block(JMJtX_row_index, 0, j_i0.rows(), 1) +=
            subblock * x.block(x_row_index, 0, subblock.cols(), 1);
        x_row_index += j_j0.rows();
      }
    }

    JMJtX_row_index += j_i0.rows();
  }

  return JMJtX;
}

void sparse::ConstructMixedConstraints(const ConstraintsList& constraints,
                                       ArrayXb* C, VectorXd* x_lo,
                                       VectorXd* x_hi) {
  for (int i = 0; i < constraints.size(); ++i) {
    MatrixXd j0, j1;
    ArrayXb ct;
    VectorXd c_lo;
    VectorXd c_hi;
    constraints.at(i)->ComputeJ(&j0, &j1, &ct, &c_lo, &c_hi);

    // Append new rows to C, x_lo, x_hi
    CHECK(C->cols() == ct.cols());
    C->conservativeResize(C->rows() + ct.rows(), ct.cols());
    C->block(C->rows() - ct.rows(), 0, ct.rows(), ct.cols()) = ct;

    CHECK(x_lo->cols() == c_lo.cols());
    x_lo->conservativeResize(x_lo->rows() + c_lo.rows(), x_lo->cols());
    x_lo->block(x_lo->rows() - c_lo.rows(), 0, c_lo.rows(), c_lo.cols()) = c_lo;

    CHECK(x_hi->cols() == c_hi.cols());
    x_hi->conservativeResize(x_hi->rows() + c_hi.rows(), x_hi->cols());
    x_hi->block(x_hi->rows() - c_hi.rows(), 0, c_hi.rows(), c_hi.cols()) = c_hi;
  }
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

// Add a small value to all diagonal term. epsilon_diagonal is used for
// constraint force mixing.
constexpr double kEpsilonDiagonal = 0.01;

// Compute the block diagonal of the JMJt systems matrix.
MatrixXd GetBlockDiagonal(const Ensemble& en, int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();
  const MatrixXd JMJt =
      J * M * J.transpose() +
      kEpsilonDiagonal * MatrixXd::Identity(J.rows(), J.rows());
  MatrixXd JMJt_block_diagonal = MatrixXd::Zero(JMJt.rows(), JMJt.cols());
  for (int i = 0; i < JMJt.rows() / dof_per_constraint; ++i) {
    JMJt_block_diagonal.block(i * dof_per_constraint, i * dof_per_constraint,
                              dof_per_constraint, dof_per_constraint) =
        JMJt.block(i * dof_per_constraint, i * dof_per_constraint,
                   dof_per_constraint, dof_per_constraint);
  }
  return JMJt_block_diagonal;
}

// Compare (JMJt_block_diagonal * x) when calculated using CalculateBlockDx()
// versus forming the JMJt systems matrix.
bool CompareBlockDx(const Ensemble& en, int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();

  // Use a randomly generated x and scale
  const VectorXd x = VectorXd::Random(J.rows());
  double scale = VectorXd::Random(1)(0);

  // Calculate block Dx using CalculateBlockDx()
  const VectorXd Dx =
      sparse::CalculateBlockDx(en.constraints(), M, x, kEpsilonDiagonal, scale);

  // Calculate block Dx by forming JMJt the systems matrix.
  const MatrixXd JMJt_block_diagonal =
      GetBlockDiagonal(en, dof_per_constraint) * scale;
  std::cout << "JMJt_block_diagonal =\n" << JMJt_block_diagonal << std::endl;
  const VectorXd JMJt_Dx = JMJt_block_diagonal * x;

  return (Dx - JMJt_Dx).norm() < kAllowNumericalError;
}

// Compute the block upper triangle of the JMJt systems matrix.The
// block diagonal is included.
MatrixXd GetBlockUpperTriangle(const Ensemble& en, int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();
  const MatrixXd JMJt =
      J * M * J.transpose() +
      kEpsilonDiagonal * MatrixXd::Identity(J.rows(), J.rows());
  MatrixXd JMJt_block_upper_triangle = JMJt.triangularView<Eigen::Upper>();
  for (int i = 0; i < JMJt.rows() / dof_per_constraint; ++i) {
    JMJt_block_upper_triangle.block(i * dof_per_constraint,
                                    i * dof_per_constraint, dof_per_constraint,
                                    dof_per_constraint) =
        JMJt.block(i * dof_per_constraint, i * dof_per_constraint,
                   dof_per_constraint, dof_per_constraint);
  }
  return JMJt_block_upper_triangle;
}

// Compute the block strictly upper triangle of the JMJt systems matrix. The
// block diagonal is excluded.
MatrixXd GetBlockStrictlyUpperTriangle(const Ensemble& en,
                                       int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();
  const MatrixXd JMJt =
      J * M * J.transpose() +
      kEpsilonDiagonal * MatrixXd::Identity(J.rows(), J.rows());
  MatrixXd JMJt_block_upper_triangle = JMJt.triangularView<Eigen::Upper>();
  for (int i = 0; i < JMJt.rows() / dof_per_constraint; ++i) {
    JMJt_block_upper_triangle.block(i * dof_per_constraint,
                                    i * dof_per_constraint, dof_per_constraint,
                                    dof_per_constraint) =
        MatrixXd::Zero(dof_per_constraint, dof_per_constraint);
  }
  return JMJt_block_upper_triangle;
}

// Compare (JMJt_block_upper_triangle * x) when calculated using
// CalculateBlockUx() versus forming the JMJt systems matrix.
bool CompareBlockUx(const Ensemble& en, int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();

  // Use a randomly generated x
  const VectorXd x = VectorXd::Random(J.rows());

  // Calculate block Ux using CalculateBlockUx()
  const VectorXd Ux =
      sparse::CalculateBlockUx(en.constraints(), M, x, kEpsilonDiagonal);

  // Calculate block Ux by forming JMJt the systems matrix.
  MatrixXd JMJt_block_upper_triangle =
      GetBlockStrictlyUpperTriangle(en, dof_per_constraint);
  const VectorXd JMJt_Ux = JMJt_block_upper_triangle * x;

  return (Ux - JMJt_Ux).norm() < kAllowNumericalError;
}

// Compute the block lower triangle of the JMJt systems matrix.The
// block diagonal is included.
MatrixXd GetBlockLowerTriangle(const Ensemble& en, int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();
  const MatrixXd JMJt =
      J * M * J.transpose() +
      kEpsilonDiagonal * MatrixXd::Identity(J.rows(), J.rows());
  MatrixXd JMJt_block_lower_triangle = JMJt.triangularView<Eigen::Lower>();
  for (int i = 0; i < JMJt.rows() / dof_per_constraint; ++i) {
    JMJt_block_lower_triangle.block(i * dof_per_constraint,
                                    i * dof_per_constraint, dof_per_constraint,
                                    dof_per_constraint) =
        JMJt.block(i * dof_per_constraint, i * dof_per_constraint,
                   dof_per_constraint, dof_per_constraint);
  }
  return JMJt_block_lower_triangle;
}

// Compute the block strictly lower triangle of the JMJt systems matrix. The
// block diagonal is excluded.
MatrixXd GetBlockStrictlyLowerTriangle(const Ensemble& en,
                                       int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();
  const MatrixXd JMJt =
      J * M * J.transpose() +
      kEpsilonDiagonal * MatrixXd::Identity(J.rows(), J.rows());
  MatrixXd JMJt_block_lower_triangle = JMJt.triangularView<Eigen::Lower>();
  for (int i = 0; i < JMJt.rows() / dof_per_constraint; ++i) {
    JMJt_block_lower_triangle.block(i * dof_per_constraint,
                                    i * dof_per_constraint, dof_per_constraint,
                                    dof_per_constraint) =
        MatrixXd::Zero(dof_per_constraint, dof_per_constraint);
  }
  return JMJt_block_lower_triangle;
}

// Compare (JMJt_block_lower_triangle * x) when calculated using
// CalculateBlockLx() versus forming the JMJt systems matrix.
bool CompareBlockLx(const Ensemble& en, int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();

  // Use a randomly generated x
  const VectorXd x = VectorXd::Random(J.rows());

  // Calculate block Lx using CalculateBlockLx()
  const VectorXd Lx =
      sparse::CalculateBlockLx(en.constraints(), M, x, kEpsilonDiagonal);

  // Calculate block Lx by forming JMJt the systems matrix.
  MatrixXd JMJt_block_lower_triangle =
      GetBlockStrictlyLowerTriangle(en, dof_per_constraint);
  const VectorXd JMJt_Lx = JMJt_block_lower_triangle * x;

  return (Lx - JMJt_Lx).norm() < kAllowNumericalError;
}

bool CompareBlockLxUx(const Ensemble& en, int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();

  // Use a randomly generated x
  const VectorXd x = VectorXd::Random(J.rows());

  // Calculate block LxUx using CalculateBlockLxUx()
  const VectorXd LxUx =
      sparse::CalculateBlockLxUx(en.constraints(), M, x, kEpsilonDiagonal);

  // Calculate block Lx by forming JMJt the systems matrix.
  MatrixXd JMJt_block_lower_upper_triangle =
      GetBlockStrictlyLowerTriangle(en, dof_per_constraint) +
      GetBlockStrictlyUpperTriangle(en, dof_per_constraint);
  const VectorXd JMJt_LxUx = JMJt_block_lower_upper_triangle * x;

  return (LxUx - JMJt_LxUx).norm() < kAllowNumericalError;
}

bool CompareBlockJMJtX(const Ensemble& en, int dof_per_constraint) {
  const MatrixXd J = en.ComputeJ();
  const MatrixXd M = en.M_inverse();

  // Use a randomly generated x
  const VectorXd x = VectorXd::Random(J.rows());

  // Calculate (JMJt * x) using CalculateBlockJMJtX().
  const VectorXd JMJtX =
      sparse::CalculateBlockJMJtX(en.constraints(), M, x, kEpsilonDiagonal);

  // Calculate (JMJt * x) by forming JMJt the systems matrix.
  MatrixXd JMJt = J * M * J.transpose() +
                  kEpsilonDiagonal * MatrixXd::Identity(J.rows(), J.rows());

  return (JMJtX - JMJt * x).norm() < kAllowNumericalError;
}

// Test CalculateBlockDx on a hanging chain ensemble.
TEST_FUNCTION(CalculateBlockDx_chain) {
  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(CompareBlockDx(chain, 1));
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockDx(chain, 1));
  }
}

// Test CalculateBlockDx on a falling cairn ensemble.
TEST_FUNCTION(CalculateBlockDx_cairn) {
  Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
  cairn.Init();
  cairn.InitStabilize();
  CHECK(CompareBlockDx(cairn, 1));
  for (int i = 0; i < kNumSimSteps; ++i) {
    cairn.Step(kSimTimeStep * 5, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockDx(cairn, 1));
  }
}

// Test CalculateBlockUx on a hanging chain ensemble.
TEST_FUNCTION(CalculateBlockUx_chain) {
  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(CompareBlockUx(chain, 1));
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockUx(chain, 1));
  }
}

// Test CalculateBlockUx on a falling cairn ensemble.
TEST_FUNCTION(CalculateBlockUx_cairn) {
  for (int j = 0; j < 10; ++j) {
    Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
    cairn.Init();
    cairn.InitStabilize();
    CHECK(CompareBlockUx(cairn, 1));
    for (int i = 0; i < kNumSimSteps; ++i) {
      cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
      CHECK(CompareBlockUx(cairn, 1));
    }
  }
}

// Test CalculateBlockLx on a hanging chain ensemble.
TEST_FUNCTION(CalculateBlockLx_chain) {
  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(CompareBlockLx(chain, 1));
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockLx(chain, 1));
  }
}

// Test CalculateBlockLx on a falling cairn ensemble.
TEST_FUNCTION(CalculateBlockLx_cairn) {
  Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
  cairn.Init();
  cairn.InitStabilize();
  CHECK(CompareBlockLx(cairn, 1));
  for (int i = 0; i < kNumSimSteps; ++i) {
    cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockLx(cairn, 1));
  }
}

// Test CalculateBlockLxUx on a hanging chain ensemble.
TEST_FUNCTION(CalculateBlockLxUx_chain) {
  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(CompareBlockLxUx(chain, 1));
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockLxUx(chain, 1));
  }
}

// Test CalculateBlockLxUx on a falling cairn ensemble.
TEST_FUNCTION(CalculateBlockLxUx_cairn) {
  Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
  cairn.Init();
  cairn.InitStabilize();
  CHECK(CompareBlockLxUx(cairn, 1));
  for (int i = 0; i < kNumSimSteps; ++i) {
    cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockLxUx(cairn, 1));
  }
}

// Test CalculateBlockJMJtX on a hanging chain ensemble.
TEST_FUNCTION(CalculateBlockJMJtX_chain) {
  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(CompareBlockJMJtX(chain, 1));
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockJMJtX(chain, 1));
  }
}

// Test CalculateBlockLxUx on a falling cairn ensemble.
TEST_FUNCTION(CalculateBlockJMJtX_cairn) {
  Cairn cairn(4, {-0.2, 0.2}, {-0.2, 0.2}, {1, 8});
  cairn.Init();
  cairn.InitStabilize();
  CHECK(CompareBlockJMJtX(cairn, 1));
  for (int i = 0; i < kNumSimSteps; ++i) {
    cairn.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(CompareBlockJMJtX(cairn, 1));
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
    auto x = sparse::MatrixSolveDiagonal(A, b);
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(MatrixSolveBlockDiagonal_matrix) {
  const int block_dim = 1;
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate a block diagonal matrix with dimension between 6 and
    // 90, where each block is (block_dim * block_dim).
    int dim = (rand() % 28 + 2) * block_dim;
    MatrixXd A = MatrixXd::Zero(dim, dim);
    for (int j = 0; j < dim / block_dim; ++j) {
      A.block(j * block_dim, j * block_dim, block_dim, block_dim) =
          MatrixXd::Random(block_dim, block_dim);
    }
    // Randomly generate the rhs vector
    const VectorXd b = VectorXd::Random(dim);
    // Solve
    auto x = sparse::MatrixSolveBlockDiagonal(A, b, block_dim);
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(MatrixSolveBlockDiagonal_ensemble) {
  auto check_solve_block_diagonal = [](const Ensemble& en) {
    const MatrixXd A = GetBlockDiagonal(en, 1);
    const VectorXd b = VectorXd::Random(A.rows());
    const VectorXd x = sparse::MatrixSolveBlockDiagonal(
        en.constraints(), en.M_inverse(), b, kEpsilonDiagonal);
    return (A * x - b).norm() < kAllowNumericalError;
  };

  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(check_solve_block_diagonal(chain));
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(check_solve_block_diagonal(chain));
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
    auto x = sparse::MatrixSolveLowerTriangle(A, b);
    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A = " << GetConditionNumber(A)
                << std::endl;
      std::cout << "(Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(MatrixSolveBlockLowerTriangle_matrix) {
  const int block_dim = 1;
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate a block lower triangle matrix with dimension between
    // 6 and 90, where each block is (block_dim * block_dim).
    int dim = (rand() % 28 + 2) * block_dim;
    MatrixXd A = MatrixXd::Random(dim, dim);
    A = A.triangularView<Eigen::Lower>();
    for (int j = 0; j < dim / block_dim; ++j) {
      A.block(j * block_dim, j * block_dim, block_dim, block_dim) =
          MatrixXd::Random(block_dim, block_dim);
    }
    if (GetConditionNumber(A) > kGoodConditionNumber) {
      --i;
      continue;
    }
    // Randomly generate the rhs vector
    const VectorXd b = VectorXd::Random(dim);
    // Solve
    auto x = sparse::MatrixSolveBlockLowerTriangle(A, b, block_dim);

    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A [" << A.rows() << "x" << A.cols()
                << "] = " << GetConditionNumber(A) << std::endl;
      std::cout << "(Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }

    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(MatrixSolveBlockLowerTriangle_ensemble) {
  auto check_solve_block_lower_triangle = [](const Ensemble& en) {
    const MatrixXd A = GetBlockLowerTriangle(en, 1);
    const VectorXd b = VectorXd::Random(A.rows());
    const VectorXd x = sparse::MatrixSolveBlockLowerTriangle(
        en.constraints(), en.M_inverse(), b, kEpsilonDiagonal);
    // std::cout << "A = \n" << A << std::endl;
    // std::cout << "b = \n" << b << std::endl;
    // std::cout << "x = \n" << x << std::endl;
    // std::cout << "A * x - b = \n" << A * x - b << std::endl;
    return (A * x - b).norm() < kAllowNumericalError;
  };

  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(check_solve_block_lower_triangle(chain));
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(check_solve_block_lower_triangle(chain));
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
    auto x = sparse::MatrixSolveUpperTriangle(A, b);
    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A = " << GetConditionNumber(A)
                << std::endl;
      std::cout << "(Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }
    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(MatrixSolveBlockUpperTriangle_matrix) {
  const int block_dim = 1;
  for (int i = 0; i < kNumTestInsts; ++i) {
    // Randomly generate a block upper triangle matrix with dimension between
    // 6 and 90, where each block is (block_dim * block_dim).
    int dim = (rand() % 28 + 2) * block_dim;
    MatrixXd A = MatrixXd::Random(dim, dim);
    A = A.triangularView<Eigen::Upper>();
    for (int j = 0; j < dim / block_dim; ++j) {
      A.block(j * block_dim, j * block_dim, block_dim, block_dim) =
          MatrixXd::Random(block_dim, block_dim);
    }
    if (GetConditionNumber(A) > kGoodConditionNumber) {
      --i;
      continue;
    }
    // Randomly generate the rhs vector
    const VectorXd b = VectorXd::Random(dim);
    // Solve
    auto x = sparse::MatrixSolveBlockUpperTriangle(A, b, block_dim);

    if ((A * x - b).norm() > kAllowNumericalError) {
      std::cout << "Condition number of A [" << A.rows() << "x" << A.cols()
                << "] = " << GetConditionNumber(A) << std::endl;
      std::cout << "(Ax-b).norm() = " << (A * x - b).norm() << std::endl;
    }

    CHECK((A * x - b).norm() < kAllowNumericalError);
  }
}

TEST_FUNCTION(MatrixSolveBlockUpperTriangle_ensemble) {
  auto check_solve_block_upper_triangle = [](const Ensemble& en) {
    const MatrixXd A = GetBlockUpperTriangle(en, 1);
    const VectorXd b = VectorXd::Random(A.rows());
    const VectorXd x = sparse::MatrixSolveBlockUpperTriangle(
        en.constraints(), en.M_inverse(), b, kEpsilonDiagonal);
    return (A * x - b).norm() < kAllowNumericalError;
  };

  Chain chain(4, Vector3d(0, 0, 2));
  chain.Init();
  CHECK(check_solve_block_upper_triangle(chain));
  for (int i = 0; i < kNumSimSteps; ++i) {
    chain.Step(kSimTimeStep, Ensemble::Integrator::OPEN_DYNAMICS_ENGINE);
    CHECK(check_solve_block_upper_triangle(chain));
  }
}

}  // namespace
