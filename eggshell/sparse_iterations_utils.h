// Library of utils functions for sparse iteration methods.

#ifndef __SPARSE_ITERATIONS_UTILS_H__
#define __SPARSE_ITERATIONS_UTILS_H__

#include "Eigen/Dense"
#include "ensembles.h"
#include "utils.h"

namespace sparse {

// Solve Ax=b for special matrices, and their sparse analog for ensembles
// without constructing the systems matrix.

// Solve Ax=b for (sparse) diagonal A.
VectorXd MatrixSolveDiagonal(const MatrixXd& D, const VectorXd& rhs,
                             const ArrayXb& C, const VectorXd& x_lo,
                             const VectorXd& x_hi);
VectorXd MatrixSolveDiagonal(const MatrixXd& D, const VectorXd& rhs);
VectorXd MatrixSolveBlockDiagonal(const MatrixXd& D, const VectorXd& rhs,
                                  const ArrayXb& C, const VectorXd& x_lo,
                                  const VectorXd& x_hi, int block_dim);
VectorXd MatrixSolveBlockDiagonal(const MatrixXd& D, const VectorXd& rhs,
                                  int block_dim);
VectorXd MatrixSolveSparseDiagonal(const ConstraintsList& constraints,
                                   const MatrixXd& M_inverse,
                                   const VectorXd& rhs, double epsilon = 0.0,
                                   double scale = 1.0);

// Solve Ax=b for (block) lower triangle A.
VectorXd MatrixSolveLowerTriangle(const MatrixXd& L, const VectorXd& rhs);
VectorXd MatrixSolveLowerTriangle(const MatrixXd& L, const VectorXd& rhs,
                                  const ArrayXb& C, const VectorXd& x_lo,
                                  const VectorXd& x_hi);
VectorXd MatrixSolveBlockLowerTriangle(const MatrixXd& L, const VectorXd& rhs,
                                       int block_dim);
VectorXd MatrixSolveBlockLowerTriangle(const MatrixXd& L, const VectorXd& rhs,
                                       const ArrayXb& C, const VectorXd& x_lo,
                                       const VectorXd& x_hi, int block_dim);
VectorXd MatrixSolveSparseLowerTriangle(const ConstraintsList& constraints,
                                        const MatrixXd& M_inverse,
                                        const VectorXd& rhs,
                                        double epsilon_diagonal = 0.0,
                                        double scale_diagonal = 1.0);

// Solve Ax=b for (block) upper triangle A.
VectorXd MatrixSolveUpperTriangle(const MatrixXd& U, const VectorXd& rhs);
VectorXd MatrixSolveUpperTriangle(const MatrixXd& U, const VectorXd& rhs,
                                  const ArrayXb& C, const VectorXd& x_lo,
                                  const VectorXd& x_hi);
VectorXd MatrixSolveBlockUpperTriangle(const MatrixXd& U, const VectorXd& rhs,
                                       int block_dim);
VectorXd MatrixSolveBlockUpperTriangle(const MatrixXd& U, const VectorXd& rhs,
                                       const ArrayXb& C, const VectorXd& x_lo,
                                       const VectorXd& x_hi, int block_dim);
VectorXd MatrixSolveSparseUpperTriangle(const ConstraintsList& constraints,
                                        const MatrixXd& M_inverse,
                                        const VectorXd& rhs,
                                        double epsilon_diagonal = 0.0,
                                        double scale_diagonal = 1.0);

// Calculate Dx, the sparse diagonal times x. Optionally scale all elements of
// Dx, used in successive overrelaxation (SOR). Optionally add epsilon to the
// diagonal elements, used in constraint force mixing (CFM).
VectorXd CalculateSparseDx(const ConstraintsList& constraints,
                           const MatrixXd& M_inverse, const VectorXd& x,
                           double epsilon = 0.0, double scale = 1.0);

// Calculate Lx, the sparse strictly lower triangle times x. scale_diagonal and
// epsilon_diagonal are unused, but present to allow uniform argument list with
// other CalculateSparse* functions.
VectorXd CalculateSparseLx(const ConstraintsList& constraints,
                           const MatrixXd& M_inverse, const VectorXd& x,
                           double epsilon_diagonal = 0.0,
                           double scale_diagonal = 1.0);

// Calculate Ux, the sparse strictly upper triangle times x. scale_diagonal and
// epsilon_diagonal are unused, but present to allow uniform argument list with
// other CalculateSparse* functions.
VectorXd CalculateSparseUx(const ConstraintsList& constraints,
                           const MatrixXd& M_inverse, const VectorXd& x,
                           double epsilon_diagonal = 0.0,
                           double scale_diagonal = 1.0);

// Calculate Lx + Ux, the sparse strictly lower and upper triangles times x.
// scale_diagonal and epsilon_diagonal are unused, but present to allow uniform
// argument list with other CalculateSparse* functions.
VectorXd CalculateSparseLxUx(const ConstraintsList& constraints,
                             const MatrixXd& M_inverse, const VectorXd& x,
                             double epsilon_diagonal = 0.0,
                             double scale_diagonal = 1.0);

// Calculate Lx + Dx, the sparse strictly lower triangle plus diagonal times x
VectorXd CalculateSparseLxDx(const ConstraintsList& constraints,
                             const MatrixXd& M_inverse, const VectorXd& x,
                             double epsilon_diagonal = 0.0,
                             double scale_diagonal = 1.0);

// Calculate Ux + Dx, the sparse strictly lower triangle plus diagonal times x
VectorXd CalculateSparseUxDx(const ConstraintsList& constraints,
                             const MatrixXd& M_inverse, const VectorXd& x,
                             double epsilon_diagonal = 0.0,
                             double scale_diagonal = 1.0);

// Calculate JMJtX for an ensemble without explicitly forming the systems matrix
// JMJt.
VectorXd CalculateSparseJMJtX(const ConstraintsList& constraints,
                              const MatrixXd& M_inverse, const VectorXd& x,
                              double epsilon_diagonal = 0.0);

// Construct C = constraint type, true for equality, false for
// inequality. For inequality constraints, {x_lo, x_hi} indicates lower and
// higher x bound in Ax = b+w.
void ConstructMixedConstraints(const ConstraintsList& constraints, ArrayXb* C,
                               VectorXd* x_lo, VectorXd* x_hi);

}  // namespace sparse

#endif
