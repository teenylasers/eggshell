// Library of utils functions for sparse iteration methods.

#ifndef __SPARSE_ITERATIONS_UTILS_H__
#define __SPARSE_ITERATIONS_UTILS_H__

#include "Eigen/Dense"
#include "ensembles.h"
#include "utils.h"

namespace sparse {

// Solve Ax=b for special matrices, and their block analog for ensembles without
// constructing the systems matrix.

// Solve Ax=b for (block) diagonal A.
VectorXd MatrixSolveDiagonal(const MatrixXd& D, const VectorXd& rhs);
VectorXd MatrixSolveBlockDiagonal(const MatrixXd& D, const VectorXd& rhs,
                                  int block_dim);
VectorXd MatrixSolveBlockDiagonal(const ConstraintsList& constraints,
                                  const MatrixXd& M_inverse,
                                  const VectorXd& rhs);

// Solve Ax=b for (block) lower triangle A.
VectorXd MatrixSolveLowerTriangle(const MatrixXd& L, const VectorXd& rhs);
VectorXd MatrixSolveBlockLowerTriangle(const MatrixXd& L, const VectorXd& rhs,
                                       int block_dim);
VectorXd MatrixSolveBlockLowerTriangle(const ConstraintsList& constraints,
                                       const MatrixXd& M_inverse,
                                       const VectorXd& rhs);

// Solve Ax=b for (block) upper triangle A.
VectorXd MatrixSolveUpperTriangle(const MatrixXd& U, const VectorXd& rhs);
VectorXd MatrixSolveBlockUpperTriangle(const MatrixXd& U, const VectorXd& rhs,
                                       int block_dim);
VectorXd MatrixSolveBlockUpperTriangle(const ConstraintsList& constraints,
                                       const MatrixXd& M_inverse,
                                       const VectorXd& rhs);

// Calculate Lx, the block strictly lower triangle times x
VectorXd CalculateBlockLx(const ConstraintsList& constraints,
                          const MatrixXd& M_inverse, const VectorXd& x);

// Calculate Ux, the block strictly upper triangle times x
VectorXd CalculateBlockUx(const ConstraintsList& constraints,
                          const MatrixXd& M_inverse, const VectorXd& x);

// Calculate Lx + Ux, the block strictly lower triangle times x
VectorXd CalculateBlockLxPlusUx(const ConstraintsList& constraints,
                                const MatrixXd& M_inverse, const VectorXd& x);
// Calculate Dx, the block diagonal times x
VectorXd CalculateBlockDx(const ConstraintsList& constraints,
                          const MatrixXd& M_inverse, const VectorXd& x);

// Calculate JMJtX for an ensemble without explicitly forming the systems matrix
// JMJt.
VectorXd CalculateBlockJMJtX(const ConstraintsList& constraints,
                             const MatrixXd& M_inverse, const VectorXd& x);

}  // namespace sparse

#endif
