// Library of iterative methods for sparse matrices

#ifndef __SPARSE_ITERATIONS_H__
#define __SPARSE_ITERATIONS_H__

#include "Eigen/Dense"
#include "utils.h"

namespace sparse {

// Iterative methods to solve Ax=b
VectorXd JacobiIteration(const MatrixXd& A, const VectorXd& b);
VectorXd GaussSeidelIteration(const MatrixXd& A, const VectorXd& b);
VectorXd SORIteration(const MatrixXd& A, const VectorXd& b);

}  // namespace sparse

#endif
