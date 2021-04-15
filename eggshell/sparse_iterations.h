// Library of iterative methods for sparse matrices

#ifndef __SPARSE_ITERATIONS_H__
#define __SPARSE_ITERATIONS_H__

#include "Eigen/Dense"
#include "ensembles.h"
#include "utils.h"

namespace sparse {

// Iterative methods to solve Ax=b
VectorXd JacobiIteration(const MatrixXd& A, const VectorXd& b);
VectorXd GaussSeidelIteration(const MatrixXd& A, const VectorXd& b);
VectorXd SORIteration(const MatrixXd& A, const VectorXd& b);

// Iterative methods given components, joint constraints, and contact
// constraints, without explicitly forming the systems matrix.
VectorXd JacobiIteration(const ConstraintsList& constraints,
                         const MatrixXd& M_inverse, const VectorXd& rhs);
VectorXd GaussSeidelIteration(const ConstraintsList& constraints,
                              const MatrixXd& M_inverse, const VectorXd& rhs);
VectorXd SORIteration(const ConstraintsList& constraints,
                      const MatrixXd& M_inverse, const VectorXd& rhs);

}  // namespace sparse

#endif
