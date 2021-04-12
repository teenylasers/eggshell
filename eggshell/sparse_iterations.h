// Library of iterative methods for sparse matrices

#ifndef __SPARSE_ITERATIONS_H__
#define __SPARSE_ITERATIONS_H__

#include "Eigen/Dense"
#include "ensembles.h"
#include "utils.h"

namespace sparse {

// Pointer to the function that solves Ax=b for a special A, implementation
// depends on whether A is diagonal, lower, or upper triangular.
typedef VectorXd (*matrix_solver)(const MatrixXd&, const VectorXd&);

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

//*************  Helper functions to iterative methods. *************//

// Calculate the strictly lower triangle times x
VectorXd CalculateBlockLx(const ConstraintsList& constraints,
                          const MatrixXd& M_inverse, const VectorXd& x);

// Calculate the strictly upper triangle times x
VectorXd CalculateBlockUx(const ConstraintsList& constraints,
                          const MatrixXd& M_inverse, const VectorXd& x);

// Calculate the diagonal times x
VectorXd CalculateBlockDx(const ConstraintsList& constraints,
                          const MatrixXd& M_inverse, const VectorXd& x);
}  // namespace sparse

#endif
