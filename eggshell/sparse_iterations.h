// Library of iterative methods for sparse matrices

#ifndef __SPARSE_ITERATIONS_H__
#define __SPARSE_ITERATIONS_H__

#include "Eigen/Dense"
#include "ensembles.h"
#include "utils.h"

namespace sparse {

// Iterative methods to solve Ax=b
VectorXd JacobiIteration(const MatrixXd& A, const VectorXd& b);
VectorXd JacobiIteration(const MatrixXd& A, const VectorXd& b, const ArrayXb& C,
                         const VectorXd& x_lo, const VectorXd& x_hi);
VectorXd GaussSeidelIteration(const MatrixXd& A, const VectorXd& b);
VectorXd GaussSeidelIteration(const MatrixXd& A, const VectorXd& b,
                              const ArrayXb& C, const VectorXd& x_lo,
                              const VectorXd& x_hi);
VectorXd SORIteration(const MatrixXd& A, const VectorXd& b);
VectorXd SORIteration(const MatrixXd& A, const VectorXd& b, const ArrayXb& C,
                      const VectorXd& x_lo, const VectorXd& x_hi);

// Iterative methods given components, joint constraints, and contact
// constraints, without explicitly forming the systems matrix.
VectorXd JacobiIteration(const ConstraintsList& constraints,
                         const MatrixXd& M_inverse, const VectorXd& rhs,
                         double cfm = 0.0);
VectorXd GaussSeidelIteration(const ConstraintsList& constraints,
                              const MatrixXd& M_inverse, const VectorXd& rhs,
                              double cfm = 0.0);
VectorXd SORIteration(const ConstraintsList& constraints,
                      const MatrixXd& M_inverse, const VectorXd& rhs,
                      double cfm = 0.0);

}  // namespace sparse

#endif
