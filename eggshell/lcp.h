// Library of lcp solvers

#ifndef __LCP_H__
#define __LCP_H__

#include "Eigen/Dense"
#include "utils.h"

namespace Lcp {

bool MurtyPrincipalPivot(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                         VectorXd& w);
bool MurtyPrincipalPivot(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                         VectorXd& w, const double x_lo, const double x_hi);
bool MurtyPrincipalPivot(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                         VectorXd& w, const VectorXd& x_lo,
                         const VectorXd& x_hi);

// C = constraint_type, true for equality constraint, false for inequality
// constraint
bool MixedConstraintsSolver(const MatrixXd& A, const VectorXd& b,
                            const ArrayXb& C, const VectorXd& x_lo,
                            const VectorXd& x_hi, VectorXd& x, VectorXd& w);

}  // namespace Lcp

#endif
