// Library of lcp solvers

#ifndef __LCP_H__
#define __LCP_H__

#include "Eigen/Dense"
#include "util.h"

using namespace Eigen;

namespace Lcp {

bool MurtyPrincipalPivot(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                         VectorXd& w);
bool MurtyPrincipalPivot(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                         VectorXd& w, const double x_lo, const double x_hi);

// C = constraint_type, true for equality constraint, false for inequality
// constraint
bool MixedConstraintsSolver(const MatrixXd& A, const VectorXd& b,
                            const ArrayXb& C, const VectorXd& x_lo,
                            const VectorXd& x_hi, VectorXd& x, VectorXd& w);

// TODO: combine matrix traverse for select and update
// TODO: move this function to somewhere that extends Eigen
// TODO: make ind an array of boolean instead of int
// TODO: combine for matrices and vectors.
// TODO: return type should work for all element types: d, f, i
// Select submatrix from A using a vector ind of indices, return the results in
// Matrix ret. No modifications to input A or ind.
MatrixXd SelectSubmatrix(const MatrixXd& A, const ArrayXb& row_ind,
                         const ArrayXb& col_ind);
VectorXd SelectSubvector(const VectorXd& v, const ArrayXb& ind);
// Update submatrix of A indicated by ind with content of m. Dimensions of
// ind(true) and m shouild match.
void UpdateSubmatrix(MatrixXd& A, const ArrayXb& row_ind,
                     const ArrayXb& col_ind, const MatrixXd& m);
void UpdateSubvector(VectorXd& v, const ArrayXb& ind, const VectorXd& n);
void UpdateSubvector(VectorXd& v, const ArrayXb& ind, double d);

}  // namespace Lcp

#endif
