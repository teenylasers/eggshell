// Library of lcp solvers

#ifndef __LCP_H__
#define __LCP_H__

#include "Eigen/Dense"

using namespace Eigen;

namespace Lcp {

bool MurtyPrinciplePivot(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                         VectorXd& w);

// TODO: combine matrix traverse for select and update
// TODO: move this function to somewhere that extends Eigen
// TODO: test results against Matlab.
// TODO: make ind an array of boolean instead of int
// TODO: combine for matrices and vectors.
// TODO: return type should work for all element types: d, f, i
// TODO: ArrayXi implementation is error-prone, elements are assumed to be 0 or
// 1 but are not enforced to be.
// Select submatrix from A using a vector ind of indices, return the results in
// Matrix ret. No modifications to input A or ind.
MatrixXd SelectSubmatrix(const MatrixXd& A, const ArrayXi& row_ind,
                         const ArrayXi& col_ind);
VectorXd SelectSubvector(const VectorXd& v, const ArrayXi& ind);
// Update submatrix of A indicated by ind with content of m. Dimensions of
// ind(true) and m shouild match.
void UpdateSubmatrix(MatrixXd& A, const ArrayXi& row_ind,
                     const ArrayXi& col_ind, const MatrixXd& m);
void UpdateSubvector(VectorXd& v, const ArrayXi& ind, const VectorXd& n);
void UpdateSubvector(VectorXd& v, const ArrayXi& ind, double d);

}  // namespace Lcp

#endif
