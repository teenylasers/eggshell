#ifndef __UTILS_H__
#define __UTILS_H__

#include <array>

#include "Eigen/Dense"

using Eigen::ArrayXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

typedef Eigen::Array<bool, Eigen::Dynamic, 1> ArrayXb;

// Check whether columns of rotation matrix R are orthonormal
bool IsOrthonormal(const Matrix3d& R);

// Generate the cross-product matrix for vector a, so that a x b = a_bar * b.
Matrix3d CrossMat(const Vector3d& a);

// Convert angular velocity w to equivalent quaternion
// q = [cos(|w|/2) 1/2*sinc(|w|/2) * w]
Quaterniond WtoQ(const Vector3d& w, double dt);

// Generate a random position with the confines of x/y/z_bound.
Vector3d RandomPosition(const std::array<double, 2>& x_bound,
                        const std::array<double, 2>& y_bound,
                        const std::array<double, 2>& z_bound);
// Generate a random velocity with a magnitude limit.
Vector3d RandomVelocity(double limit);
// Generate a random angular velocity with a magnitude limit.
Vector3d RandomAngularVelocity(double limit);
// Generate a random rotation matrix.
Matrix3d RandomRotation();
Matrix3d RandomRotationViaQuaternion();
Matrix3d RandomRotationViaGramSchmidt();

Matrix3d GramSchmidt(const Matrix3d& m);

/////////////////////////////////////////////////////////////////////////

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
// ind(true) and m should match.
void UpdateSubmatrix(MatrixXd& A, const ArrayXb& row_ind,
                     const ArrayXb& col_ind, const MatrixXd& m);

// Update subvector of v indicated by ind with content of n. Dimensions of
// ind(true) and n should match.
void UpdateSubvector(VectorXd& v, const ArrayXb& ind, const VectorXd& n);

// Update subvector of v indicated by ind with d.
void UpdateSubvector(VectorXd& v, const ArrayXb& ind, double d);

/////////////////////////////////////////////////////////////////////////

// Randomly generate a well-conditioned SPD matrix.
MatrixXd GenerateSPDMatrix(int dimension);

// Randomly generate a well-conditioned diagonal-dominant matrix.
MatrixXd GenerateDiagonalDominantMatrix(int dimension);

// Return the rotation matrix that aligns vector a to vector b
Matrix3d AlignVectors(const Vector3d& a, const Vector3d& b);

// Return the condition number of a matrix
double GetConditionNumber(const MatrixXd& A);

// Return the spectral radius of a matrix
double GetSpectralRadius(const MatrixXd& A);

// Check whether matrix A is singular or rank-deficient.
bool CheckMatrixCondition(const MatrixXd& A);

// Return matrix sparsity, the proportion of the elements that are zero.
double GetMatrixSparsity(const MatrixXd& A);

// Return matrix block sparsity, the proportion of N x M blocks that are all
// zero, where N is any integer, M is block width (3 or 6).
double GetMatrixBlockSparsity(const MatrixXd& A, int block_width = 6);

// Invert special matrices
MatrixXd InvertDiagonalMatrix(const MatrixXd& D);

#endif
