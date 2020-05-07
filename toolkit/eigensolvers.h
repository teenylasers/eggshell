// Copyright (C) 2014-2020 Russell Smith.
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.

// We want to solve for the M smallest-in-magnitude eigenpairs of
//
//   A*x = lambda * B*x
//
// Where 'A' and 'B' are symmetric, 'B' is positive definite, and 'A' is
// positive definite or positive semi-definite. When 'A' represents a Laplacian
// it is positive definite with Dirichlet boundary conditions and positive
// semi-definite with Neumann boundary conditions, as any constant offset can
// be added to the solution. We call this the 'laplacian eigenproblem'.
//
// With Dirichlet boundary conditions we can use the shift-and-invert mode of
// ARPACK with sigma=0 to estimate the largest magnitude eigenvalues of the
// inverse problem, as this is somewhat faster than using 'which=SM'. This
// shift-and-invert mode requires us to compute inv(A)*B. For the Neumann
// boundary conditions however 'A' is positive semi-definite (i.e. singular) so
// this is not possible. Therefore we use sigma < 0 so that we compute
// inv(A-sigma*B))*B, which pushes A to the positive definite (and therefore
// nonsingular) matrix A-sigma*B, allowing for a Cholesky or LDLT solution. We
// will recover eigenvalues sorted by their distance from sigma, but since all
// eigenvalues are >=0 this does not change the recovered order. To avoid
// messing up ARPACK convergence however we use a negative sigma that is only
// up to -1 times the expected lowest magnitude positive eigenvalue (call this
// lambda1). In a laplacian system the lambda1 estimate can come from the
// natural frequency of a rectangular cavity that bounds the model.

#ifndef __TOOLKIT_EIGENSOLVERS_H__
#define __TOOLKIT_EIGENSOLVERS_H__

#include <vector>
#include <Eigen/Dense>
#include "error.h"
#include "myvector"

namespace eigensolvers {

// Debugging mode.
inline bool kDebug() { return false; }

using namespace Eigen;
using std::vector;

// ARPACK entry points (fortran interface).
extern "C" void dsaupd_(int *ido, char *bmat, int *n, char *which, int *nev,
                        double *tol, double *resid, int *ncv, double *v,
                        int *ldv, int *iparam, int *ipntr, double *workd,
                        double *workl, int *lworkl, int *info, int bmat_length,
                        int which_length);
extern "C" void dseupd_(int *rvec, char *All, int *select, double *d, double *z,
                        int *ldz, double *sigma, char *bmat, int *n,
                        char *which, int *nev, double *tol, double *resid,
                        int *ncv, double *v, int *ldv, int *iparam, int *ipntr,
                        double *workd, double *workl, int *lworkl, int *ierr,
                        int All_length, int bmat_length, int which_length);

class LaplacianEigenSolver {
public:
  typedef SparseMatrix<double> SMatrix;

  // Solve the laplacian eigenproblem. If the 'B' pointer is null then B is
  // taken to be the identity matrix. If 'A' is positive semidefinite (i.e. has
  // a zero eigenvalue) then 'sigma' must be some fraction 0<sigma<1 of the
  // smallest eigenvalue. If 'A' is positive definite then sigma can be zero.
  // The 'num_eigenpairs' is clamped to A.cols()-1, which is the largest number
  // of eigenpairs that ARPACK can recover.
  LaplacianEigenSolver(const SMatrix &A, const SMatrix *B,
                       int num_eigenpairs, double sigma);

  // Return the status of the eigensearch.
  ComputationInfo Status() const { return status_; }
  int GetNumConvergedEigenvalues() const { return num_converged_; }
  int GetNumIterations() const { return num_iterations_; }

  // Return the eigenvalue vector. Note that the vector size might be less than
  // num_eigenpairs.
  const VectorXd &GetEigenValues() const { return eigenvalues_; }

  // Return the eigenvector matrix (one eigenvector per column). Note that the
  // number of columns might be less than num_eigenpairs.
  const MatrixXd &GetEigenVectors() const { return eigenvectors_; }

 private:
  VectorXd eigenvalues_;
  MatrixXd eigenvectors_;
  ComputationInfo status_;
  int num_converged_, num_iterations_;
};

inline
LaplacianEigenSolver::LaplacianEigenSolver(const SMatrix &A, const SMatrix *B,
                                           int num_eigenpairs, double sigma) {
  status_ = Success;
  num_converged_ = 0;
  num_iterations_ = 0;

  // Check arguments.
  CHECK(A.cols() == A.rows());                  // A square
  if (B) {
    CHECK(B->cols() == B->rows());              // B square
  }

  // For debugging, check that A and B are symmetric.
  if (kDebug()) {
    SMatrix Acheck = A - SMatrix(A.transpose());
    double one = 1;
    Acheck.prune(one);
    CHECK(Acheck.nonZeros() == 0);
    if (B) {
      SMatrix Bcheck = (*B) - SMatrix(B->transpose());
      Bcheck.prune(one);
      CHECK(Bcheck.nonZeros() == 0);
    }
  }

  // B is sometimes badly scaled, affecting the convergence of ARPACK. Rescale
  // it. We will have to rescale the eigenvalues too, but the eigenvectors
  // should not change.
  double Bscale = 0;
  if (B) {
    CHECK(B->isCompressed());
    const double *Bvalues = B->valuePtr();
    for (int i = 0; i < B->nonZeros(); i++) {
      Bscale = std::max(Bscale, fabs(Bvalues[i]));
    }
    CHECK(Bscale > 0);
    sigma *= Bscale;
  }

  // Factor A-sigma*B.
  Eigen::SimplicialLDLT<SMatrix> OP;
  if (B && sigma != 0) {
    OP.compute(A - (sigma / Bscale) * (*B));
  } else {
    if (sigma != 0) {
      SMatrix I(A.rows(), A.cols());
      I.setIdentity();
      OP.compute(A - sigma * I);
    } else {
      OP.compute(A);
    }
  }
  if (OP.info() != Eigen::Success) {
    // Factorization failed.
    status_ = Eigen::NumericalIssue;
    return;
  }

  // ARPACK parameters to dseupd_. ncv is the number of Lanczos vectors, the
  // optimal value is problem-dependent.
  int ido = 0;                  // zero for the first call
  char bmat[2] = "I";           // B is the identity
  if (B) bmat[0] = 'G';         // Generalized problem?
  int n = A.cols();             // Size of the problem
  char which[3] = "LM";         // Largest magnitude eigenvals of inverted prob
  int nev = std::min(num_eigenpairs, n - 1);    // Required: nev < n
  double tol = 0;               // Tolerance, 0 means machine precision
  vector<double> resid(n);      // residual space
  int ncv = std::min(2 * nev, n);       // Required: nev < ncv <= n
  vector<double> v(n * ncv);            // working matrix, n * ncv
  int ldv = n;
  vector<int> iparam(11);
  iparam[0] = 1;            // 1 to have ARPACK do the shifts
  iparam[2] = 1000;         // Maximum iterations
  iparam[3] = 1;            // nb, blocksize to be used in the recurrence
  iparam[6] = 3;            // Shift and invert mode
  vector<int> ipntr(11);    // Reverse communication array
  vector<double> workd(3 * n);          // Working space
  int lworkl = ncv * ncv + 8 * ncv;
  vector<double> workl(lworkl);         // Working space
  int info = 0;             // 0 to use random initial residual vector

  for (;;) {
    // The last two arguments to dsaupd_ are the hidden parameters that
    // describe the length of the bmat and which strings.
    dsaupd_(&ido, bmat, &n, which, &nev, &tol, resid.data(), &ncv, v.data(),
            &ldv, iparam.data(), ipntr.data(), workd.data(), workl.data(),
            &lworkl, &info, 1, 2);
    if (ido == 99) {
      break;
    }
    double *X = &workd[ipntr[0] - 1];
    double *Y = &workd[ipntr[1] - 1];
    if (ido == -1 || ido == 1) {
      // Compute Y = OP * X, with OP = (inv[A - sigma*B])*B
      if (B) {
        if (ido == 1) {
          // In modes 3,4,5 for ido==1 we have B*X already available
          double *BX = &workd[ipntr[2] - 1];
          VectorXd::Map(Y, n) = OP.solve(VectorXd::Map(BX, n));
        } else {
          VectorXd::Map(Y, n) = OP.solve(*B * VectorXd::Map(X, n)) / Bscale;
        }
      } else {
        VectorXd::Map(Y, n) = OP.solve(VectorXd::Map(X, n));
      }
    } else if (ido == 2) {
      // Compute Y = B * X. This is only used when bmat=G, i.e. B is supplied.
      VectorXd::Map(Y, n) = ((*B) * VectorXd::Map(X, n)) / Bscale;
    } else {
      Panic("unhandled ido of %d", ido);
    }
  }
  num_iterations_ = iparam[2];
  num_converged_ = iparam[4];

  // Handle exit code.
  if (info < 0) {
    Panic("ARPACK invalid input (code %d), see dsaupd.f", info);
  }
  if (info == 1) {
    status_ = NoConvergence;
    return;
  } else if (info == 3) {
    status_ = NumericalIssue;
    return;
  } else if (info != 0) {
    Panic("Unexpected return value from dsaupd.f");
  }

  // ARPACK parameters to dseupd_.
  int rvec = 1;                 // Compute eigenvectors
  char howmny[2] = "A";         // Compute all eigenvectors
  vector<int> select(ncv);
  eigenvalues_.resize(nev);
  // The last three arguments to dseupd_ are the hidden parameters that
  // describe the length of the howmny, bmat and which strings.
  eigenvectors_.resize(A.rows(), nev);
  dseupd_(&rvec, howmny, select.data(), eigenvalues_.data(),
          eigenvectors_.data(), &ldv, &sigma, bmat, &n, which, &nev, &tol,
          resid.data(), &ncv, v.data(), &ldv, iparam.data(), ipntr.data(),
          workd.data(), workl.data(), &lworkl, &info, 1, 1, 2);
  if (info == -14) {
    status_ = NoConvergence;
    return;
  } else if (info != 0) {
    Panic("ARPACK invalid input (code %d), see dseupd.f", info);
  }

  // Unscale eigenvalues if necessary.
  if (B) {
    eigenvalues_ /= Bscale;
  }

  // I *think* that ARPACK guarantees sorted eigenvalues from dsaupd_/dseupd_,
  // though this is not clearly stated in the manual. Check it here.
  for (int i = 1; i < eigenvalues_.size(); i++) {
    CHECK(eigenvalues_[i] >= eigenvalues_[i-1]);
  }

  status_ = Success;
}

}  // namespace eigensolvers

#endif
