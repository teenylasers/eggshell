// Copyright (C) 2014-2021 Russell Smith.
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

// Solve the Linear Complementarity problem (LCP).

#ifndef __TOOLKIT_LCP_H__
#define __TOOLKIT_LCP_H__

#include <vector>
#include <Eigen/Dense>
#include "error.h"

namespace lcp {

using Eigen::VectorXd;
using Eigen::MatrixXd;

//***************************************************************************
// MatrixPermutation.
// Maintain a permutation of a matrix A, and allow vectors to be permuted and
// unpermuted according to that permutation. We distinguish between original
// indexes that apply to the original matrix A, and permuted indexes that apply
// to the matrix A that we currently have.

class MatrixPermutation {
 public:
  explicit MatrixPermutation(MatrixXd &A);

  int Asize() const { return A_.rows(); }

  // Swap rows and columns of A_ and x_. Permuted (not original) indexes i and
  // j are given.
  void SwapRowsAndColumns(int i, int j);

  // Permute the original vector 'in' to get the permuted vector 'out'.
  void Permute(const VectorXd &in, VectorXd *out) const;

  // Unpermute the permuted vector 'in' to get the original vector 'out'.
  // Optionally start from the given (permuted) index.
  void Unpermute(const VectorXd &in, VectorXd *out, int start_index = 0) const;

  // Return the permuted index for original index i.
  int PermutedIndexOf(int i) const { return iperm_[i]; }

  // Return the original index for permuted index i.
  int OriginalIndexOf(int i) const { return perm_[i]; }

 protected:
  MatrixXd &A_;   // Original A, permuted by perm_

 private:
  // perm_[i] is the original index of current index i. iperm_ is the inverse
  // of perm_.
  std::vector<int> perm_, iperm_;

  DISALLOW_COPY_AND_ASSIGN(MatrixPermutation);
};

//***************************************************************************
// LinearReducer.
// This maintains a factorization of a top left submatrix of the matrix A. The
// factorization is of all (original) indexes that are in the "index set".
// Allow rows to be swapped in to and out of this factorization. Only the lower
// triangle of A is ever accessed. A is permuted in place.

class LinearReducer : public MatrixPermutation {
 public:
  LinearReducer(MatrixXd &A, const VectorXd &b);

  // Is (original, not permuted) index i in the index set?
  bool HasIndex(int i) const;

  // Add or remove (original, not permuted) indexes from the index set.
  void AddIndex(int i);
  void RemoveIndex(int i);

  // Solve a sub-problem in A. Solve A*x=b for x, with x(~index_set) forced to
  // the 'clamp' values in c(~index_set) by adjusting b(~index_set). When c=0
  // this is equivalent to solving the principal subproblem:
  // A(index_set,index_set)*x(index_set) = b(index_set)
  void SubSolve(const VectorXd &c, VectorXd *x);

  // Compute b = A*x. The b vector is only filled in for indexes *not* in the
  // index set.
  void MultiplyA(const VectorXd &x, VectorXd *b) const;

 private:
  VectorXd x_;    // Solution of A*x = b, permuted by perm_
  MatrixXd L_;    // Cholesky factorization of Aii (top/left of A in index set)
  int index_;     // All permuted indexes < index_ are in the index set

  void SwapRowsAndColumns(int i, int j);

  DISALLOW_COPY_AND_ASSIGN(LinearReducer);
};

//***************************************************************************
// LCP.

// Use the Murty principal pivoting method to solve the LCP problem A*x=b+w,
// x>=0, w>=0, x'*w=0. Murty guarantees convergence for P-matrices, we will use
// only real symmetric positive definite matrices (which are also P-matrices).
// The algorithm is optimized by assuming that the solution has a smallish
// number of zeros in x, letting us use the LinearReducer to best advantage.
//
// In the "Box LCP" version of this problem we solve A*x = b + w, with x[i] and
// w[i] on one of the three line segments in this diagram:
//
//        w[i]
//        /|\      | index_set[i]=false
//         |       | c[i] = lo[i]
//         |       |
//     w>0 |       |
//         |       |    index_set[i]=true
//     w=0 +       +-----------+-----------+
//         |                               |
//     w<0 |                               |
//         |                               | index_set[i]=false
//         |                               | c[i] = hi[i]
//         |                               |
//         +-------|-----------|-----------|----------> x[i]
//                lo[i]       x=0         hi[i]
//
// We must have lo <= 0 and hi >= 0.

// The LCP solver algorithms.
enum Algorithm {
  // Principal pivoting method described in: Murty, K. G. (1988). Linear
  // complementarity, linear and nonlinear programming.
  MURTY,

  // Principal pivoting method described in: Cottle, R. W.; Dantzig, G. B.
  // (1968). "Complementary pivot theory of mathematical programming". Linear
  // Algebra and its Applications. 1: 103–125.
  COTTLE_DANTZIG,
};

// LCP solver settings. Here, "infinity" is taken to be either the maximum
// representable double (__DBL_MAX__) or the actual infinity value.
struct Settings {
  // The algorithm to use.
  Algorithm algorithm = MURTY;

  // If true, solve the box LCP problem. Vectors lo and hi must be valid. If
  // false, this is equivalent to lo=0 and hi=infinity.
  bool box_lcp = true;

  // Use a Schur complement approach to speed up solutions where large numbers
  // of unbounded variables are present (i.e. lo=-infinity, hi=infinity).
  bool schur_complement = true;

  // Maximum number of solver iterations to run before giving up and returning
  // false. However it is often not possible to know what a reasonable limit
  // might be, which is why max_time also exists.
  int max_iterations = __INT_MAX__;

  // Maximum wall clock time to run before giving up and returning false.
  double max_time = __DBL_MAX__;
};

// Solve the LCP problem. Return true on success or false if a solution
// can not be found. The matrix 'A' is permuted in-place.
bool SolveLCP(const Settings &settings, MatrixXd &A, const VectorXd &b,
              const VectorXd &lo, const VectorXd &hi,
              VectorXd *x, VectorXd *w) MUST_USE_RESULT;

}  // namespace lcp

#endif
