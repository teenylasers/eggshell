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
// LinearReducer.
// Given a matrix A, solve problems involving principal submatrices of A.
// Only the lower triangle of A is ever accessed.

class LinearReducer {
 public:
  LinearReducer(const MatrixXd &A, const VectorXd &b);
  virtual ~LinearReducer();

  int Asize() const { return A_.rows(); }

  // Access the index set.
  bool HasIndex(int i) const;

  // Add or remove indexes from the index set.
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

  // For debugging, print the internal state to stdout.
  void DebuggingPrintState() const;

 private:
  // A permutation of A,x,b. perm_[i] is the original index of current index i.
  // iperm_ is the inverse of perm_.
  std::vector<int> perm_, iperm_;
  MatrixXd A_;    // Original A permuted by perm_
  VectorXd x_;    // Solution of A*x = b, permuted by perm_
  MatrixXd L_;    // Cholesky factorization of Aii (top/left of A in index set)
  int index_;     // All permuted indexes < index_ are in the index set

  // Swap rows and columns of A_, b_, x_. Permuted (not original) indexes i and
  // j are given.
  void SwapRowsAndColumns(int i, int j);

  DISALLOW_COPY_AND_ASSIGN(LinearReducer);
};

//***************************************************************************
// LCP.

// Use the Murty principal pivoting method to solve the LCP problem A*x=b+w,
// x>=0, w>=0, x'*w=0. Murty guarantees convergence for P-matrices, we will use
// only real symmetric positive definite matrices (which are also P-matrices).
// The algorithm is optimized by assuming that the solution has a small number
// of zeros in x, letting us use the LinearReducer to best advantage.
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

// Solve the standard LCP problem. Return true on success or false if a
// solution can not be found.
bool SolveLCP(const MatrixXd &A, const VectorXd &b,
              VectorXd *x, VectorXd *w) MUST_USE_RESULT;

// Solve the "Box LCP" problem. Return true on success or false if a solution
// can not be found.
bool SolveBoxLCP(const MatrixXd &A, const VectorXd &b,
                  const VectorXd &lo, const VectorXd &hi,
                  VectorXd *x, VectorXd *w) MUST_USE_RESULT;

}  // namespace lcp

#endif
