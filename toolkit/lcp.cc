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

#include <stdio.h>
#include <iostream>
#include <utility>
#include "lcp.h"
#include "testing.h"
#include "random.h"

using Eigen::MatrixXd;
using std::vector;

namespace lcp {

//***************************************************************************
// More convenient access to Eigen's LLT Cholesky solver. This provides an
// important feature missing from Eigen: the ability to perform rank updates on
// already-factored matrices.

// Below we peek into some of Eigen's internals. If we upgrade Eigen, make sure
// that this code still works as expected.
#define EIGEN_VERSION_IS(x, y, z) (EIGEN_WORLD_VERSION == x && \
    EIGEN_MAJOR_VERSION == y && EIGEN_MINOR_VERSION == z)
#if !EIGEN_VERSION_IS(3, 3, 8) && !EIGEN_VERSION_IS(3, 3, 9)
  #error "Unexpected Eigen version. Update check after ensuring code is ok."
#endif

// Perform an in-place Cholesky factorization on an SPD matrix A to give a
// matrix L. Only the lower triangle is accessed.
inline void Cholesky(MatrixXd *A_and_L) {
  Eigen::LLT<Eigen::Ref<MatrixXd>> llt(*A_and_L);
}

// Solve in place: L*x = b, regarding just the top left n*n block of L. Only
// the lower triangle of L is read.
inline void LSolve(const MatrixXd &L, int n, VectorXd *x_and_b) {
  L.block(0, 0, n, n).triangularView<Eigen::Lower>().solveInPlace(*x_and_b);
}

// Solve L*L'*x = b, regarding just the top left n*n block of L. Only the lower
// triangle of L is read.
inline void LLTSolve(const MatrixXd &L, int n, VectorXd *x_and_b) {
  L.block(0, 0, n, n).triangularView<Eigen::Lower>().solveInPlace(*x_and_b);
  L.block(0, 0, n, n).adjoint().triangularView<Eigen::Upper>()
   .solveInPlace(*x_and_b);
}

// Perform an in-place rank update on a block(i,i,p,p) of an already-factored
// L. Only the lower triangle of L is read.
inline void RankUpdate(int i, int p, const VectorXd &vec, double sigma,
                       MatrixXd *L) {
  int n = L->rows();
  Eigen::Map<MatrixXd, 0, Eigen::OuterStride<>> Lmap(L->data() + i + i*n, p, p,
                                                     Eigen::OuterStride<>(n));
  Eigen::internal::llt_rank_update_lower(Lmap, vec, sigma);
}

//***************************************************************************
// Modify factorizations.

// On entry the top left (n-1)*(n-1) block of L is a factorization of the top
// left (n-1)*(n-1) block of A. Fill in row n-1 of L so that now the n*n
// top-left block of L is a complete factorization of the n*n top-left block of
// A. Only the lower triangle of A and L is accessed.

static void AddCholeskyRow(const MatrixXd &A, int n, MatrixXd *L) {
  // Perform one further iteration of Choleksy decomposition.
  CHECK(n >= 0 && A.rows() >= n);
  if (n == 1) {
    (*L)(0, 0) = sqrt(A(0, 0));
  } else {
    VectorXd ell(A.block(n-1, 0, 1, n-1).transpose());
    LSolve(*L, n-1, &ell);
    L->block(n-1, 0, 1, n-1) = ell.transpose();
    (*L)(n-1, n-1) = sqrt(A(n-1, n-1) - ell.squaredNorm());
  }
}

// Given an n*n factorization L of an n*n matrix A, modify L so that it is an
// (n-1)*(n-1) factorization of A with row and column n-1 moved to i. A is not
// actually changed. Only the top left n*n blocks of A and L are considered, A
// and L can actually be larger. Only the lower triangle of A and L is
// accessed.

static void SwapCholeskyRows(const MatrixXd &A, int i, int n, MatrixXd *L) {
  if (n <= 1 || i == n-1) {
    return;
  }

  if (i == 0) {
    // Perform a rank-2 update to L. Note that for scalar b and column vector
    // a, if wq=[b/2+1;a] and wr=[b/2-1;a] then:
    //   wq*wq'/2 - wr*wr'/2 == [b,a'; a,zeros]
    // So we can adjust the top/left of A with a rank 2 update by selecting b
    // and a appropriately.
    VectorXd wq = A.block(n-1, 0, 1, n-1).transpose() - A.block(0, 0, n-1, 1);
    wq[0] = (A(n-1, n-1) - A(0, 0))*0.5 + 1;
    RankUpdate(0, n-1, wq, 0.5, L);
    wq[0] = (A(n-1, n-1) - A(0, 0))*0.5 - 1;
    RankUpdate(0, n-1, wq, -0.5, L);
  } else {
    // Partitioning the factorization       |1 column
    //   [ L1  0  0 ] [ L1' l1 L2' ]   [ A  a B' ]
    //   [ l1' e  0 ] [  0  e  l2' ] = [ a' b c' ]  <-- 1 row
    //   [ L2  l2 E ] [  0  0   E' ]   [ B  c C  ]
    // We are changing a,b,c. Expanding L*L' and see what changes:
    //   L1 l1 = new_a                <-- solve for l1
    //   l1'l1 + e^2 = new_b          <-- solve for e
    //   L2 l1 + l2 e = new_c         <-- solve for l2
    //   L2 L2' + l2 l2' + E E' = C   <-- rank 2 update to E
    VectorXd l1 = A.block(n-1, 0, 1, i).transpose();
    LSolve(*L, i, &l1);
    auto e = sqrt(A(n-1, n-1) - l1.dot(l1));
    (*L)(i, i) = e;
    L->block(i, 0, 1, i) = l1.transpose();
    if (i < n-2) {
      // Rank update the original l2.
      RankUpdate(i+1, n-2-i, L->block(i+1, i, n-2-i, 1), 1, L);
      L->block(i+1, i, n-2-i, 1) = (A.block(n-1, i+1, 1, n-2-i).transpose() -
                                    L->block(i+1, 0, n-2-i, i) * l1) / e;
      // Rank update with the new l2.
      RankUpdate(i+1, n-2-i, L->block(i+1, i, n-2-i, 1), -1, L);
    }
  }
}

//***************************************************************************
// MatrixPermutation.

MatrixPermutation::MatrixPermutation(MatrixXd &A) : A_(A) {
  perm_.resize(A_.rows());
  iperm_.resize(A_.rows());
  for (int i = 0; i < A_.rows(); i++) {
    perm_[i] = i;
    iperm_[i] = i;
  }
}

void MatrixPermutation::SwapRowsAndColumns(int i, int j) {
  // Note that we physically move data around rather than simply adjusting a
  // permutation and doing later indirection. This is because data locality is
  // important for the speed of big matrix operations, so the cost of data
  // movement pays off.
  if (i == j) {
    return;
  }
  if (i > j) {
    std::swap(i, j);      // Enforce i < j
  }
  int i2 = perm_[i];      // perm_ maps current index to original
  int j2 = perm_[j];
  std::swap(perm_[i], perm_[j]);
  std::swap(iperm_[i2], iperm_[j2]);
  // For swapping in A_ we can't do the following since we only access the
  // lower triangle:
  //   A_.col(i).swap(A_.col(j));
  //   A_.row(i).swap(A_.row(j));
  int n = A_.rows();
  A_.block(i, 0, 1, i).swap(A_.block(j, 0, 1, i));
  A_.block(j+1, i, n-j-1, 1).swap(A_.block(j+1, j, n-j-1, 1));
  A_.block(i+1, i, j-i-1, 1).swap(A_.block(j, i+1, 1, j-i-1).transpose());
  std::swap(A_(i, i), A_(j, j));
}

void MatrixPermutation::Permute(const VectorXd &in, VectorXd *out) const {
  for (int i = 0; i < perm_.size(); i++) {
    (*out)[i] = in[perm_[i]];
  }
}

void MatrixPermutation::Unpermute(const VectorXd &in, VectorXd *out,
                                  int start_index) const {
  for (int i = start_index; i < perm_.size(); i++) {
    (*out)[perm_[i]] = in[i];
  }
}

//***************************************************************************
// LinearReducer.

LinearReducer::LinearReducer(MatrixXd &A, const VectorXd &b)
    : MatrixPermutation(A) {
  index_ = A.rows();

  // Factor A into L_, solve for x_.
  L_ = A;
  Cholesky(&L_);
  x_ = b;
  LLTSolve(L_, L_.rows(), &x_);
}

LinearReducer::~LinearReducer() {
}

bool LinearReducer::HasIndex(int i) const {
  return PermutedIndexOf(i) < index_;
}

void LinearReducer::AddIndex(int i) {
  i = PermutedIndexOf(i);
  if (i >= index_) {
    SwapRowsAndColumns(index_, i);
    index_++;
    // Update the factorization.
    AddCholeskyRow(A_, index_, &L_);
  }
}

void LinearReducer::RemoveIndex(int i) {
  i = PermutedIndexOf(i);
  if (i < index_) {
    SwapCholeskyRows(A_, i, index_, &L_);
    index_--;
    SwapRowsAndColumns(index_, i);
  }
}

void LinearReducer::SubSolve(const VectorXd &c, VectorXd *x) {
  // Partitioning the variables of the A*x=b matrix equation (with i=index_set,
  // n=~index_set) gives:
  //   [ Aii Ain ] [ xi ] = [ bi ]
  //   [ Ani Ann ] [ xn ] = [ bn ]
  // We want to add something to bn to force xn to cn, and then determine the
  // effect on xi. Let's say we have a solution Ei,En of:
  //   [ Aii Ain ] [ Ei ] = [ 0 ]       (eq. 1)
  //   [ Ani Ann ] [ En ] = [ I ]  <-- I is the identity matrix
  // Multiplying this on the right by a vector k gives:
  //   [ Aii Ain ] [ Ei*k ] = [ 0 ]
  //   [ Ani Ann ] [ En*k ] = [ k ]
  // Adding the above to our original A*x=b solution gives:
  //   [ Aii Ain ] [ xi + Ei*k ] = [ bi     ]
  //   [ Ani Ann ] [ xn + En*k ] = [ bn + k ]
  // So we need to solve for k to satisfy: xn + En*k = cn. Once we have that we
  // can compute new_xi = xi + Ei*k = xi + Ei*inv(En)*(cn - xn).
  // From eq. 1 we have
  //   Ei = -inv(Aii)*Ain*En
  // so
  //   new_xi = xi - inv(Aii)*Ain*En*inv(En)*(cn - xn).
  //          = xi - inv(Aii)*Ain*(cn - xn).

  // Handle the easy cases.
  int n = A_.rows();
  if (index_ == 0) {            // An easy case, all x values clamped
    *x = c;
    return;
  } else if (index_ >= n) {     // An easy case, no x values clamped
    x->resize(n);
    Unpermute(x_, x);
    return;
  }

  // Permute c to match our A.
  VectorXd c2(n);
  Permute(c, &c2);

  // Compute xi -= inv(Aii) Ain (cn - xn)
  VectorXd newx_head = A_.block(index_, 0, n - index_, index_).transpose() *
                       (c2.tail(n - index_) - x_.tail(n - index_));
  LLTSolve(L_, index_, &newx_head);

  // Unpermute.
  x->resize(n);
  for (int i = 0; i < index_; i++) {
    (*x)[OriginalIndexOf(i)] = x_[i] - newx_head[i];
  }
  for (int i = index_; i < n; i++) {
    (*x)[OriginalIndexOf(i)] = c[OriginalIndexOf(i)];
  }
}

void LinearReducer::MultiplyA(const VectorXd &x, VectorXd *b) const {
  int n = A_.rows();
  if (index_ >= n) {
    // No indexes not in the index set, nothing to do, but caller will still
    // expect an allocated 'b' vector.
    b->resize(n);
    return;
  }

  // Permute x to match our A.
  VectorXd x2(n);
  Permute(x, &x2);

  // Do the multiplication, but only for indexes not in the index set.
  VectorXd b2(n);
  b2.tail(n - index_) = A_.block(index_, 0, n - index_, index_) * x2.head(index_) +
      A_.block(index_, index_, n - index_, n - index_).selfadjointView<Eigen::Lower>()
       * x2.tail(n - index_);

  // Unpermute (indexes not in the index set only).
  b->resize(n);
  Unpermute(b2, b, index_);
}

void LinearReducer::SwapRowsAndColumns(int i, int j) {
  MatrixPermutation::SwapRowsAndColumns(i, j);
  std::swap(x_[i], x_[j]);
}

//***************************************************************************
// LCP.

bool SolveLCP_Murty(const Settings &settings,
                    MatrixXd &A, const VectorXd &b,
                    VectorXd *x, VectorXd *w) {
  int n = A.rows();
  LinearReducer reducer(A, b);
  VectorXd c(n);            // Clamp values, which will always be zero
  c.setZero();

  // The reducer index set is the indexes that are nonzero in x. This is
  // initially all indexes.
  for (int iteration = 0; iteration < settings.max_iterations; iteration++) {
    // Compute x for this index set.
    reducer.SubSolve(c, x);

    // Compute w = A*x - b. This doesn't compute elements of w that are known
    // to be zero.
    reducer.MultiplyA(*x, w);
    for (int i = 0; i < n; i++) {
      if (reducer.HasIndex(i)) {
        (*w)[i] = 0;
      } else {
        (*w)[i] -= b[i];
      }
    }

    // Check to see if we have a solution. If not, add or remove an index from
    // the index set and try again.
    for (int i = 0; i < n; i++) {
      if (reducer.HasIndex(i) && ((*x)[i] < 0 || (*w)[i] > 0)) {
        reducer.RemoveIndex(i);
        goto retry;
      }
      if (!reducer.HasIndex(i) && ((*x)[i] > 0 || (*w)[i] < 0)) {
        reducer.AddIndex(i);
        goto retry;
      }
    }

    // We have a correct solution.
    return true;
   retry:;
  }

  // We exceeded the maximum number of iterations.
  return false;
}

bool SolveLCP_BoxMurty(const Settings &settings,
                       MatrixXd &A, const VectorXd &b,
                       const VectorXd &lo, const VectorXd &hi,
                       VectorXd *x, VectorXd *w) {
  int n = A.rows();
  LinearReducer reducer(A, b);

  // The reducer index set is the indexes where w=0 and lo<x<hi. This is
  // initially all indexes.
  VectorXd c(n);      // Clamp vector
  c.setZero();
  for (int iteration = 0; iteration < settings.max_iterations; iteration++) {
    // Compute x for this index set.
    reducer.SubSolve(c, x);

    // Compute w = A*x - b. This doesn't compute elements of w that are known
    // to be zero.
    reducer.MultiplyA(*x, w);
    for (int i = 0; i < n; i++) {
      if (reducer.HasIndex(i)) {
        (*w)[i] = 0;
      } else {
        (*w)[i] -= b[i];
      }
    }

    // Check to see if this is a solution. If not, add or remove an index from
    // the index set and try again.
    for (int i = 0; i < n; i++) {
      if (reducer.HasIndex(i)) {
        if ((*x)[i] < lo[i]) {
          reducer.RemoveIndex(i);
          c[i] = lo[i];
          goto retry;
        } else if ((*x)[i] > hi[i]) {
          reducer.RemoveIndex(i);
          c[i] = hi[i];
          goto retry;
        }
      } else {
        if (c[i] == lo[i] && (*w)[i] < 0) {
          reducer.AddIndex(i);
          c[i] = 0;
          goto retry;
        } else if (c[i] == hi[i] && (*w)[i] > 0) {
          reducer.AddIndex(i);
          c[i] = 0;
          goto retry;
        }
      }
    }

    // We have a correct solution.
    return true;
   retry:;
  }

  // We exceeded the maximum number of iterations.
  return false;
}

bool SolveLCP_BoxSchur(const Settings &settings,
                       MatrixXd &A, const VectorXd &b,
                       const VectorXd &lo, const VectorXd &hi,
                       VectorXd *x, VectorXd *w, int nub = -1) {
  // To solve the linear system
  //   [ Z B' ] [ y ] = [ c ]
  //   [ B C  ] [ z ] = [ d ]
  // by the Schur complement we do this:
  //   L*L' = Z             // Factor Z
  //   Q = inv(L)*B'        // Note: solving B rows one by one is inefficient
  //   R = C - Q'*Q         // R = C - B*inv(Z)*B'. Only compute lower triangle.
  //   t = inv(Z)*c         // Z solve
  //   z = d - B*t          // z = d - B*inv(Z)*c
  //   z = inv(R)*z         // z = inv(C - B*inv(Z)*B') (d - B*inv(Z)*c)
  //   y = inv(Z)*(c - B'*z)
  // One advantage here is that we refer to Z purely by its ability to solve
  // for some right hand side, so we do not necessarily need a dense matrix
  // representation of Z. Another advantage is that we can solve for z as an
  // LCP problem.

  // Identify the number of unbounded variables.
  int n = A.rows();
  if (nub < 0) {
    nub = 0;
    for (; nub < n; nub++) {
      if (lo[nub] > -__DBL_MAX__ || hi[nub] < -__DBL_MAX__) {
        break;                // This index is bounded
      }
    }
  }
  if (nub == n) {
    // This is not actually an LCP problem, so just solve it directly here.
    *x = A.llt().solve(b);
    w->resize(A.rows());
    w->setZero();
    return true;
  }

  // Form the sub-matrices of the Schur complement. Only refer to the lower
  // triangle of A, which means we only refer to the lower triangle of Z and C.
  typedef Eigen::Map<const MatrixXd, 0, Eigen::OuterStride<>> Map;
  Map Z(A.data(), nub, nub, Eigen::OuterStride<>(n));
  Map B(A.data() + nub, n-nub, nub, Eigen::OuterStride<>(n));
  Map C(A.data() + nub + n*nub, n-nub, n-nub, Eigen::OuterStride<>(n));
  Map c(b.data(), nub, 1, Eigen::OuterStride<>(n));
  Map d(b.data() + nub, n-nub, 1, Eigen::OuterStride<>(n));
  Map lo2(lo.data() + nub, n-nub, 1, Eigen::OuterStride<>(n));
  Map hi2(hi.data() + nub, n-nub, 1, Eigen::OuterStride<>(n));

  // Schur complement with LCP. Note we only compute the lower triangle of R.
  MatrixXd L = Z;
  lcp::Cholesky(&L);
  MatrixXd Q = L.triangularView<Eigen::Lower>().solve(B.transpose());
  MatrixXd R = C.triangularView<Eigen::Lower>();
  R.selfadjointView<Eigen::Lower>().rankUpdate(Q.transpose(), -1);
  VectorXd t = c;
  lcp::LLTSolve(L, nub, &t);
  // The non-LCP variant is:
  //   lcp::Cholesky(&R);
  //   VectorXd z = d - B*t;
  //   lcp::LLTSolve(R, n-nub, &z);
  VectorXd rhs = d - B*t;
  VectorXd z, w2;
  Settings settings2 = settings;
  settings2.schur_complement = false;
  if (!SolveLCP(settings2, R, rhs, lo2, hi2, &z, &w2)) {
    return false;
  }
  VectorXd y = c - B.transpose()*z;
  lcp::LLTSolve(L, nub, &y);

  // Assemble results.
  x->resize(n);
  x->head(nub) = y;
  x->tail(n - nub) = z;
  w->resize(n);
  w->head(nub).setZero();
  w->tail(n - nub) = w2;
  return true;
}

//***************************************************************************
// Drivers.

bool SolveLCP(const Settings &settings,
              MatrixXd &A, const VectorXd &b,
              const VectorXd &lo, const VectorXd &hi,
              VectorXd *x, VectorXd *w) {
  if (settings.schur_complement) {
    if (settings.box_lcp) {
      return SolveLCP_BoxSchur(settings, A, b, lo, hi, x, w);
    } else {
      Panic("Schur complement solver only available for box LCP");
    }
  }

  if (settings.algorithm == MURTY) {
    if (settings.box_lcp) {
      return SolveLCP_BoxMurty(settings, A, b, lo, hi, x, w);
    } else {
      return SolveLCP_Murty(settings, A, b, x, w);
    }
  } else {
    Panic("Unknown LCP solver selection");
  }
}

}  // namespace lcp

//***************************************************************************
// LCP testing.

namespace {

using Eigen::VectorXd;
using Eigen::MatrixXd;

const int N = 7;            // Problem sizes below

TEST_FUNCTION(LinearReducer) {
  for (int iteration = 0; iteration < 1000; iteration++) {
    printf("Iteration %d\n", iteration);

    // Create a random positive definite linear problem.
    MatrixXd A0 = MatrixXd::Random(N,N);
    MatrixXd A = A0*A0.transpose();
    VectorXd b = VectorXd::Random(N,1);
    VectorXd c = VectorXd::Random(N,1);         // Clamp vector
    // Only pass the lower triangle of A to LinearReducer, to ensure that it
    // only reads that.
    MatrixXd Alower = A.triangularView<Eigen::Lower>();
    lcp::LinearReducer lr(Alower, b);

    // Set a random index set. Make sure we test the corner cases of all
    // indexes added and all indexes removed.
    if (iteration == 0) {
      for (int i = 0; i < N; i++) {
        lr.AddIndex(i);
      }
    } else if (iteration == 1) {
      for (int i = 0; i < N; i++) {
        lr.RemoveIndex(i);
      }
    } else {
      for (int i = 0; i < N*2; i++) {
        if (RandomInt(2)) {
          lr.AddIndex(RandomInt(N));
        } else {
          lr.RemoveIndex(RandomInt(N));
        }
      }
    }
    printf("index_set =");
    for (int i = 0; i < N; i++) {
      printf(" %d", int(lr.HasIndex(i)));
    }
    printf("\n");

    // Solve the subproblem.
    VectorXd x;
    lr.SubSolve(c, &x);

    // Check that the solution has the requested clamp values outside the
    // index set.
    CHECK(x.size() == N);
    for (int i = 0; i < N; i++) {
      if (!lr.HasIndex(i)) {
        CHECK(fabs(x[i] - c[i]) < 1e-10);
      }
    }

    // Check that A*x can recover the original 'b', within the index set only.
    VectorXd b2 = A*x;
    for (int i = 0; i < N; i++) {
      if (lr.HasIndex(i)) {
        CHECK(fabs(b2[i] - b[i]) < 1e-7);
      }
    }

    // Check MultiplyA for a random input.
    VectorXd q = VectorXd::Random(N,1), r, good_r = A*q;
    lr.MultiplyA(q, &r);
    CHECK(r.size() == N);
    for (int i = 0; i < N; i++) {
      if (!lr.HasIndex(i)) {
        CHECK(fabs(good_r[i] - r[i]) < 1e-10);
      }
    }
  }
  printf("Success\n");
}

TEST_FUNCTION(Murty) {
  for (int iteration = 0; iteration < 1000; iteration++) {
    // Create a random positive definite LCP problem.
    MatrixXd A0 = MatrixXd::Random(N,N);
    MatrixXd A = A0*A0.transpose();
    VectorXd b = VectorXd::Random(N,1);

    {
      // Solve the standard LCP problem. Only pass in the lower triangle of A,
      // to ensure that the upper triangle is not read.
      VectorXd x, w;
      MatrixXd Alower = A.triangularView<Eigen::Lower>();
      CHECK(lcp::SolveLCP_Murty(lcp::Settings(), Alower, b, &x, &w));

      // Check the solution.
      for (int i = 0; i < N; i++) {
        CHECK(x[i] >= 0);
        CHECK(w[i] >= 0);
        CHECK(x[i]*w[i] == 0);
      }
      double error = (A*x - b - w).norm();
      printf("Error = %e\n", error);
      CHECK(error < 1e-6);

      // Solve the same problem as a box LCP problem with lo=0 and hi=infinity.
      // We should get the same solution.
      VectorXd lo(N), hi(N), x2, w2;
      for (int i = 0; i < N; i++) {
        lo[i] = 0;
        hi[i] = __DBL_MAX__;
      }
      MatrixXd A2 = A;
      CHECK(lcp::SolveLCP_BoxMurty(lcp::Settings(), A2, b, lo, hi, &x2, &w2));
      CHECK((x-x2).norm() < 1e-10);
      CHECK((w-w2).norm() < 1e-10);
    }

    {
      VectorXd x, w, lo(N), hi(N);

      // Solve a box LCP problem with random lo and hi vectors. Only pass in
      // the lower triangle of A, to ensure that the upper triangle is not
      // read.
      for (int i = 0; i < N; i++) {
        lo[i] = -RandomDouble() * 10.0;
        hi[i] = +RandomDouble() * 10.0;
        // Occasionally make lo or hi equal to zero.
        int r = RandomInt(100);
        if (r == 0) {
          lo[i] = 0;
        } else if (r == 1) {
          hi[i] = 0;
        }
      }
      MatrixXd Alower = A.triangularView<Eigen::Lower>();
      CHECK(lcp::SolveLCP_BoxMurty(lcp::Settings(), Alower, b, lo, hi, &x, &w));

      // Check the solution.
      //std::cout << "x:\n" << x << "\n";
      //std::cout << "w:\n" << w << "\n";
      for (int i = 0; i < N; i++) {
        CHECK( ((x[i] >= lo[i] && x[i] <= hi[i]) && w[i] == 0) ||
               (x[i] == lo[i] && w[i] >= 0) ||
               (x[i] == hi[i] && w[i] <= 0))
      }
      double error = (A*x - b - w).norm();
      printf("Error = %e\n", error);
      CHECK(error < 1e-6);
    }
  }
  printf("Success\n");
}

TEST_FUNCTION(Cholesky) {
  MatrixXd A0 = MatrixXd::Random(N,N);
  MatrixXd A = A0 * A0.transpose();
  Eigen::LLT<MatrixXd> llt(A);
  lcp::Cholesky(&A);
  MatrixXd L1 = llt.matrixL();
  MatrixXd L2 = A.triangularView<Eigen::Lower>();
  double error = (L1 - L2).norm();
  printf("Error = %e\n", error);
  CHECK(error < 1e-10);
}

TEST_FUNCTION(LSolve) {
  for (int n = 1; n < N; n++) {
    // Form a triangular L and right hand side b
    MatrixXd L0 = MatrixXd::Random(N, N);
    MatrixXd L = L0.triangularView<Eigen::Lower>();
    VectorXd b = MatrixXd::Random(n, 1);
    VectorXd x = b;
    lcp::LSolve(L, n, &x);
    double error = (L.block(0, 0, n, n)*x - b).norm();
    printf("Error = %e\n", error);
    CHECK(error < 1e-10);
  }
}

TEST_FUNCTION(AddCholeskyRow) {
  for (int n = 1; n <= N; n++) {
    // Form a positive definite A, factor, and then add a row.
    MatrixXd A0 = MatrixXd::Random(N,N);
    MatrixXd A = A0 * A0.transpose();
    MatrixXd L = A.triangularView<Eigen::Lower>();
    lcp::Cholesky(&L);
    MatrixXd Loriginal = L;
    L.block(n-1, 0, N-n+1, N).setZero();
    L.block(0, n-1, N, N-n+1).setZero();
    lcp::AddCholeskyRow(A, n, &L);
    double error = (L.block(0, 0, n, n) - Loriginal.block(0, 0, n, n)).norm();
    printf("Error = %e\n", error);
    CHECK(error < 1e-10);
  }
}

TEST_FUNCTION(SwapCholeskyRows) {
  for (int sz = N; sz < N+5; sz++) {
    for (int n = 0; n < N; n++) {
      // Form a positive definite A, factor, and then remove a row/column.
      MatrixXd A0 = MatrixXd::Random(sz, sz);
      MatrixXd A = A0 * A0.transpose();
      Eigen::LLT<MatrixXd> llt(A);
      MatrixXd L = llt.matrixL();
      MatrixXd Lorig = L;
      lcp::SwapCholeskyRows(A, n, N, &L);

      // Compare with explicit refactorization.
      MatrixXd A2 = A;
      A2.col(n).swap(A2.col(N-1));
      A2.row(n).swap(A2.row(N-1));
      Eigen::LLT<MatrixXd> llt2(A2);
      MatrixXd L2 = llt2.matrixL();
      double error = (L.block(0, 0, N-1, N-1) - L2.block(0, 0, N-1, N-1)).norm();
      printf("Removing row/col %d, error = %e\n", n, error);
      CHECK(error < 1e-9);

      // Check the parts of L that should not have been touched.
      if (sz > N) {
        CHECK(L.block(N, 0, sz-N, sz) == Lorig.block(N, 0, sz-N, sz))
      }
    }
  }
}

TEST_FUNCTION(SolveLCP_BoxSchur) {
  const int n = 20;
  MatrixXd A0 = MatrixXd::Random(n, n);
  MatrixXd A = A0 * A0.transpose();
  VectorXd b = VectorXd::Random(n);

  const int M = 1;     // Number of repeats, for more accurate timing

  // Full solve.
  double t1 = Now();
  VectorXd x;
  for (int i = 0; i < M; i++) {
    MatrixXd L = A;
    lcp::Cholesky(&L);
    x = b;
    lcp::LLTSolve(L, n, &x);
  }
  double t2 = Now();
  double error = (A*x - b).norm();
  printf("Full solve,            time = %f, error = %e\n", (t2-t1)/M, error);
  CHECK(error < 1e-6);

  // Unbounded problem, test nub==n (i.e. just do a direct factor and solve).
  VectorXd lo(n), hi(n), x2, w2;
  lo.setConstant(-__DBL_MAX__);
  hi.setConstant(__DBL_MAX__);
  t1 = Now();
  for (int i = 0; i < M; i++) {
    // Only pass the lower triangle of A to the solver, to ensure that the upper
    // triangle is not read.
    MatrixXd Alower = A.triangularView<Eigen::Lower>();
    CHECK(lcp::SolveLCP_BoxSchur(lcp::Settings(),
                                 Alower, b, lo, hi, &x2, &w2, n));
  }
  t2 = Now();
  error = (x2 - x).norm();
  printf("Solve, nub=n,   time = %f, error = %e\n", (t2-t1)/M, error);
  CHECK(error < 1e-6);
  CHECK(w2.size() == x2.size());
  for (int i = 0; i < w2.size(); i++) {
    CHECK(w2[i] == 0);
  }

  // Unbounded problem, test schur complement for nub==n/2.
  VectorXd x3, w3;
  t1 = Now();
  for (int i = 0; i < M; i++) {
    MatrixXd Alower = A.triangularView<Eigen::Lower>();
    CHECK(lcp::SolveLCP_BoxSchur(lcp::Settings(),
                                 Alower, b, lo, hi, &x3, &w3, n/2));
  }
  t2 = Now();
  error = (x3 - x).norm();
  printf("Solve, nub=n/2, time = %f, error = %e\n", (t2-t1)/M, error);
  CHECK(error < 1e-6);
  CHECK(w3.size() == x3.size());
  for (int i = 0; i < w3.size(); i++) {
    CHECK(w3[i] == 0);
  }

  // Solve a full box LCP problem, with nub = n/2.
  for (int i = n/2; i < n; i++) {
    lo[i] = -RandomDouble() * 10.0;
    hi[i] = +RandomDouble() * 10.0;
  }
  VectorXd x4, w4;
  t1 = Now();
  for (int i = 0; i < M; i++) {
    MatrixXd Alower = A.triangularView<Eigen::Lower>();
    CHECK(lcp::SolveLCP_BoxSchur(lcp::Settings(),
                                 Alower, b, lo, hi, &x4, &w4));
  }
  t2 = Now();
  error = (A*x4 - b - w4).norm();
  printf("Solve lcp,  time = %f, error = %e\n", (t2-t1)/M, error);
  CHECK(error < 1e-6);
  for (int i = 0; i < n; i++) {
    CHECK(x4[i] >= lo[i] && x4[i] <= hi[i]);
  }
}

}  // anonymous namespace
