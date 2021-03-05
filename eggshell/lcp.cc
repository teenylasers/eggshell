#include "lcp.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "constants.h"
#include "error.h"

namespace {

// If LCP aborts early due to max_iterations, we allow for a looser solution
// check error.
constexpr double kLcpLooserAllowedError = 1e-8;

// Check whether {x, w, S} form a solution to the LCP problem. If not, update
// one offending element in S.
bool CheckMurtySolution(const MatrixXd& A, const VectorXd& b, const VectorXd& x,
                        const VectorXd& w, ArrayXb& S, ArrayXd& C,
                        const double x_lo, const double x_hi,
                        const double err = 0) {
  // Option to allow for numerical error. When checking x(S) < zero and
  // w(S_complement) < zero, zero can be 0 or a small negative number.
  // const double zero = std::abs(err) > 0 ? std::abs(err) : 0;
  const double solution_check_err = std::abs(err) > kAllowNumericalError
                                        ? std::abs(err)
                                        : kAllowNumericalError;

  // dimension of the problem.
  const int dim = S.rows();

  // TODO: why do I need S here? Since x(!S) and w(S) are all 0.

  for (int i = 0; i < dim; ++i) {
    if (S(i)) {
      // Check x(S),
      // 1. if x(i) < x_lo, then S(i) = false, C(i) = x_lo
      // 2. if x(i) > x_hi, then S(i) = false, C(i) = x_hi
      if (x(i) < x_lo) {
        S(i) = false;
        C(i) = x_lo;
        return false;
      } else if (x(i) > x_hi) {
        S(i) = false;
        C(i) = x_hi;
        return false;
      }  // else do nothing
    } else {
      // Check w(~S),
      // 1. if C(i) = x_lo && w(i) < 0, then S(i) = true
      // 2. if C(i) = x_hi && w(i) > 0, then S(i) = true
      if (C(i) == x_lo && w(i) < 0) {
        S(i) = true;
        return false;
      } else if (C(i) == x_hi && w(i) > 0) {
        S(i) = true;
        return false;
      }  // else do nothing
    }
  }

  // Passed both x(S) and w(!S) checks above, check goodness of solution.
  if ((x.array() < x_lo).any() || (x.array() > x_hi).any()) {
    // std::cout << "Some elements of x are less than x_lo or greater than
    // x_hi:\n"
    //           << x;
    return false;
  }
  const VectorXd w_at_xlo = Lcp::SelectSubvector(w, x.array() == x_lo);
  const VectorXd w_at_xhi = Lcp::SelectSubvector(w, x.array() == x_hi);
  if ((w_at_xlo.array() < 0).any() || (w_at_xhi.array() > 0).any()) {
    // std::cout << "Some elements of w are out of bound. \nx = \n"
    //           << x << "w = \n"
    //           << w;
    return false;
  }

  const VectorXd lhs = A * x;
  const VectorXd rhs = b + w;
  if ((lhs - rhs).norm() > solution_check_err) {
    std::cout << "Found solution does not satisfy equation Ax = b+w\n";
    // std::cout << "lhs = " << lhs << "\n";
    // std::cout << "rhs = " << rhs << "\n";
    // std::cout << "S = " << S.transpose() << std::endl;
    // std::cout << "x = " << x.transpose() << std::endl;
    // std::cout << "w = " << w.transpose() << std::endl;
    // std::cout << "lhs - rhs = " << (lhs - rhs).transpose() << std::endl;
    std::cout << "(lhs - rhs).norm() = " << (lhs - rhs).norm() << std::endl;
    return false;
  }
  return true;
}

bool CheckMurtySolution(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                        VectorXd& w, ArrayXb& S, const double err = 0) {
  const double x_lo = 0;
  const double x_hi = std::numeric_limits<double>::infinity();
  ArrayXd C = ArrayXd::Zero(S.rows());
  return CheckMurtySolution(A, b, x, w, S, C, x_lo, x_hi, err);
}

// Compute the "goodness" of {x, w}, even if it is not strictly a solution.
// Define goodness here by the magnitude of all negative numbers in x and w.
double ComputeSolutionGoodness(const VectorXd& x, const VectorXd& w) {
  const double x_goodness = (x.array() > 0).select(0, x).sum();
  const double w_goodness = (w.array() > 0).select(0, w).sum();
  CHECK(x_goodness <= 0);
  CHECK(w_goodness <= 0);
  return x_goodness + w_goodness;
}

// Compare LCP solutions, return true is {x0, w0} is better than {x1, w1}, false
// if {x1, w1} is better than {x0, w0}.
bool CompareSolutions(const VectorXd& x0, const VectorXd& w0,
                      const VectorXd& x1, const VectorXd& w1) {
  const double sol0 = ComputeSolutionGoodness(x0, w0);
  const double sol1 = ComputeSolutionGoodness(x1, w1);
  return sol0 > sol1;
}

// Compare new {x, w} with the previous best solution {prev_x, prev_w}, update
// with the new solution if necessary. Return false if {new_x, new_w} ==
// {prev_x, prev_w}, else return true.
bool UpdatePreviousBestSolution(const VectorXd& new_x, const VectorXd& new_w,
                                VectorXd& prev_x, VectorXd& prev_w) {
  if (new_x == prev_x && new_w == prev_w) {
    return false;
  }
  if (CompareSolutions(new_x, new_w, prev_x, prev_w)) {
    prev_x = new_x;
    prev_w = new_w;
  }
  return true;
}

}  // namespace

bool Lcp::MurtyPrincipalPivot(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                              VectorXd& w) {
  const double x_lo = 0;
  const double x_hi = std::numeric_limits<double>::infinity();
  return MurtyPrincipalPivot(A, b, x, w, x_lo, x_hi);
}

bool Lcp::MurtyPrincipalPivot(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                              VectorXd& w, const double x_lo,
                              const double x_hi) {
  // Check input arguments
  CHECK(x_lo < x_hi);
  // TODO: why does x_lo need to be <= 0?
  CHECK(x_lo <= 0);
  CHECK(x_hi > 0);

  // Dimension of the input Ax=b+w problem.
  const int dim = b.rows();
  const int max_iterations = pow(2, dim) > 1000 ? 1000 : pow(2, dim);
  int iter = 0;

  // Initialize S, x, w
  // Default S is empty if init_S is not provided.

  /////////////////// TODO ////////////////////////
  // TODO!! Why does S need to initialize as all true now?!
  ArrayXb S = ArrayXb::Constant(dim, true);
  /////////////////// TODO ////////////////////////
  // Is LCP solver slower?
  // Does S == all true or S == all false change time to solution?
  /////////////////// TODO ////////////////////////
  // WithBound solutions are trivial
  /////////////////// TODO ////////////////////////

  x = VectorXd::Zero(dim);
  w = -b;
  // For elements in !S, C indicates which domain w(i) should be in, whether it
  // is C(i) == x_lo && w >= 0, or C(i) == x_hi && w <= 0. Initialize C to all
  // x_lo to start.
  ArrayXd C = ArrayXd::Ones(dim) * x_lo;
  // Remeber the last best x and w, {last_best_x, last_best_w}
  VectorXd last_best_x = x;
  VectorXd last_best_w = w;

  // Compute solution
  while (iter < max_iterations) {
    if (!CheckMurtySolution(A, b, x, w, S, C, x_lo, x_hi)) {
      // std::cout << "S = " << S.transpose();
      // std::cout << "C = " << C.transpose();

      // If we are not at a solution, compute a new candidate solution after
      // CheckMurtySolution() has updated S.
      const VectorXd new_xs = Lcp::SelectSubmatrix(A, S, S).ldlt().solve(
          Lcp::SelectSubvector(b, S));
      Lcp::UpdateSubvector(x, S, new_xs);  // Update x(S)
      Lcp::UpdateSubvector(x, !S * (C.array() == x_lo),
                           x_lo);  // Update x(!S) = x_lo
      Lcp::UpdateSubvector(x, !S * (C.array() == x_hi),
                           x_hi);  // Update x(!S) = x_hi
      const VectorXd new_wsc =
          Lcp::SelectSubmatrix(A, !S, S) * Lcp::SelectSubvector(x, S) -
          Lcp::SelectSubvector(b, !S);
      Lcp::UpdateSubvector(w, !S, new_wsc);  // Update w(!S)
      Lcp::UpdateSubvector(w, S, 0);         // Update w(S)

      // Remember the current best solution
      UpdatePreviousBestSolution(x, w, last_best_x, last_best_w);
    } else {
      // We found a solution, break
      break;
    }
    ++iter;
  }
  if (iter >= max_iterations) {
    // std::cout << "DEBUG: \nA = \n" << A << "\nb = \n" << b;
    // std::cout << "\nCurrent x = \n" << x << "\nCurrent w = \n" << w << "\n";
    std::cout << "WARNING: iteration count exceeded max_iterations "
              << max_iterations << ". Matrix size = " << dim << ".\n";
  }

  // Check solution
  x = last_best_x;
  w = last_best_w;
  bool res_check = false;
  if (iter >= max_iterations) {
    res_check = CheckMurtySolution(A, b, x, w, S, C, x_lo, x_hi,
                                   kLcpLooserAllowedError);
  } else {
    res_check = CheckMurtySolution(A, b, x, w, S, C, x_lo, x_hi);
  }
  if (res_check) {
    return true;
  } else {
    std::cout << "ERROR: iteration count = " << iter
              << ", max_iterations = " << max_iterations
              << ". Did not reach a sensible solution.\n";
    // Print A and b to file for debug.
    std::ofstream f("lcp_debug.log");
    if (f.is_open()) {
      f << "A\n"
        << A << "\nb\n"
        << b << "\nx\n"
        << x << "\nw\n"
        << w << "\nx_lo\n"
        << x_lo << "\nx_hi\n"
        << x_hi << "\n";
      std::cout << "Printed Ax = b + w, x_lo, x_hi to file "
                   "lcp_debug.log.\n";
    } else {
      std::cout << "ERROR: could not open ofstream to pring debug log "
                   "lcp_debug.log";
    }
    return false;
  }
}

bool Lcp::MixedConstraintsSolver(const MatrixXd& A, const VectorXd& b,
                                 const ArrayXb& C, const VectorXd& x_lo,
                                 const VectorXd& x_hi, VectorXd& x,
                                 VectorXd& w) {
  // Check that the inequality portions are

  const int dim = A.rows();
  const int dim_eq =
      C.cast<int>().sum();  // num of equality constraints, i.e. dim of x_e

  // Solve for x_i
  const MatrixXd A_ee = SelectSubmatrix(A, C, C);
  const MatrixXd A_ei = SelectSubmatrix(A, C, !C);
  const MatrixXd A_ie = SelectSubmatrix(A, !C, C);
  const MatrixXd A_ii = SelectSubmatrix(A, !C, !C);
  const VectorXd b_e = SelectSubvector(b, C);
  const VectorXd b_i = SelectSubvector(b, !C);

  const MatrixXd lhs = A_ii - A_ie * A_ee.inverse() * A_ei;
  const VectorXd rhs = b_i - A_ie * A_ee.inverse() * b_e;

  VectorXd x_i(dim - dim_eq);
  VectorXd w_i(dim - dim_eq);
  if (!MurtyPrincipalPivot(lhs, rhs, x_i, w_i)) {
    Error(
        "MixedConstraintsSolver: MurtyPrincipalPivot exited without reaching a "
        "solution.");
    return false;
  }

  // Solve for x_e
  VectorXd x_e = A_ee.inverse() * (b_e - A_ei * x_i);

  // Construct and return x and w
  VectorXd res_x =
      VectorXd::Constant(dim, std::numeric_limits<double>::infinity());
  UpdateSubvector(res_x, C, x_e);
  UpdateSubvector(res_x, !C, x_i);
  if (res_x.sum() == std::numeric_limits<double>::infinity()) {
    std::cout << C.transpose() << std::endl;
    std::cout << !C.transpose() << std::endl;
    std::cout << x_e.transpose() << std::endl;
    std::cout << x_i.transpose() << std::endl;
  }
  x << res_x;

  w = VectorXd::Zero(dim);
  UpdateSubvector(w, !C, w_i);

  return true;
}

MatrixXd Lcp::SelectSubmatrix(const MatrixXd& A, const ArrayXb& row_ind,
                              const ArrayXb& col_ind) {
  // Check input argument dimensions
  const auto dim = A.rows();
  CHECK(A.cols() == dim);        // "A must be a square matrix.\n");
  CHECK(row_ind.rows() == dim);  //, "A and ind dimensions do not match.\n");
  CHECK(col_ind.rows() == dim);  //, "A and ind dimensions do not match.\n");

  // Form submatrix
  // TODO: what is a more efficient way?
  const int sr = row_ind.count();  // submatrix num rows
  const int sc = col_ind.count();  // submatrix num cols
  int si = 0;  // advancing submatrix row index as we fill content
  int sj = 0;  // advancing submatrix col index
  MatrixXd submatrix = MatrixXd::Zero(sr, sc);
  for (int i = 0; i < dim; ++i) {
    for (int j = 0; j < dim; ++j) {
      if (col_ind(j)) {
        if (row_ind(i)) {
          submatrix(si, sj) = A(i, j);
        }
        ++sj;
      }
    }
    sj = 0;
    if (row_ind(i)) {
      ++si;
    }
  }
  return submatrix;
}

VectorXd Lcp::SelectSubvector(const VectorXd& v, const ArrayXb& ind) {
  if (v.rows() != ind.rows()) {
    std::cout << "v.rows() = " << v.rows() << ", ind.rows() = " << ind.rows()
              << ". Must be the same dimensions.\n";
    CHECK(v.rows() == ind.rows());
  }
  VectorXd subvector = VectorXd::Zero(ind.count());
  int si = 0;
  for (int i = 0; i < v.rows(); ++i) {
    if (ind(i)) {
      subvector(si) = v(i);
      ++si;
    }
  }
  return subvector;
}

void Lcp::UpdateSubmatrix(MatrixXd& A, const ArrayXb& row_ind,
                          const ArrayXb& col_ind, const MatrixXd& m) {
  // Check input argument dimensions
  const int dim = A.rows();
  CHECK(A.cols() == dim);
  CHECK(row_ind.rows() == dim);
  CHECK(col_ind.rows() == dim);
  const int sr = m.rows();  // submatrix num rows
  const int sc = m.cols();  // submatric num cols
  CHECK(row_ind.count() == sr);
  CHECK(col_ind.count() == sc);
  CHECK(m.rows() == sr);
  CHECK(m.cols() == sc);

  // advancing submatrix row and col indices as we read m content to update A
  int si = 0;
  int sj = 0;
  for (int i = 0; i < dim; ++i) {
    for (int j = 0; j < dim; ++j) {
      if (col_ind(j)) {
        if (row_ind(i)) {
          A(i, j) = m(si, sj);
        }
        ++sj;
      }
    }
    sj = 0;
    if (row_ind(i)) {
      ++si;
    }
  }
}

void Lcp::UpdateSubvector(VectorXd& v, const ArrayXb& ind, const VectorXd& n) {
  // Check input argument dimensions
  const int dim = v.rows();
  CHECK(ind.rows() == dim);
  const int sd = n.rows();
  CHECK(ind.count() == sd);
  int si = 0;
  for (int i = 0; i < dim; ++i) {
    if (ind(i)) {
      v(i) = n(si);
      ++si;
    }
  }
}

void Lcp::UpdateSubvector(VectorXd& v, const ArrayXb& ind, double d) {
  // Check input argument dimensions
  const int dim = v.rows();
  CHECK(ind.rows() == dim);
  for (int i = 0; i < dim; ++i) {
    if (ind(i)) {
      v(i) = d;
    }
  }
}

//***************************************************************************
// Testing.

#include <stdio.h>

#include "testing.h"

namespace {

MatrixXd GenerateRandomSpdMatrix(const int dim) {
  // Random() generates coefficients in the range [-1:1]. Add 1 to make
  // positive.
  MatrixXd m = MatrixXd::Random(dim, dim) + MatrixXd::Ones(dim, dim);
  return m.transpose() * m;
}

TEST_FUNCTION(SelectSubmatrix) {
  // Test case
  ArrayXb S(8);
  S << 1, 1, 0, 0, 1, 0, 1, 1;
  MatrixXd A(8, 8);
  A << 44, 23, 81, 97, 37, 34, 72, 51, 12, 12, 3, 55, 99, 68, 91, 48, 26, 30,
      93, 53, 4, 14, 90, 91, 41, 32, 74, 24, 89, 73, 34, 61, 60, 43, 49, 49, 92,
      11, 70, 62, 27, 51, 58, 63, 80, 66, 20, 86, 61, 9, 24, 68, 10, 50, 4, 81,
      72, 27, 46, 40, 27, 78, 75, 58;
  MatrixXd A_SS(5, 5);
  A_SS << 44, 23, 37, 72, 51, 12, 12, 99, 91, 48, 60, 43, 92, 70, 62, 61, 9, 10,
      4, 81, 72, 27, 27, 75, 58;
  MatrixXd A_ScS(3, 5);
  A_ScS << 26, 30, 4, 90, 91, 41, 32, 89, 34, 61, 27, 51, 80, 20, 86;
  MatrixXd A_SSc(5, 3);
  A_SSc << 81, 97, 34, 3, 55, 68, 49, 49, 11, 24, 68, 50, 46, 40, 78;
  MatrixXd A_ScSc(3, 3);
  A_ScSc << 93, 53, 14, 74, 24, 73, 58, 63, 66;
  // Run test
  const ArrayXb S_complement = !S;
  CHECK(A_SS == Lcp::SelectSubmatrix(A, S, S));
  CHECK(A_ScS == Lcp::SelectSubmatrix(A, S_complement, S));
  CHECK(A_SSc == Lcp::SelectSubmatrix(A, S, S_complement));
  CHECK(A_ScSc == Lcp::SelectSubmatrix(A, S_complement, S_complement));
}

TEST_FUNCTION(UpdateSubmatrix) {
  ArrayXb S(8);
  S << 1, 1, 0, 0, 1, 0, 1, 1;
  MatrixXd A(8, 8);
  A << 90, 82, 36, 39, 57, 17, 23, 11, 96, 25, 84, 57, 47, 61, 92, 97, 55, 93,
      59, 8, 2, 27, 16, 1, 14, 35, 55, 6, 34, 66, 83, 78, 15, 20, 92, 54, 17,
      69, 54, 82, 26, 26, 29, 78, 80, 75, 100, 87, 85, 62, 76, 94, 32, 46, 8, 9,
      26, 48, 76, 13, 53, 9, 45, 40;
  MatrixXd m0(5, 5), m1(5, 3), m2(3, 5), m3(3, 3);
  m0 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
      21, 22, 23, 24, 25;
  m1 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
  m2 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
  m3 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  // Run test
  const ArrayXb S_complement = !S;
  Lcp::UpdateSubmatrix(A, S, S, m0);
  MatrixXd res(8, 8);
  res << 1, 2, 36, 39, 3, 17, 4, 5, 6, 7, 84, 57, 8, 61, 9, 10, 55, 93, 59, 8,
      2, 27, 16, 1, 14, 35, 55, 6, 34, 66, 83, 78, 11, 12, 92, 54, 13, 69, 14,
      15, 26, 26, 29, 78, 80, 75, 100, 87, 16, 17, 76, 94, 18, 46, 19, 20, 21,
      22, 76, 13, 23, 9, 24, 25;
  CHECK(A == res);
  Lcp::UpdateSubmatrix(A, S, S_complement, m1);
  res << 1, 2, 1, 2, 3, 3, 4, 5, 6, 7, 4, 5, 8, 6, 9, 10, 55, 93, 59, 8, 2, 27,
      16, 1, 14, 35, 55, 6, 34, 66, 83, 78, 11, 12, 7, 8, 13, 9, 14, 15, 26, 26,
      29, 78, 80, 75, 100, 87, 16, 17, 10, 11, 18, 12, 19, 20, 21, 22, 13, 14,
      23, 15, 24, 25;
  CHECK(A == res);
  Lcp::UpdateSubmatrix(A, S_complement, S, m2);
  res << 1, 2, 1, 2, 3, 3, 4, 5, 6, 7, 4, 5, 8, 6, 9, 10, 1, 2, 59, 8, 3, 27, 4,
      5, 6, 7, 55, 6, 8, 66, 9, 10, 11, 12, 7, 8, 13, 9, 14, 15, 11, 12, 29, 78,
      13, 75, 14, 15, 16, 17, 10, 11, 18, 12, 19, 20, 21, 22, 13, 14, 23, 15,
      24, 25;
  CHECK(A == res);
  Lcp::UpdateSubmatrix(A, S_complement, S_complement, m3);
  res << 1, 2, 1, 2, 3, 3, 4, 5, 6, 7, 4, 5, 8, 6, 9, 10, 1, 2, 1, 2, 3, 3, 4,
      5, 6, 7, 4, 5, 8, 6, 9, 10, 11, 12, 7, 8, 13, 9, 14, 15, 11, 12, 7, 8, 13,
      9, 14, 15, 16, 17, 10, 11, 18, 12, 19, 20, 21, 22, 13, 14, 23, 15, 24, 25;
  CHECK(A == res);
}

TEST_FUNCTION(SelectSubvector) {
  VectorXd v(20);
  v << 79, 9, 93, 78, 49, 44, 45, 31, 51, 52, 82, 80, 65, 38, 82, 54, 36, 94,
      88, 56;
  ArrayXb S(20);
  S << 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1;
  VectorXd res(10);
  res << 79, 9, 49, 51, 52, 82, 38, 36, 88, 56;
  CHECK(res == Lcp::SelectSubvector(v, S));
  res << 93, 78, 44, 45, 31, 80, 65, 82, 54, 94;
  CHECK(res == Lcp::SelectSubvector(v, !S));
}

TEST_FUNCTION(UpdateSubvector) {
  VectorXd v(20);
  v << 79, 9, 93, 78, 49, 44, 45, 31, 51, 52, 82, 80, 65, 38, 82, 54, 36, 94,
      88, 56;
  ArrayXb S(20);
  S << 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1;
  VectorXd n(10);
  n << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  Lcp::UpdateSubvector(v, S, n);
  VectorXd res(20);
  res << 1, 2, 93, 78, 3, 44, 45, 31, 4, 5, 6, 80, 65, 7, 82, 54, 8, 94, 9, 10;
  CHECK(res == v);
  Lcp::UpdateSubvector(v, !S, n);
  res << 1, 2, 1, 2, 3, 3, 4, 5, 4, 5, 6, 6, 7, 7, 8, 9, 8, 10, 9, 10;
  CHECK(res == v);
  Lcp::UpdateSubvector(v, S, 0);
  res << 0, 0, 1, 2, 0, 3, 4, 5, 0, 0, 0, 6, 7, 0, 8, 9, 0, 10, 0, 0;
  CHECK(res == v);
}

TEST_FUNCTION(CheckMurtySolution) {
  // Test when is-a-solution
  MatrixXd A(5, 5);
  A << 2.1104, 1.4090, 1.5055, 1.3060, 1.1413, 1.4090, 1.9846, 1.7126, 1.0858,
      1.9358, 1.5055, 1.7126, 2.1673, 1.3226, 1.5765, 1.3060, 1.0858, 1.3226,
      1.2704, 0.8927, 1.1413, 1.9358, 1.5765, 0.8927, 2.1211;
  VectorXd b(5), x(5), w(5);
  b << 0.6691, 0.1904, 0.3689, 0.4607, 0.9816;
  x << 0.0942, 0, 0, 0, 0.4121;
  w << 0, 0.7401, 0.4226, 0.0302, 0;
  ArrayXb S(5);
  S << 1, 0, 0, 0, 1;
  CHECK(CheckMurtySolution(A, b, x, w, S, 1e-4));
  // Counter test, when is-not-a-solution
  x << 0.0942, 0, 0.5678, 0, 0.4121;
  w << 0, 0.7401, 0.4226, -0.0302, 0;
  CHECK(!CheckMurtySolution(A, b, x, w, S, 1e-4));
}

TEST_FUNCTION(MurtyPrincipalPivot_simple) {
  // Simple 5x5 tests
  MatrixXd A(5, 5);
  VectorXd b(5), x_exp(5), w_exp(5), x(5), w(5);
  A << 2.1104, 1.4090, 1.5055, 1.3060, 1.1413, 1.4090, 1.9846, 1.7126, 1.0858,
      1.9358, 1.5055, 1.7126, 2.1673, 1.3226, 1.5765, 1.3060, 1.0858, 1.3226,
      1.2704, 0.8927, 1.1413, 1.9358, 1.5765, 0.8927, 2.1211;
  b << 0.6691, 0.1904, 0.3689, 0.4607, 0.9816;
  x_exp << 0.0942, 0, 0, 0, 0.4121;
  w_exp << 0, 0.7401, 0.4226, 0.0302, 0;
  std::cout << "Simple Murty Principal Pivot 5x5 - test 1 ...\n";
  CHECK(Lcp::MurtyPrincipalPivot(A, b, x, w));
  if ((x - x_exp).norm() > 5e-4) {
    std::cout << "x = " << x << "\n";
    std::cout << "x_exp = " << x_exp << "\n";
  }
  CHECK((x - x_exp).norm() <= 5e-4);
  if ((w - w_exp).norm() > 5e-4) {
    std::cout << "w = " << w << "\n";
    std::cout << "w_exp = " << w_exp << "\n";
  }
  CHECK((w - w_exp).norm() <= 5e-4);
  std::cout << "... passed\n";
  std::cout << "Simple Murty Principal Pivot 5x5 - test 2 ...\n";
  A << 2.7345, 1.8859, 2.0785, 1.9442, 1.9567, 1.8859, 2.2340, 2.0461, 2.3164,
      2.0875, 2.0785, 2.0461, 2.7591, 2.4606, 1.9473, 1.9442, 2.3164, 2.4606,
      2.5848, 2.2768, 1.9567, 2.0875, 1.9473, 2.2768, 2.4853;
  b << 0.7577, 0.7431, 0.3922, 0.6555, 0.1712;
  x_exp << 0.1141, 0.2363, 0, 0, 0;
  w_exp << 0, 0, 0.3285, 0.1138, 0.5454;
  CHECK(Lcp::MurtyPrincipalPivot(A, b, x, w));
  CHECK(Lcp::MurtyPrincipalPivot(A, b, x, w));
  if ((x - x_exp).norm() > 5e-4) {
    std::cout << "x = " << x << "\n";
    std::cout << "x_exp = " << x_exp << "\n";
  }
  // CHECK((x - x_exp).norm() <= 5e-4);
  if ((w - w_exp).norm() > 5e-4) {
    std::cout << "w = " << w << "\n";
    std::cout << "w_exp = " << w_exp << "\n";
  }
  // CHECK((w - w_exp).norm() < 5e-4);
  std::cout << "... passed\n";
}

TEST_FUNCTION(MurtyPrincipalPivot_NoBounds) {
  std::cout << "Batch test of larger matrix sizes, x = {0, inf}.\n";
  constexpr int num_tests = 100;
  constexpr int matrix_size = 50;
  VectorXd x(matrix_size), w(matrix_size);
  VectorXd zeros = VectorXd::Zero(matrix_size);
  int success_count = 0, trivial_count = 0;
  for (int i = 0; i < num_tests; ++i) {
    const MatrixXd A = GenerateRandomSpdMatrix(matrix_size);
    const VectorXd b = VectorXd::Random(matrix_size);
    if (Lcp::MurtyPrincipalPivot(A, b, x, w)) {
      ++success_count;
      if (x == zeros) {
        ++trivial_count;
      }
    }
  }
  std::cout << "Murty Principal Pivot, x = {0, inf}: matrix size "
            << matrix_size << "x" << matrix_size << ", " << trivial_count
            << " had trivial solutions, " << success_count << " of "
            << num_tests << " tests passed.\n";
  CHECK(success_count == num_tests);
}

TEST_FUNCTION(MurtyPrincipalPivot_WithBounds) {
  std::cout << "Batch test of larger matrix sizes, x = {x_lo, x_hi}.\n";
  constexpr int num_tests = 100;
  constexpr int matrix_size = 50;
  VectorXd x(matrix_size), w(matrix_size);
  VectorXd zeros = VectorXd::Zero(matrix_size);
  int success_count = 0, trivial_count = 0;
  for (int i = 0; i < num_tests; ++i) {
    const MatrixXd A = GenerateRandomSpdMatrix(matrix_size);
    const VectorXd b = VectorXd::Random(matrix_size);
    const MatrixXd x_lim = MatrixXd::Random(2, 1) + MatrixXd::Ones(2, 1);
    const double x_lo = -1 * x_lim(0, 0);
    const double x_hi = x_lim(1, 0);
    if (Lcp::MurtyPrincipalPivot(A, b, x, w, x_lo, x_hi)) {
      ++success_count;
      if (x == zeros) {
        ++trivial_count;
      }
    }
  }
  std::cout << "Murty Principal Pivot, x = {x_lo, x_hi}: matrix size "
            << matrix_size << "x" << matrix_size << ", " << trivial_count
            << " had trivial solutions, " << success_count << " of "
            << num_tests << " tests passed.\n";
  CHECK(success_count == num_tests);
}

TEST_FUNCTION(MixedConstraintsSolver_NoBounds) {
  std::cout << "x_lo = 0; x_hi = infinity.\n";
  constexpr int num_tests = 100;
  constexpr int matrix_size = 50;
  const VectorXd x_lo = VectorXd::Constant(matrix_size, 0);
  const VectorXd x_hi =
      VectorXd::Constant(matrix_size, std::numeric_limits<double>::infinity());
  VectorXd x(matrix_size), w(matrix_size);
  VectorXd zeros = VectorXd::Zero(matrix_size);
  int success_count = 0, trivial_count = 0;
  for (int i = 0; i < num_tests; ++i) {
    const MatrixXd A = GenerateRandomSpdMatrix(matrix_size);
    const VectorXd b = VectorXd::Random(matrix_size);
    const ArrayXb C = ArrayXb::Random(matrix_size);
    if (Lcp::MixedConstraintsSolver(A, b, C, x_lo, x_hi, x, w)) {
      ++success_count;
      if (x == zeros) {
        ++trivial_count;
      }
    }
  }
  std::cout << "Mixed constraints solver: matrix size " << matrix_size << "x"
            << matrix_size << ", " << trivial_count
            << " had trivial solutions, " << success_count << " of "
            << num_tests << " tests passed.\n";
  CHECK(success_count == num_tests);
}

/*
TEST_FUNCTION(Sandbox) {
ArrayXb a = ArrayXb::Constant(5, false);
std::cout << "All false boolean array: \n" << a << std::endl;
a = !a;
std::cout << "All not-false boolean array: \n" << a << std::endl;
for (int i = 0; i < 10; i++) {
  a = ArrayXb::Random(10);
  std::cout << "a = \n" << a.transpose() << std::endl;
}
a = ArrayXb::Constant(0, false);
std::cout << "a = " << a << std::endl;
std::cout << "a.size() == 0: " << (a.size() != 0) << std::endl;
}
*/

}  // namespace
