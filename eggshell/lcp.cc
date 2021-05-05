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
                        const VectorXd& x_lo, const VectorXd& x_hi,
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
      // 1. if x(i) < x_lo(i), then S(i) = false, C(i) = x_lo(i)
      // 2. if x(i) > x_hi(i), then S(i) = false, C(i) = x_hi(i)
      if (x(i) < x_lo(i)) {
        S(i) = false;
        C(i) = x_lo(i);
        return false;
      } else if (x(i) > x_hi(i)) {
        S(i) = false;
        C(i) = x_hi(i);
        return false;
      }  // else do nothing
    } else {
      // Check w(~S),
      // 1. if C(i) = x_lo && w(i) < 0, then S(i) = true
      // 2. if C(i) = x_hi && w(i) > 0, then S(i) = true
      if (C(i) == x_lo(i) && w(i) < 0) {
        S(i) = true;
        return false;
      } else if (C(i) == x_hi(i) && w(i) > 0) {
        S(i) = true;
        return false;
      }  // else do nothing
    }
  }

  // Passed both x(S) and w(!S) checks above, check goodness of solution.
  if ((x.array() < x_lo.array()).any() || (x.array() > x_hi.array()).any()) {
    // std::cout << "Some elements of x are less than x_lo or greater than
    // x_hi:\n"
    //           << x;
    return false;
  }
  const VectorXd w_at_xlo = SelectSubvector(w, x.array() == x_lo.array());
  const VectorXd w_at_xhi = SelectSubvector(w, x.array() == x_hi.array());
  if ((w_at_xlo.array() < 0).any() || (w_at_xhi.array() > 0).any()) {
    // std::cout << "Some elements of w are out of bound. \nx = \n"
    //           << x << "w = \n"
    //           << w;
    return false;
  }

  const VectorXd lhs = A * x;
  const VectorXd rhs = b + w;
  if ((lhs - rhs).norm() > solution_check_err) {
    // std::cout << "Found solution does not satisfy equation Ax = b+w\n";
    // // std::cout << "lhs = " << lhs << "\n";
    // // std::cout << "rhs = " << rhs << "\n";
    // // std::cout << "S = " << S.transpose() << std::endl;
    // // std::cout << "x = " << x.transpose() << std::endl;
    // // std::cout << "w = " << w.transpose() << std::endl;
    // // std::cout << "lhs - rhs = " << (lhs - rhs).transpose() << std::endl;
    // std::cout << "(lhs - rhs).norm() = " << (lhs - rhs).norm() << std::endl;
    return false;
  }
  return true;
}

bool CheckMurtySolution(const MatrixXd& A, const VectorXd& b, const VectorXd& x,
                        const VectorXd& w, ArrayXb& S, const double err = 0) {
  const VectorXd x_lo = VectorXd::Constant(A.rows(), 0);
  const VectorXd x_hi =
      VectorXd::Constant(A.rows(), std::numeric_limits<double>::infinity());
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
  const VectorXd x_lo = VectorXd::Constant(A.rows(), 0);
  const VectorXd x_hi =
      VectorXd::Constant(A.rows(), std::numeric_limits<double>::infinity());
  return MurtyPrincipalPivot(A, b, x, w, x_lo, x_hi);
}

bool Lcp::MurtyPrincipalPivot(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                              VectorXd& w, const double x_lo,
                              const double x_hi) {
  const VectorXd x_lo_vector = VectorXd::Constant(A.rows(), x_lo);
  const VectorXd x_hi_vector = VectorXd::Constant(A.rows(), x_hi);
  return MurtyPrincipalPivot(A, b, x, w, x_lo_vector, x_hi_vector);
}

bool Lcp::MurtyPrincipalPivot(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                              VectorXd& w, const VectorXd& x_lo,
                              const VectorXd& x_hi) {
  // Check input arguments
  CHECK((x_lo.array() < x_hi.array()).all());
  // TODO: why does x_lo need to be <= 0?
  CHECK((x_lo.array() <= 0).all());
  CHECK((x_hi.array() > 0).all());

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
  ArrayXd C = ArrayXd::Ones(dim) * x_lo.array();
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
      const VectorXd new_xs =
          SelectSubmatrix(A, S, S).ldlt().solve(SelectSubvector(b, S));

      // Update x(S)
      UpdateSubvector(x, S, new_xs);

      // Update x(!S) = x_lo
      const VectorXd select_x_lo =
          SelectSubvector(x_lo, !S * (C.array() == x_lo.array()));
      UpdateSubvector(x, !S * (C.array() == x_lo.array()), select_x_lo);

      // Update x(!S) = x_hi
      const VectorXd select_x_hi =
          SelectSubvector(x_hi, !S * (C.array() == x_hi.array()));
      UpdateSubvector(x, !S * (C.array() == x_hi.array()), select_x_hi);

      // Construct new w
      const VectorXd new_wsc =
          SelectSubmatrix(A, !S, S) * SelectSubvector(x, S) -
          SelectSubvector(b, !S);
      UpdateSubvector(w, !S, new_wsc);  // Update w(!S)
      UpdateSubvector(w, S, 0);         // Update w(S)

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
  const int dim_eq = C.count();  // num of equality constraints, i.e. dim of x_e

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
    std::cout << "MixedConstraintsSolver\n";
    std::cout << "A dimensions = " << A.rows() << " x " << A.cols()
              << ", condition number = " << GetConditionNumber(A) << std::endl;
    std::cout << "A_ii (condition number " << GetConditionNumber(A_ii)
              << ") = \n"
              << A_ii << std::endl;
    std::cout << "A_ie * A_ee.inverse() * A_ei (condition number "
              << GetConditionNumber(A_ie * A_ee.inverse() * A_ei) << ") = \n"
              << A_ie * A_ee.inverse() * A_ei << std::endl;
    std::cout << "lhs (condition number " << GetConditionNumber(lhs) << ") =\n"
              << lhs << std::endl;
    return false;
  }

  // Solve for x_e
  VectorXd x_e = A_ee.ldlt().solve(b_e - A_ei * x_i);

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

//***************************************************************************
// Testing.

#include <stdio.h>

#include "testing.h"
#include "utils.h"

namespace {

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
    const MatrixXd A = GenerateSPDMatrix(matrix_size);
    const VectorXd b = VectorXd::Random(matrix_size);
    if (Lcp::MurtyPrincipalPivot(A, b, x, w)) {
      if ((A * x - b - w).norm() < kAllowNumericalError) {
        ++success_count;
        if (x == zeros) {
          ++trivial_count;
        }
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
    const MatrixXd A = GenerateSPDMatrix(matrix_size);
    const VectorXd b = VectorXd::Random(matrix_size);
    const MatrixXd x_lim = MatrixXd::Random(2, 1) + MatrixXd::Ones(2, 1);
    const double x_lo = -1 * x_lim(0, 0);
    const double x_hi = x_lim(1, 0);
    if (Lcp::MurtyPrincipalPivot(A, b, x, w, x_lo, x_hi)) {
      if ((A * x - b - w).norm() < kAllowNumericalError) {
        ++success_count;
        if (x == zeros) {
          ++trivial_count;
        }
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
  const VectorXd x_lo = VectorXd::Zero(matrix_size);
  const VectorXd x_hi =
      VectorXd::Constant(matrix_size, std::numeric_limits<double>::infinity());
  VectorXd x(matrix_size), w(matrix_size);
  VectorXd zeros = VectorXd::Zero(matrix_size);
  int success_count = 0, trivial_count = 0;
  for (int i = 0; i < num_tests; ++i) {
    const MatrixXd A = GenerateSPDMatrix(matrix_size);
    const VectorXd b = VectorXd::Random(matrix_size);
    const ArrayXb C = ArrayXb::Random(matrix_size);
    if (Lcp::MixedConstraintsSolver(A, b, C, x_lo, x_hi, x, w)) {
      if ((A * x - b - w).norm() < kAllowNumericalError &&
          (C || x.array() >= x_lo.array()).all() &&
          (C || x.array() <= x_hi.array()).all()) {
        ++success_count;
        if (x == zeros) {
          ++trivial_count;
        }
      }
    }
  }
  std::cout << "Mixed constraints solver: matrix size " << matrix_size << "x"
            << matrix_size << ", " << trivial_count
            << " had trivial solutions, " << success_count << " of "
            << num_tests << " tests passed.\n";
  CHECK(success_count == num_tests);
}

TEST_FUNCTION(MixedConstraintsSolver_WithBounds) {
  std::cout << "x_lo = -10; x_hi = 10.\n";
  constexpr int num_tests = 100;
  constexpr int matrix_size = 50;
  const VectorXd x_lo = VectorXd::Constant(matrix_size, -10);
  const VectorXd x_hi = VectorXd::Constant(matrix_size, 10);
  VectorXd x(matrix_size), w(matrix_size);
  VectorXd zeros = VectorXd::Zero(matrix_size);
  int success_count = 0, trivial_count = 0;
  for (int i = 0; i < num_tests; ++i) {
    const MatrixXd A = GenerateSPDMatrix(matrix_size);
    const VectorXd b = VectorXd::Random(matrix_size);
    const ArrayXb C = ArrayXb::Random(matrix_size);
    if (Lcp::MixedConstraintsSolver(A, b, C, x_lo, x_hi, x, w)) {
      if ((A * x - b - w).norm() < kAllowNumericalError &&
          (x.array() >= x_lo.array()).all() &&
          (x.array() <= x_hi.array()).all()) {
        ++success_count;
        if (x == zeros) {
          ++trivial_count;
        }
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
