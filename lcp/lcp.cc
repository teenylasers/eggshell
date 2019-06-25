#include "lcp.h"

#include <cmath>
#include <iostream>
#include <string>

#include "constants.h"
#include "error.h"

namespace {

// TODO: should be a widely used function. Either figure out ArrayXb in Eigen or
// extend.
ArrayXi LogicalNot(const ArrayXi& a) { return (a - 1).abs(); }

// Check whether {x, w, S} form a solution to the LCP problem. If not, update
// one offending element in S.
bool CheckMurtySolution(const MatrixXd& A, const VectorXd& b, const VectorXd& x,
                        const VectorXd& w, ArrayXi& S,
                        const double err = kAllowNumericalError) {
  VectorXd::Index offending_index;

  // Check x(S) are all >0.
  // const VectorXd x_s Lcp::SelectSubvector(x, S);
  const ArrayXd x_s = x.array() * S.cast<double>();
  if (x_s.minCoeff(&offending_index) < 0) {
    if (S(offending_index) == 1) {
      S(offending_index) = 0;
      return false;
    }
  }

  // Check w(~S) are all >0.
  const ArrayXd w_sc = w.array() * LogicalNot(S).cast<double>();
  if (w_sc.minCoeff(&offending_index) < 0) {
    if (S(offending_index) == 0) {
      S(offending_index) = 1;
      return false;
    }
  }

  const VectorXd lhs = A * x;
  const VectorXd rhs = b + w;
  if ((lhs - rhs).norm() > err) {
    std::cout << "Found solution does not satisfy equation Ax = b+w\n";
    std::cout << "lhs = " << lhs << "\n";
    std::cout << "rhs = " << rhs << "\n";
    std::cout << "lhs - rhs = " << lhs - rhs << "\n";
    std::cout << (lhs - rhs).norm();
    CHECK(false);
  }
  return true;
}

}  // namespace

bool Lcp::MurtyPrinciplePivot(const MatrixXd& A, const VectorXd& b, VectorXd& x,
                              VectorXd& w) {
  // TODO: check input arguments

  // Dimension of the input Ax=b+w problem.
  const int dim = b.rows();
  const int max_iterations = pow(2, dim) > 1000 ? 1000 : pow(2, dim);
  int iter = 0;

  // TODO: start from a different init S
  // TODO: make S a boolean array?
  // Initialize starting S, x, and w.
  ArrayXi S = ArrayXi::Zero(dim);  // Start with S is empty.
  x = VectorXd::Zero(dim);
  w = -b;

  while (iter < max_iterations) {
    // If we are not at a solution, compute a new candidate solution after
    // CheckMurtySolution() has updated S.
    if (!CheckMurtySolution(A, b, x, w, S)) {
      const VectorXd new_xs = Lcp::SelectSubmatrix(A, S, S).ldlt().solve(
          Lcp::SelectSubvector(b, S));
      Lcp::UpdateSubvector(x, S, new_xs);         // Update x(S)
      Lcp::UpdateSubvector(x, LogicalNot(S), 0);  // Update x(~S)
      const VectorXd new_wsc = Lcp::SelectSubmatrix(A, LogicalNot(S), S) *
                                   Lcp::SelectSubvector(x, S) -
                               Lcp::SelectSubvector(b, LogicalNot(S));
      Lcp::UpdateSubvector(w, LogicalNot(S), new_wsc);
      Lcp::UpdateSubvector(w, S, 0);
    } else {
      break;
    }
    ++iter;
  }

  if (iter >= max_iterations) {
    //printf("ERROR: iteration count exceeded max_iterations.\n");
    std::cout << "ERROR: iteration count exceeded max_iterations "
              << max_iterations << ".\n";
    return false;
  } else if (!CheckMurtySolution(A, b, x, w, S)) {
    std::cout << "ERROR: check solution returned false, error in algorithm.\n";
    return false;
  } else {
    // std::cout << "\nLCP Murty ... A = \n" << A << "\n";
    // std::cout << "\nLCP Murty ... b = \n " << b << "\n";
    // std::cout << "\nLCP Murty ... x = \n" << x << "\n";
    // std::cout << "\nLCP Murty ... w = \n " << w << "\n";
    return true;
  }
}

MatrixXd Lcp::SelectSubmatrix(const MatrixXd& A, const ArrayXi& row_ind,
                              const ArrayXi& col_ind) {
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

VectorXd Lcp::SelectSubvector(const VectorXd& v, const ArrayXi& ind) {
  CHECK(v.rows() ==
        ind.rows());  //, "v and ind must be the same dimensions.\n");
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

void Lcp::UpdateSubmatrix(MatrixXd& A, const ArrayXi& row_ind,
                          const ArrayXi& col_ind, const MatrixXd& m) {
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

void Lcp::UpdateSubvector(VectorXd& v, const ArrayXi& ind, const VectorXd& n) {
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

void Lcp::UpdateSubvector(VectorXd& v, const ArrayXi& ind, double d) {
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
  ArrayXi S(8);
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
  const ArrayXi S_complement = LogicalNot(S);
  CHECK(A_SS == Lcp::SelectSubmatrix(A, S, S));
  CHECK(A_ScS == Lcp::SelectSubmatrix(A, S_complement, S));
  CHECK(A_SSc == Lcp::SelectSubmatrix(A, S, S_complement));
  CHECK(A_ScSc == Lcp::SelectSubmatrix(A, S_complement, S_complement));
}

TEST_FUNCTION(UpdateSubmatrix) {
  ArrayXi S(8);
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
  const ArrayXi S_complement = LogicalNot(S);
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
  ArrayXi S(20);
  S << 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1;
  VectorXd res(10);
  res << 79, 9, 49, 51, 52, 82, 38, 36, 88, 56;
  CHECK(res == Lcp::SelectSubvector(v, S));
  res << 93, 78, 44, 45, 31, 80, 65, 82, 54, 94;
  CHECK(res == Lcp::SelectSubvector(v, LogicalNot(S)));
}

TEST_FUNCTION(UpdateSubvector) {
  VectorXd v(20);
  v << 79, 9, 93, 78, 49, 44, 45, 31, 51, 52, 82, 80, 65, 38, 82, 54, 36, 94,
      88, 56;
  ArrayXi S(20);
  S << 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1;
  VectorXd n(10);
  n << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  Lcp::UpdateSubvector(v, S, n);
  VectorXd res(20);
  res << 1, 2, 93, 78, 3, 44, 45, 31, 4, 5, 6, 80, 65, 7, 82, 54, 8, 94, 9, 10;
  CHECK(res == v);
  Lcp::UpdateSubvector(v, LogicalNot(S), n);
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
  ArrayXi S(5);
  S << 1, 0, 0, 0, 1;
  CHECK(CheckMurtySolution(A, b, x, w, S, 1e-4));
  // Counter test, when is-not-a-solution
  x << 0.0942, 0, 0.5678, 0, 0.4121;
  w << 0, 0.7401, 0.4226, -0.0302, 0;
  CHECK(!CheckMurtySolution(A, b, x, w, S, 1e-4));
}

TEST_FUNCTION(MurtyPrinciplePivot_simple) {
  // Simple 5x5 tests
  MatrixXd A(5, 5);
  VectorXd b(5), x_exp(5), w_exp(5), x(5), w(5);
  A << 2.1104, 1.4090, 1.5055, 1.3060, 1.1413, 1.4090, 1.9846, 1.7126, 1.0858,
      1.9358, 1.5055, 1.7126, 2.1673, 1.3226, 1.5765, 1.3060, 1.0858, 1.3226,
      1.2704, 0.8927, 1.1413, 1.9358, 1.5765, 0.8927, 2.1211;
  b << 0.6691, 0.1904, 0.3689, 0.4607, 0.9816;
  x_exp << 0.0942, 0, 0, 0, 0.4121;
  w_exp << 0, 0.7401, 0.4226, 0.0302, 0;
  std::cout << "Simple Murty Principle Pivot 5x5 - test 1 ...\n";
  CHECK(Lcp::MurtyPrinciplePivot(A, b, x, w));
  if ((x - x_exp).norm() <= 5e-4) {
    std::cout << "x = " << x << "\n";
    std::cout << "x_exp = " << x_exp << "\n";
    CHECK((x - x_exp).norm() <= 5e-4);
  }
  if ((w - w_exp).norm() < 5e-4) {
    std::cout << "w = " << w << "\n";
    std::cout << "w_exp = " << w_exp << "\n";
    CHECK((w - w_exp).norm() < 5e-4);
  }
  std::cout << "... passed\n";
  std::cout << "Simple Murty Principle Pivot 5x5 - test 2 ...\n";
  A << 2.7345, 1.8859, 2.0785, 1.9442, 1.9567, 1.8859, 2.2340, 2.0461, 2.3164,
      2.0875, 2.0785, 2.0461, 2.7591, 2.4606, 1.9473, 1.9442, 2.3164, 2.4606,
      2.5848, 2.2768, 1.9567, 2.0875, 1.9473, 2.2768, 2.4853;
  b << 0.7577, 0.7431, 0.3922, 0.6555, 0.1712;
  x << 0.1141, 0.2363, 0, 0, 0;
  w << 0, 0, 0.3285, 0.1138, 0.5454;
  CHECK(Lcp::MurtyPrinciplePivot(A, b, x, w));
  CHECK(Lcp::MurtyPrinciplePivot(A, b, x, w));
  if ((x - x_exp).norm() <= 5e-4) {
    std::cout << "x = " << x << "\n";
    std::cout << "x_exp = " << x_exp << "\n";
    CHECK((x - x_exp).norm() <= 5e-4);
  }
  if ((w - w_exp).norm() < 5e-4) {
    std::cout << "w = " << w << "\n";
    std::cout << "w_exp = " << w_exp << "\n";
    CHECK((w - w_exp).norm() < 5e-4);
  }
  std::cout << "... passed\n";
}

TEST_FUNCTION(MurtyPrinciplePivot_batch) {
  std::cout << "Batch test of larger matrix sizes.\n";
  constexpr int num_tests = 100;
  constexpr int matrix_size = 50;
  VectorXd x(matrix_size), w(matrix_size);
  VectorXd zeros = VectorXd::Zero(matrix_size);
  int success_count = 0, trivial_count = 0;
  for (int i = 0; i < num_tests; ++i) {
    const MatrixXd A = GenerateRandomSpdMatrix(matrix_size);
    const VectorXd b = VectorXd::Random(matrix_size);
    if (Lcp::MurtyPrinciplePivot(A, b, x, w)) {
      ++success_count;
      if (x == zeros) {
        ++trivial_count;
      }
    }
  }
  std::cout << "Murty Principle Pivot: matrix size " << matrix_size << "x"
            << matrix_size << ", " << trivial_count
            << " had trivial solutions, " << success_count << " of "
            << num_tests << " tests passed.\n";
  CHECK(success_count == num_tests);
}

}  // namespace
