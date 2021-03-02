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

#ifndef __LEVENBERG_MARQUARDT_H_
#define __LEVENBERG_MARQUARDT_H_

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "error.h"
#include <vector>

namespace optimize {

struct OptimizerOptions {
  double max_error;             // Stop when |e| < max_error
  int max_iterations;           // Max optimizer outer-loop iterations
  int max_evaluations;          // Max objective function evaluations
  double initial_lambda;        // Initial lambda value for LM
  double max_lambda;            // Largest useful lambda value for LM
  bool debug_jacobian;          // Numerical Jacobian checks
  double jacobian_h;            // Step size for numerical Jacobian checks

  OptimizerOptions() {
    max_error = 1e-6;
    max_iterations = 1000;
    max_evaluations = 10000;
    initial_lambda = 1e-6;
    max_lambda = 1e20;
    debug_jacobian = false;
    jacobian_h = 1e-6;
  }
};

struct OptimizerStatus {
  const char *error;            // Human readable error message, 0 if none
  int num_iterations;           // Total number of iterations needed
  int num_evaluations;          // Total number of function evaluations needed
  double E;                     // Error for returned solution

  OptimizerStatus() { Clear(); }
  void Clear() {
    error = 0;
    num_iterations = num_evaluations = 0;
    E = 0;
  }
};

class LevenbergMarquardt {
 public:
  virtual ~LevenbergMarquardt();

  // Given starting point p, update it to a value that minimizes e(p). Return
  // true on success or false if optimization failed for some reason. The
  // status is updated on return, and on error status.error will contain an
  // error message.
  bool Optimize(const OptimizerOptions &opt, Eigen::VectorXd *p,
                OptimizerStatus *status);

  // Given the point p, compute e(p) and entries for constraint matrix J=de/dp.
  // Both e and Jtrip may not have correct sizes on entry. Jtrips can be 0 in
  // which case it is not filled in.
  typedef Eigen::Triplet<double> Trip;
  virtual void ComputeError(const Eigen::VectorXd &p, Eigen::VectorXd *e,
                            std::vector<Trip> *Jtrips) = 0;


 private:
  // Compute: delta_p = J'*inv(J*J' + lambda*I)*e0. The number of columns in J
  // is m. Return false if (J*J'+lambda*I) is not positive definite.
  bool ComputeUpdate(int m, double lambda, const std::vector<Trip> &Jtrips,
                     const Eigen::VectorXd &e0, Eigen::VectorXd *delta_p)
                     MUST_USE_RESULT;

  // Call ComputeError(), but when in Jacobian debugging mode also numerically
  // compute the Jacobian from e(p) to make sure that Jtrips is correct.
  void ComputeError2(const OptimizerOptions &opt,
                     const Eigen::VectorXd &p, Eigen::VectorXd *e,
                     std::vector<Trip> *Jtrips);
};

}  // namespace optimize

#endif
