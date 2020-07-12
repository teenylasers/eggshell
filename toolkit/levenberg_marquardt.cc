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

// We use the Levenberg-Marquardt algorithm to optimize vector function e(p). A
// quadratic approximation of e around p0 is:
//
//   e(p) = e0 + J*(p-p0)             where J is de/dp
//
// To find the p where e(p) = 0:
//
//   e0 + J*delta_p = 0               assume delta_p = J'*q
//   e0 + J*J'*q = 0
//   q = -inv(J*J')*e0
//   delta_p = -J'*inv(J*J')*e0

#include "levenberg_marquardt.h"
#include "testing.h"
#include <stdio.h>

using std::vector;
using namespace Eigen;
using namespace optimize;

// Uncomment the following two lines for verbose debug output.
//   #define VERBOSE(x) std::cout << x;
//   #include <iostream>
#define VERBOSE(x)

LevenbergMarquardt::~LevenbergMarquardt() {
}

bool LevenbergMarquardt::Optimize(const OptimizerOptions &opt,
                                  Eigen::VectorXd *p, OptimizerStatus *status) {
  vector<Trip> Jtrips;                  // Entries in J
  VectorXd e, new_e, delta_p, new_p;
  status->Clear();

  double lambda = opt.initial_lambda;
  for (; status->num_iterations < opt.max_iterations &&
         status->num_evaluations < opt.max_evaluations;
         status->num_iterations++) {
    VERBOSE("Iteration " << status->num_iterations <<
            ", p = " << p->transpose() << std::endl)

    // Compute the constraint matrix J and the error e for the current p.
    ComputeError2(opt, *p, &e, &Jtrips);
    status->E = e.norm();
    status->num_evaluations++;
    VERBOSE("ComputeError: e = " << e.transpose() << ", E=" << status->E <<
            ", Jtrips has " << Jtrips.size() << " entries" << std::endl)

    // If the current error is good enough then we're done.
    if (status->E <= opt.max_error) {
      return true;
    }

    // Compute trial delta_p values for increasing lambdas until we find one
    // that reduces the error.
    while (status->num_evaluations < opt.max_evaluations) {
      // Compute an update to p: delta_p = -J'*inv(J*J' + lambda*I)*e
      VERBOSE("ComputeUpdate: lambda = " << lambda << std::endl)
      while (!ComputeUpdate(p->size(), lambda, Jtrips, e, &delta_p)) {
        // Factorization failed, J*J' is likely near singular. Increasing
        // lambda will hopefully fix that.
        lambda *= 10;
        if (lambda > opt.max_lambda) {
          status->error = "lambda > max_lambda trying to make J*J' nonsingular";
          return false;
        }
        VERBOSE("Factorization failed, trying lambda = " << lambda << std::endl)
      }
      // Compute the error at the updated p.
      new_p = *p - delta_p;
      if (new_p == *p) {
        // lambda is so large that delta_p is too small to affect p. Give up.
        status->error = "lambda too large to make progress";
        return false;
      }
      ComputeError2(opt, new_p, &new_e, 0);
      double new_E = new_e.norm();
      VERBOSE("delta_p = " << delta_p.transpose() << std::endl <<
              "new_p = " << new_p.transpose() << std::endl <<
              "ComputeError: new_e = " << new_e.transpose() << std::endl)
      status->num_evaluations++;
      if (new_E < status->E) {
        // Error at the new p is better, so jump to the new p.
        VERBOSE("new_E = " << new_E << " (better, new_p accepted)" << std::endl)
        p->swap(new_p);
        lambda /= 10;
        break;
      } else {
        // Otherwise try again with a larger lambda.
        VERBOSE("new_E = " << new_E << " (not better)" << std::endl)
        lambda *= 10;
        if (lambda > opt.max_lambda) {
          status->error = "lambda > max_lambda trying to find lower error";
          return false;
        }
      }
    }
  }
  status->error = "exceeded max number of iterations or evaluations";
  return false;
}

bool LevenbergMarquardt::ComputeUpdate(int m, double lambda,
                                       const std::vector<Trip> &Jtrips,
                                       const Eigen::VectorXd &e,
                                       VectorXd *delta_p) {
  const int n = e.size();              // Rows in J
  SparseMatrix<double> J(n, m), A(n, n);
  J.setFromTriplets(Jtrips.begin(), Jtrips.end());
  A = J * J.transpose();                // @@@ Only compute lower triangle of A?
  for (int i = 0; i < n; i++) {         // Add to diagonal of A
    A.coeffRef(i, i) += lambda;         // @@@ Is there a faster way?
  }
  CHECK(A.isCompressed());              // Otherwise factorization makes a copy
  SimplicialLDLT<SparseMatrix<double> > solver;
  solver.analyzePattern(A);             // @@@ Pattern could be reused?
  solver.factorize(A);
  if (solver.info() != Eigen::Success) {
    // Matrix factorization failed.
    return false;
  }
  *delta_p = J.transpose() * solver.solve(e);
  return true;
}

void LevenbergMarquardt::ComputeError2(const OptimizerOptions &opt,
                                       const Eigen::VectorXd &p,
                                       Eigen::VectorXd *e,
                                       std::vector<Trip> *Jtrips) {
  ComputeError(p, e, Jtrips);
  if (!opt.debug_jacobian || !Jtrips) {
    return;
  }
  // Numerically compute the Jacobian from e(p), using center difference.
  MatrixXd J(e->size(), p.size());
  VectorXd delta(p.size()), e1, e2;
  delta.setZero();
  for (int i = 0; i < p.size(); i++) {
    delta[i] = opt.jacobian_h;
    ComputeError(p - delta, &e1, 0);
    ComputeError(p + delta, &e2, 0);
    delta[i] = 0;
    J.col(i) = (e2 - e1) / (2 * opt.jacobian_h);
  }
  // Compare J with Jtrips. Complain about more than 1% relative error.
  for (int k = 0; k < Jtrips->size(); k++) {
    int i = (*Jtrips)[k].row();
    int j = (*Jtrips)[k].col();
    double largest = std::max(fabs(J(i, j)), fabs((*Jtrips)[k].value()));
    double error = J(i, j) - (*Jtrips)[k].value();
    if (fabs(error / largest) > 0.01) {
      printf("J(%d,%d) = %f, Jtrips = %f\n", i, j, J(i, j),
             (*Jtrips)[k].value());
    }
    J(i, j) = 0;
  }
  for (int i = 0; i < J.rows(); i++) {
    for (int j = 0; j < J.cols(); j++) {
      if (J(i, j) != 0) {
        printf("J(%d,%d) = %f, not accounted for in Jtrips\n", i, j, J(i,j));
      }
    }
  }
}

//***************************************************************************
// Testing

static double RandDouble() {
  return random() / double(RAND_MAX);
}

TEST_FUNCTION(LevenbergMarquardt) {
  struct RosenbrockBanana : public LevenbergMarquardt {
    void ComputeError(const VectorXd &p, VectorXd *e, vector<Trip> *Jtrips) {
      e->resize(2);
      (*e)[0] = 1 - p(0);
      (*e)[1] = 10*(p(1) - p(0)*p(0));
      if (Jtrips) {
        Jtrips->resize(3);
        (*Jtrips)[0] = Trip(0, 0, -1);
        (*Jtrips)[1] = Trip(1, 0, -20*p(0));
        (*Jtrips)[2] = Trip(1, 1, 10);
      }
    }
  };

  OptimizerOptions opt;
  opt.max_error = 1e-10;
  opt.max_iterations = 100;
  opt.max_evaluations = 100;
  opt.debug_jacobian = true;

  RosenbrockBanana fn;
  VectorXd p(2);
  p.setZero();
  OptimizerStatus status;
  if (!fn.Optimize(opt, &p, &status)) {
    Panic("Optimize() failed: %s", status.error);
  }
  printf("Solution = (%f,%f), E = %e, found in %d iters, %d evals\n",
         p[0], p[1], status.E, status.num_iterations, status.num_evaluations);
  CHECK(fabs(p[0] - 1) < 1e-9);
  CHECK(fabs(p[1] - 1) < 1e-9);
  CHECK(status.num_iterations < 20);
  CHECK(status.num_evaluations < 60);
}

TEST_FUNCTION(LevenbergMarquardt_PointsInARow) {
  struct PointDistance : public LevenbergMarquardt {
    // p is the 1D location of a bunch of points which should have distance 1
    // from each other. Point 0 should have distance 1 to 0.
    void ComputeError(const VectorXd &p, VectorXd *e, vector<Trip> *Jtrips) {
      e->resize(p.size());
      if (Jtrips) {
        Jtrips->clear();
      }
      for (int i = 0; i < p.size(); i++) {
        if (i > 0) {
          (*e)[i] = p[i] - p[i-1] - 1;
          if (Jtrips) {
            Jtrips->push_back(Trip(i, i, 1));
            Jtrips->push_back(Trip(i, i - 1, -1));
          }
        } else {
          (*e)[0] = p[0] - 1;
          if (Jtrips) {
            Jtrips->push_back(Trip(0, 0, 1));
          }
        }
      }
    }
  };

  OptimizerOptions opt;
  opt.max_error = 1e-10;
  opt.max_iterations = 1000;
  opt.max_evaluations = 1000;
  opt.debug_jacobian = true;

  PointDistance fn;
  VectorXd p(100);
  for (int i = 0; i < p.size(); i++) {
    p[i] = RandDouble() * 100 - 50;
  }
  OptimizerStatus status;
  if (!fn.Optimize(opt, &p, &status)) {
    Panic("Optimize() failed: %s", status.error);
  }
  printf("Solution with E = %e, found in %d iters, %d evals\n",
         status.E, status.num_iterations, status.num_evaluations);
  for (int i = 0; i < p.size(); i++) {
    if (fabs(p[i] - i - 1) > 1e-9) {
      Panic("p[%d] is %f, should be %d", i, p[i], i + 1);
    }
  }
  CHECK(status.num_iterations < 5);
  CHECK(status.num_evaluations < 10);
}
