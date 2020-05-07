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

#include "optimizer.h"
#include "thread.h"
#include "testing.h"
#include <stdio.h>

using std::vector;

//***************************************************************************
// Message passing between two threads. One thread calls Send(), the other
// calls Receive(). Only one message is allowed to be in flight at once.

template <class T>
class Pipe {
 public:
  Pipe() : cond_(mutex_), message_(0), in_flight_(false) {
  }

  ~Pipe() {
    delete message_;            // Unreceived messages are deleted here
  }

  // Send a message to the other thread. Only one message is allowed to be in
  // flight at once.
  void Send(T *msg) {
    {
      MutexLock lock(&mutex_);
      CHECK(!in_flight_);
      in_flight_ = true;
      message_ = msg;
    }
    cond_.Signal();             // Signal the receiving thread.
  }

  // Returns when message is available from other thread.
  T *Receive() MUST_USE_RESULT {
    MutexLock lock(&mutex_);
    while (!in_flight_) {
      cond_.Wait();
    }
    T *msg = message_;
    message_ = 0;
    in_flight_ = false;
    return msg;
  }

 private:
  Mutex mutex_;
  ConditionVariable cond_;
  T *message_;                  // Current message in flight, can be 0
  bool in_flight_;              // True if a message is currently in flight
};

//***************************************************************************
// InteractiveOptimizer.

struct ParametersAndFlag {
  vector<double> parameters;
  bool jacobians_needed;
};

struct InteractiveOptimizer::Implementation : public Thread {
  Pipe<ParametersAndFlag> parameters_pipe_;
  Pipe<ErrorsAndJacobians> errors_pipe_;
  InteractiveOptimizer *optimizer_;
  vector<InteractiveOptimizer::Parameter> parameter_info_;
  bool done_;                   // Has Optimize() completed?
  bool aborting_;               // In the process of giving up in Optimize()?

  Implementation() : Thread(JOINABLE) {
    optimizer_ = 0;
    done_ = false;
    aborting_ = false;
  }

  // Main function of the thread.
  void *Entry() {
    // Do the optimization.
    vector<double> optimized_parameters;
    bool success = optimizer_->Optimize(parameter_info_, &optimized_parameters);

    // Final parameters are now available.
    done_ = true;
    if (success) {
      ParametersAndFlag *p = new ParametersAndFlag;
      p->parameters = optimized_parameters;
      p->jacobians_needed = false;
      parameters_pipe_.Send(p);
    } else {
      parameters_pipe_.Send(0);
    }
    return 0;
  }
};

InteractiveOptimizer::InteractiveOptimizer() {
  impl_ = 0;
  jacobians_needed_ = false;
  function_tolerance_ = 1e-6;
  parameter_tolerance_ = 1e-8;
  gradient_tolerance_ = 1e-10;
}

InteractiveOptimizer::~InteractiveOptimizer() {
  if (impl_) {
    // If the optimizer thread is not done we need to shut it down. This might
    // require the cooperation of Optimize(), which is obtained by setting the
    // aborting_ flag.
    vector<double> dummy;
    impl_->aborting_ = true;
    while (!impl_->done_) {
      while (!DoOneIteration(dummy, dummy)) {
      }
    }
    impl_->Wait();
    delete impl_;
  }
}

void InteractiveOptimizer::Initialize(
            const vector<InteractiveOptimizer::Parameter> &start) {
  CHECK(impl_ == 0);                    // Make sure this is only called once
  impl_ = new Implementation;
  impl_->optimizer_ = this;
  impl_->parameter_info_ = start;
  impl_->Run();
  ParametersAndFlag *p = impl_->parameters_pipe_.Receive();
  if (p) {
    parameters_.swap(p->parameters);
    jacobians_needed_ = p->jacobians_needed;
    delete p;
  } else {
    // Failure is indicated by a NULL pointer returned through the pipe.
    parameters_.clear();
    jacobians_needed_ = false;
  }
}

bool InteractiveOptimizer::DoOneIteration(const vector<double> &errors,
                                          const vector<double> &jacobians) {
  // Provide the errors for the previous set of parameters to the thread.
  ErrorsAndJacobians *e = new ErrorsAndJacobians;
  e->errors = errors;
  e->jacobians = jacobians;
  impl_->errors_pipe_.Send(e);

  // Return the parameters to evaluate the objective function at.
  // Failure is indicated by a NULL pointer passed through the pipe.
  ParametersAndFlag *p = impl_->parameters_pipe_.Receive();
  if (p) {
    parameters_.swap(p->parameters);
    jacobians_needed_ = p->jacobians_needed;
    delete p;
  } else {
    parameters_.clear();
    jacobians_needed_ = false;
  }
  return impl_->done_;
}

void InteractiveOptimizer::Evaluate(const vector<double> &parameters,
                            bool jacobians_needed,
                            ErrorsAndJacobians **errors_and_jacobians) const {
  // Send parameters to the main thread.
  ParametersAndFlag *p = new ParametersAndFlag;
  p->parameters = parameters;
  p->jacobians_needed = jacobians_needed;
  impl_->parameters_pipe_.Send(p);

  // Wait for DoOneIteration() to be called with the errors and jacobians we
  // need.
  *errors_and_jacobians = impl_->errors_pipe_.Receive();

  // Sanity checking.
  if (!jacobians_needed && (*errors_and_jacobians) &&
      !(*errors_and_jacobians)->jacobians.empty()) {
    Panic("Internal error: Jacobians computed needlessly");
  }
}

//***************************************************************************
// Interface to the Ceres optimizer. This is only included if
// __TOOLKIT_USE_CERES__ is defined.

#ifdef __TOOLKIT_USE_CERES__

// For debugging the ceres optimizer, compare user supplied jacobians with
// numerically computed jacobians to see if there are (significant)
// differences. Numbers will be printed to stdout and need to be examined by
// eye, since differences are common, especially in meshed FEM models.
static const bool kDebugJacobians = false;

// For debugging the ceres optimizer, see if the solution found is really a
// local minimum along all parameter directions.
static const bool kDebugIsSolutionMinimum = false;

// A fake android logging system. We don't actually run android, we just have
// ceres's miniglog use this function so that it doesn't try to use stderr and
// abort().

#include "ceres-build/android/log.h"
void __android_log_write(int android_log_level,
                         const char *tag, const char *message) {
  switch (android_log_level) {
    case ANDROID_LOG_FATAL:
      Panic("%s: %s", tag, message);
      break;
    case ANDROID_LOG_ERROR:
      Error("%s: %s", tag, message);
      break;
    case ANDROID_LOG_WARN:
      Warning("%s: %s", tag, message);
      break;
    case ANDROID_LOG_INFO:
      Message("%s: %s", tag, message);
      break;
    case ANDROID_LOG_DEBUG:
      return;                           // Too chatty
    case ANDROID_LOG_VERBOSE:
      return;                           // Too chatty
    default:
      Panic("Unknown android_log_level");
  }
}

// Unfortunately when ceres uses 'miniglog' it defines its own version of CHECK
// in its public header files. We have to be careful not to use CHECK past this
// point because of confusion about which error system actually gets invoked.
#undef CHECK
#include "ceres/ceres.h"
#undef CHECK

// A ceres cost function that uses the InteractiveOptimizer to do the work.
class CeresCostFunction : public ceres::CostFunction {
 public:
  CeresInteractiveOptimizer *opt_;

  CeresCostFunction(CeresInteractiveOptimizer *opt) : opt_(opt) {
    mutable_parameter_block_sizes()->resize(1);
    mutable_parameter_block_sizes()->at(0) = opt_->num_parameters_;
    set_num_residuals(opt_->num_errors_);
  }

  // This is the function that is called by ceres.
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const {
    if (opt_->impl_->aborting_) {
      return false;
    }

    // Compute residuals for the given parameters.
    int num_params = parameter_block_sizes()[0];
    vector<double> params(num_params);
    for (int i = 0; i < num_params; i++) {
      params[i] = parameters[0][i];
    }
    InteractiveOptimizer::ErrorsAndJacobians *ej = 0;
    opt_->Evaluate(params, jacobians != 0, &ej);
    if (!opt_->impl_->aborting_) {
      for (int i = 0; i < num_residuals(); i++) {
        residuals[i] = ej->errors[i];
        if (!std::isfinite(residuals[i])) {
          // Residuals that are infinite or NaN will return false to ceres,
          // preventing stepping into this region. Too many such returns will
          // cause an abort and sub optimal optimization.
          return false;
        }
      }

      // Supply or compute jacobian, if requested:
      if (jacobians) {
        bool have_user_jacobians = !ej->jacobians.empty();
        if (have_user_jacobians) {
          // Copy user-supplied jacobian data.
          if (ej->jacobians.size() != num_params * num_residuals()) {
            Panic("jacobians have size %d but %d*%d expected",
                  (int) ej->jacobians.size(), num_params, num_residuals());
          }
          for (int i = 0; i < ej->jacobians.size(); i++) {
            if (!std::isfinite(ej->jacobians[i])) {
              // Residuals that are infinite or NaN will return false to ceres,
              // preventing stepping into this region. Too many such returns
              // will cause an abort and sub optimal optimization.
              return false;
            }
          }
          memcpy(jacobians[0], ej->jacobians.data(),
                 ej->jacobians.size() * sizeof(ej->jacobians[0]));
        }

        if (!have_user_jacobians || kDebugJacobians) {
          // Compute jacobian numerically (using central difference).
          // jacobians[0][j*num_params + i] = d residual[j] / d params[i]
          for (int i = 0; i < num_params; i++) {
            const double step = opt_->impl_->parameter_info_[i].gradient_step;
            if (step <= 0) {
              Panic("Jacobian requested but not supplied, and gradient_step "
                    "is 0 for parameter %d", i);
            }
            params[i] = parameters[0][i] + step;
            delete ej;
            opt_->Evaluate(params, false, &ej);
            if (opt_->impl_->aborting_) {
              break;
            }
            vector<double> eplus(ej->errors);
            params[i] = parameters[0][i] - step;
            delete ej;
            opt_->Evaluate(params, false, &ej);
            if (opt_->impl_->aborting_) {
              break;
            }
            for (int j = 0; j < num_residuals(); j++) {
              double derivative = (eplus[j] - ej->errors[j]) / (2 * step);
              if (have_user_jacobians && kDebugJacobians) {
                // Print debug information for jacobians.
                double J = jacobians[0][j*num_params + i];
                printf("jacobian[%3d]: user=%12f numerical=%12f "
                       "error =%12f fraction =%12f\n", j*num_params + i,
                       J, derivative, J - derivative,
                       fabs(J - derivative) / derivative);
              } else {
                jacobians[0][j*num_params + i] = derivative;
              }
            }
            params[i] = parameters[0][i];
          }
        }

      }
    }
    delete ej;
    return !opt_->impl_->aborting_;
  }

  // Convenience function to evaluate the |residuals|^2 for some parameters.
  // This is used for debugging.
  double ComputeError(const vector<double> parameters) {
    const double *pp = parameters.data();
    vector<double> residuals(opt_->num_errors_);
    if (!Evaluate(&pp, residuals.data(), 0)) {
      Panic("Could not evaluate");
    }
    double error = 0;
    for (int i = 0; i < residuals.size(); i++) {
      error += residuals[i] * residuals[i];
    }
    return error;
  }
};

CeresInteractiveOptimizer::CeresInteractiveOptimizer(int num_errors,
                                                     OptimizerType type) {
  num_parameters_ = 0;
  num_errors_ = num_errors;
  type_ = type;
}

bool CeresInteractiveOptimizer::Optimize(
                          const vector<InteractiveOptimizer::Parameter> &start,
                          vector<double> *optimized_parameters) {
  // Parameters that will be mutated in place by the solver.
  num_parameters_ = start.size();
  optimized_parameters->resize(num_parameters_);
  for (int i = 0; i < num_parameters_; i++) {
    (*optimized_parameters)[i] = start[i].starting_value;
  }

  // Build the problem.
  ceres::Problem problem;
  CeresCostFunction *fn = new CeresCostFunction(this);
  problem.AddResidualBlock(fn, NULL, optimized_parameters->data());
  for (int i = 0; i < start.size(); i++) {
    problem.SetParameterLowerBound(optimized_parameters->data(), i,
                                   start[i].min_value);
    problem.SetParameterUpperBound(optimized_parameters->data(), i,
                                   start[i].max_value);
  }

  // Run the solver.
  ceres::Solver::Options options;
  if (type_ == SUBSPACE_DOGLEG) {
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    // ^ we could also try TRADITIONAL_DOGLEG.
  } else {
    // Default: LEVENBERG_MARQUARDT.
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  }
  options.use_nonmonotonic_steps = true;                // @@@ Is this useful?
  options.function_tolerance = function_tolerance_;
  options.parameter_tolerance = parameter_tolerance_;
  options.gradient_tolerance = gradient_tolerance_;
  options.minimizer_progress_to_stdout = false;
  // For small (a few hundred parameters) or dense problems we use DENSE_QR:
  options.linear_solver_type = ceres::DENSE_QR;
  options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  options.max_num_iterations = 1000000000;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Optionally see if the solution is really a minimum.
  if (kDebugIsSolutionMinimum) {
    vector<double> p(*optimized_parameters);
    double E = fn->ComputeError(p);
    printf("Checking for local minimum. Solution error = %e\n", E);
    for (int i = 0; i < p.size(); i++) {
      double original = p[i];
      printf("  param[%d]+h, delta error =", i);
      for (int j = -3; j <= 0; j++) {
        p[i] = original + pow(10, j);
        printf("  %15e", fn->ComputeError(p) - E);
      }
      printf("\n  param[%d]-h, delta error =", i);
      for (int j = -3; j <= 0; j++) {
        p[i] = original - pow(10, j);
        printf("  %15e", fn->ComputeError(p) - E);
      }
      printf("\n");
      p[i] = original;
    }
  }

  // Return status.
  if (summary.IsSolutionUsable()) {
    Message("%s", summary.BriefReport().c_str());
  } else {
    Error("%s", summary.BriefReport().c_str());
  }
  return summary.IsSolutionUsable();
}

TEST_FUNCTION(CeresInteractiveOptimizer) {
  // Optimize the Rosenbrock bananna function and see how that goes.

  for (int early_termination = 0; early_termination < 2; early_termination++) {
    for (int numerical_jacobian = 0; numerical_jacobian < 2;
         numerical_jacobian++) {

      CeresInteractiveOptimizer *opt =
          new CeresInteractiveOptimizer(2, LEVENBERG_MARQUARDT);
      opt->SetFunctionTolerance(1e-4);
      opt->SetParameterTolerance(1e-4);
      vector<InteractiveOptimizer::Parameter> start(2);
      for (int i = 0; i < start.size(); i++) {
        start[i].starting_value = 0;
        start[i].min_value = -1e9;
        start[i].max_value = 1e9;
        start[i].gradient_step = 1e-6;
      }

      opt->Initialize(start);

      vector<double> errors(2), jacobians;
      int iteration_number = 0;
      do {
        if (opt->Parameters().empty()) {
          printf("Optimization failed\n");
          return;
        }
        printf("EVALUATING ERRORS, iteration = %d, params =", iteration_number);
        for (int i = 0; i < opt->Parameters().size(); i++) {
          printf(" %.6f", opt->Parameters()[i]);
        }
        printf("\n");

        // Compute errors from the parameters.
        errors[0] = 1 - opt->Parameters()[0];
        errors[1] = 10*(opt->Parameters()[1] -
                        opt->Parameters()[0] * opt->Parameters()[0]);
        jacobians.clear();
        if (!numerical_jacobian && opt->JacobianRequested()) {
          jacobians.resize(4);
          jacobians[0] = -1;                                // de0/pp0
          jacobians[1] = 0;                                 // de0/pp1
          jacobians[2] = -20*opt->Parameters()[0];          // de1/pp0
          jacobians[3] = 10;                                // de1/pp1
        }
        iteration_number++;

        // Early termination test.
        if (early_termination && iteration_number == 10) {
          printf("EARLY TERMINATION TEST\n");
          delete opt;
          opt = 0;
          break;
        }
      } while (!opt->DoOneIteration(errors, jacobians));

      if (opt) {
        printf("FINAL PARAMETERS =");
        for (int i = 0; i < opt->Parameters().size(); i++) {
          printf(" %.6f", opt->Parameters()[i]);
        }
        printf("\n");
      }

      // Check the solution.
      if ((!numerical_jacobian && iteration_number > 50) ||
           (numerical_jacobian && iteration_number > 175)) {
        Panic("We took too many iterations");
      }
      if (!early_termination) {
        if (opt && opt->Parameters().size() != 2) {
          Panic("Expecting 2 output parameters");
        }
      }
      if (opt) {
        for (int i = 0; i < opt->Parameters().size(); i++) {
          if (fabs(opt->Parameters()[i] - 1) > 1e-4) {
            Panic("Incorrect solution");
          }
        }
      }
    }
  }
}

#endif  // __TOOLKIT_USE_CERES__
