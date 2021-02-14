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

#ifndef __OPTIMIZER_H__
#define __OPTIMIZER_H__

#include <vector>
#include "error.h"
#include "Eigen/Dense"

// An abstract interface to an optimizer.

class AbstractOptimizer {
 public:
  virtual ~AbstractOptimizer() = 0;

  // Initialize the optimizer with a starting set of parameters. Parameters()
  // will now return the first set of parameters to evaluate the objective
  // function at (which are not necessarily the same as the starting
  // parameters) and JacobianRequested() will indicate if a corresponding
  // jacobian computation is requested. If Parameters() returns an empty vector
  // then the optimizer has failed to initialize.
  struct Parameter {
    double starting_value;              // Initial parameter value
    double min_value, max_value;        // Parameter bounds
    // Parameter step size for numerical jacobian computation, or 0 if the user
    // will always supply jacobians and numerical jacobians should never be
    // computed:
    double gradient_step;
  };
  virtual void Initialize(const std::vector<Parameter> &start) = 0;

  // Return true if Initialize() has been called (successfully or not).
  virtual bool IsInitialized() const = 0;

  // Return the current set of parameters, updated by Initialize() or
  // DoOneIteration(). These are empty() until initialization, or upon error.
  const std::vector<double> &Parameters() const { return parameters_; }

  // Is a jacobian computation at the current Parameters() requested?
  bool JacobianRequested() const { return jacobians_needed_; }

  // Do one iteration of the optimizer. The errors vector is the evaluation of
  // the objective function for the last Parameters() returned by this function
  // or the constructor. If JacobianRequested() is true then the corresponding
  // jacobians can optionally be supplied (if it is not supplied then it will
  // be computed numerically). The goal of the optimizer is to minimize
  // |errors|^2. Parameters() will now return the next set of parameters to
  // evaluate the objective function at. If this function returns false then
  // optimization is still proceeding and the Parameters() are for the next
  // objective function evaluation (and JacobianRequested() will indicate if a
  // corresponding jacobian is needed). If this function returns true then
  // optimization is complete and Parameters() are the best parameters found.
  // In either case if Parameters() returns an empty vector then the optimizer
  // has failed. The format of the jacobian vector is:
  //
  //    jacobians[j*num_parameters + i] = d error[j] / d parameter[i]
  //
  // If jacobians are requested but not supplied in the jacobians argument (as
  // the vector size is 0) then numerical jacobians will be computed.
  bool DoOneIteration(const std::vector<double> &errors,
                      const std::vector<double> &jacobians) MUST_USE_RESULT;

  // The best parameters (and the corresponding error) found so far.
  const std::vector<double>& BestParameters() const { return best_parameters_; }
  double BestError() const { return best_error_; }

 protected:
  // The version of DoOneIteration implement by the subclass. DoOneIteration()
  // updates the best solution found so far and then calls this.
  virtual bool DoOneIteration2(const std::vector<double> &errors,
                               const std::vector<double> &jacobians)
                               MUST_USE_RESULT = 0;

  std::vector<double> parameters_;
  bool jacobians_needed_ = false;

 private:
  std::vector<double> best_parameters_;         // Best found so far
  double best_error_ = __DBL_MAX__;             // Best |error|^2 so far
};

// Most optimizer libraries (e.g. ceres) have their own main loop that wants to
// take control of your app. You call some solver function that in turn calls
// your objective function callback a bunch of times, and eventually the answer
// is returned to you when the solver exits. This is simple but not convenient
// in single threaded interactive applications, i.e. if the solver takes a long
// time it will block the user interface. This class provides a workaround: the
// solver is run in a separate thread, and each time the objective function
// needs to be evaluated control is returned to the user. This is only
// efficient when the objective function cost is larger than the small overhead
// of communicating between threads.

class InteractiveOptimizer : public AbstractOptimizer {
 public:
  virtual ~InteractiveOptimizer();

  // The functions required by AbstractOptimizer.
  void Initialize(const std::vector<Parameter> &start) override;
  bool IsInitialized() const override;
  bool DoOneIteration2(const std::vector<double> &errors,
                       const std::vector<double> &jacobians) override;

  struct ErrorsAndJacobians {
    std::vector<double> errors, jacobians;
  };

 protected:
  // Run a particular optimizer library. This will be called in a separate
  // thread and returns when optimization is finished, so it is allowed to take
  // a long time. Return true on success or false on failure.
  virtual bool Optimize(const std::vector<Parameter> &start,
                        std::vector<double> *optimized_parameters)
                        MUST_USE_RESULT = 0;

  // An implementation of Optimize() should call this function to compute the
  // errors (i.e. residuals) and jacobians for a given set of parameters. This
  // function handles the details of returning the parameters through
  // DoOneIteration(). The 'errors_and_jacobians' is a pointer that is updated
  // with the returned value, you must delete it when you are finished with the
  // data.
  void Evaluate(const std::vector<double> &parameters, bool jacobians_needed,
                ErrorsAndJacobians **errors_and_jacobians) const;

  // Return true if the optimizer should shut down, i.e. because Shutdown() has
  // been called. This should be checked once per iteration in Optimize().
  bool Aborting() const;

  // Show down the optimizer thread. This should be called in each subclass
  // destructor, to make sure that the thread is not using subclass data after
  // it is deleted.
  void Shutdown();

  // Return a copy of the parameter info vector passed to Initialize().
  const std::vector<Parameter> &ParameterInfo() const;

 private:
  struct Implementation;
  Implementation *impl_ = 0;
};

// The dumbest optimizer that just keeps guessing the answer. This can run for
// ever as DoOneIteration() always return false, so the caller should give up
// at some point and call BestError().

class RandomSearchOptimizer : public AbstractOptimizer {
 public:
  ~RandomSearchOptimizer();
  void Initialize(const std::vector<Parameter> &start) override;
  bool IsInitialized() const override;
  bool DoOneIteration2(const std::vector<double> &errors,
                       const std::vector<double> &jacobians) override;
 private:
  std::vector<Parameter> parameter_info_;
};

// The classic Nelder-Mead algorithm with support for simulated annealing.

class NelderMeadOptimizer : public InteractiveOptimizer {
 public:
  ~NelderMeadOptimizer();
  bool Optimize(const std::vector<Parameter> &start,
                std::vector<double> *optimized_parameters) override;

  struct Settings {
    // The optimization will terminate if the fractional range from the best to
    // worst point in the simplex is less than the given tolerance.
    double convergence_ratio = 1e-2;
    // The optimization will terminate if the best point has error <= this. We
    // can't only rely on convergence_ratio because that will never accept
    // convergence if the minimal error is 0. The good value here is very
    // problem dependent.
    double convergence_error = 1e-6;
    // In addition, only accept convergence if the simplex is less than this
    // fraction of the parameter bounds.
    double convergence_parameter = 1e-3;
    // Seconds to spend in the annealing phase.
    double annealing_time = 0;
    // Set the (approximate) size of the initial simplex, as a fraction of the
    // parameter bounds.
    double initial_fraction = 0.1;
    // Initial and final temperatures (0 is cold, T>0 explores the space with
    // energy ~= T).
    double initial_temperature = 1;
    double final_temperature = 0.001;   // This fraction of the initial temp
  };

  // Set optimization parameters.
  void SetSettings(Settings &p) {
    settings_ = p;
  }

 private:
  Settings settings_;
  double temperature_ = 0;
  Eigen::VectorXd best_parameters_;
  double best_error_ = 0;

  // An error and its associated thermal fluctuation.
  struct Error {
    double actual, thermal;
    bool operator<(const Error &v) const { return thermal < v.thermal; }
    bool operator<=(const Error &v) const { return thermal <= v.thermal; }
  };

  Error ObjectiveFunction(const Eigen::VectorXd &p);
  double Thermal() const;
};

// An optimizer factory.

enum class OptimizerType {
  LEVENBERG_MARQUARDT,          // Implemented by CeresInteractiveOptimizer
  SUBSPACE_DOGLEG,              // Implemented by CeresInteractiveOptimizer
  RANDOM_SEARCH,
  NELDER_MEAD,
};

AbstractOptimizer *OptimizerFactory(int num_errors, OptimizerType type);


// Interface to the Ceres optimizer. This is only included if
// __TOOLKIT_USE_CERES__ is defined.

#ifdef __TOOLKIT_USE_CERES__

class CeresInteractiveOptimizer : public InteractiveOptimizer {
 public:
  explicit CeresInteractiveOptimizer(int num_errors, OptimizerType type);
  ~CeresInteractiveOptimizer();

  // Set optimization termination criteria. The optimization will terminate
  // when the fractional change in function/parameter is less than the given
  // tolerance, or the gradient magnitude is less than the given tolerance.
  void SetFunctionTolerance(double x) { function_tolerance_  = x; }
  void SetParameterTolerance(double x) { parameter_tolerance_ = x; }
  void SetGradientTolerance(double x) { gradient_tolerance_  = x; }

 protected:
  bool Optimize(const std::vector<Parameter> &start,
                std::vector<double> *optimized_parameters);

 private:
  int num_parameters_, num_errors_;     // Problem size
  OptimizerType type_;                  // Optimizer type

  // Optimization parameters.
  double function_tolerance_ = 1e-6;
  double parameter_tolerance_ = 1e-8;
  double gradient_tolerance_ = 1e-10;

  friend class CeresCostFunction;
};

#endif  // __TOOLKIT_USE_CERES__

#endif
