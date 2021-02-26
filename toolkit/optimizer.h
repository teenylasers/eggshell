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

// An abstract interface to an optimizer. The goal of the optimizer is to
// minimize |errors|^2 where the errors vector is computed by an objective
// function for some parameters. This is designed to be used by a single
// threaded interactive application. The idea is that you subclass this,
// provide implementations for Initialize2() and DoOneIteration2(), and then do
// the following sequence:
//   1. Call SetSettings(), if necessary
//   2. Call Initialize()
//   3. Call Parameters() and JacobianRequested(). If the Parameters() vector
//      is empty then there has been some error.
//   4. Compute the objective function error vector and (optionally) Jacobians
//      at the parameters indicated by Parameters().
//   5. Call DoOneIteration() with the error and Jacobian information.
//   6. If DoOneIteration() returned false, go back to step 3.
//   7. If DoOneIteration() returned true, optimization is complete.
//   8. Call OptimizerSucceeded() to see if the optimization was a success.
//   9. Call BestParameters() to return the best parameters found. If the
//      optimization did not succeed then these parameters may not be very
//      good.

class AbstractOptimizer {
 public:
  virtual ~AbstractOptimizer() = 0;

  // An abstract settings object than can clone itself.
  class AbstractSettings {
  public:
    virtual ~AbstractSettings();
    virtual AbstractSettings *Clone() const = 0;
  };

  // Set and get the settings for this optimizer.
  void SetSettings(const AbstractSettings &settings) {
    delete settings_;
    settings_ = settings.Clone();
  }
  const AbstractSettings* GetSettings() const { return settings_; }

  // Each parameter to optimize needs the following information.
  struct ParameterInformation {
    double starting_value;              // Initial parameter value
    double min_value, max_value;        // Parameter bounds
    // Parameter step size for numerical jacobian computation, or 0 if the user
    // will always supply jacobians and numerical jacobians should never be
    // computed:
    double gradient_step;
  };

  // Initialize the optimizer with a starting set of parameters, and which
  // expects an error vector of the given size. Parameters() will now return
  // the first set of parameters to evaluate the objective function at (which
  // are not necessarily the same as the starting parameters) and
  // JacobianRequested() will indicate if a corresponding Jacobian computation
  // is requested. If Parameters() returns an empty vector then the optimizer
  // has failed to initialize.
  void Initialize(const std::vector<ParameterInformation> &info,
                  int num_errors);

  // Return the parameters to evaluate the objective function at, updated by
  // Initialize() or DoOneIteration(). These are empty() until initialization.
  const std::vector<double> &Parameters() const { return parameters_; }

  // Is a Jacobian computation at the current Parameters() requested?
  bool JacobianRequested() const { return jacobians_needed_; }

  // Do one iteration of the optimizer. The errors vector is the evaluation of
  // the objective function for Parameters(), and should have the size
  // num_errors. If JacobianRequested() is true then the corresponding Jacobian
  // can optionally be supplied (if it is not supplied then it will be computed
  // numerically if the gradient_step values are set). If this function returns
  // false then optimization is still proceeding and Parameters() now contains
  // the next set of parameters to evaluate the objective function at. If this
  // function returns true then optimization is complete: examine
  // OptimizerSucceeded() and BestParameters(). The format of the jacobian
  // vector is:
  //    jacobians[j*num_parameters + i] = d error[j] / d parameter[i]
  bool DoOneIteration(const std::vector<double> &errors,
                      const std::vector<double> &jacobians) MUST_USE_RESULT;

  // When DoOneIteration() returns true, did the optimization succeed?
  bool OptimizerSucceeded() const { return optimizer_succeeded_; }

  // The best parameters (and the corresponding best error) found so far.
  const std::vector<double>& BestParameters() const { return best_parameters_; }
  double BestError() const { return best_error_; }

  // Update the best parameters (and the corresponding best error) if you have
  // a better one.
  void UpdateBest(const std::vector<double> &params, double error);

  // Access values passed to Initialize().
  const std::vector<ParameterInformation>& ParameterInfo() const
    { return parameter_info_; }
  int NumErrors() const { return num_errors_; }

 protected:
  // The version of Initialize() implemented by the subclass. Initialize()
  // copies the parameter info to parameter_info_ and then calls this.
  virtual void Initialize2() = 0;

  // The version of DoOneIteration implement by the subclass. DoOneIteration()
  // updates the best solution found so far and then calls this.
  virtual bool DoOneIteration2(const std::vector<double> &errors,
                               const std::vector<double> &jacobians)
                               MUST_USE_RESULT = 0;

  AbstractSettings *settings_ = 0;
  // Current parameters and Jacobian requested, set by Initialize2() and
  // DoOneIteration2().
  std::vector<double> parameters_;
  bool jacobians_needed_ = false;
  // Set when DoOneIteration2 returns true:
  bool optimizer_succeeded_ = false;

 private:
  // Copied from Initialize():
  std::vector<ParameterInformation> parameter_info_;
  int num_errors_ = 0;
  // Best parameters and |error|^2 found so far.
  std::vector<double> best_parameters_;
  double best_error_ = __DBL_MAX__;
};

// An optimizer factory.

enum class OptimizerType {
  UNKNOWN,
  LEVENBERG_MARQUARDT,          // Implemented by CeresInteractiveOptimizer
  SUBSPACE_DOGLEG,              // Implemented by CeresInteractiveOptimizer
  RANDOM_SEARCH,
  NELDER_MEAD,
  REPEATED_LEVENBERG_MARQUARDT,
};

AbstractOptimizer *OptimizerFactory(OptimizerType type);

// Most optimizer libraries (e.g. ceres) have their own main loop that wants to
// take control of your app. You call some solver function that in turn calls
// your objective function callback a bunch of times, and eventually the answer
// is returned to you when the solver exits. This is simple but not convenient
// in single threaded interactive applications, i.e. if the solver takes a long
// time it will block the user interface. This class provides a workaround: the
// solver is run in a separate thread, and each time the objective function
// needs to be evaluated, control is returned to the user. This is only
// efficient when the objective function cost is larger than the small overhead
// of communicating between threads.

class InteractiveOptimizer : public AbstractOptimizer {
 public:
  virtual ~InteractiveOptimizer();

  // The functions required to be implemented by AbstractOptimizer. Subclasses
  // of this don't implement these, they implement Optimize() below.
  void Initialize2() override;
  bool DoOneIteration2(const std::vector<double> &errors,
                       const std::vector<double> &jacobians) override;

  struct ErrorsAndJacobians {
    std::vector<double> errors, jacobians;
    void Swap(ErrorsAndJacobians &ej) {
      errors.swap(ej.errors);
      jacobians.swap(ej.jacobians);
    }
  };

 protected:
  // A subclass overrides this to run a particular optimizer library. This will
  // be called in a separate thread and returns when optimization is finished,
  // so it is allowed to take a long time. Return true on success or false on
  // failure.
  virtual bool Optimize() = 0;

  // An implementation of Optimize() should call this function to compute the
  // errors (i.e. residuals) and jacobians for a given set of parameters. This
  // function handles the details of returning the parameters through
  // DoOneIteration().
  void Evaluate(const std::vector<double> &parameters, bool jacobians_needed,
                ErrorsAndJacobians *errors_and_jacobians) const;

  // Called once per iteration in Optimize() to see if the optimizer should
  // shut down, i.e. because Shutdown() has been called. Returns true to shut
  // down.
  bool Aborting() const;

  // Shut down the optimizer thread. This should be called in each subclass
  // destructor, to make sure that the thread is not using subclass data after
  // it is deleted.
  void Shutdown();

 private:
  struct Implementation;
  Implementation *impl_ = 0;
};

// An optimizer that encapsulates another. It runs the second optimizer
// repeatedly with random starting points, keeping track of the best solution
// found. The settings given to this optimizer are passed to the sub-optimizer.

class RepeatedOptimizer : public AbstractOptimizer {
 public:
  explicit RepeatedOptimizer(OptimizerType type) : type_(type) {}
  ~RepeatedOptimizer();
  void Initialize2() override;
  bool DoOneIteration2(const std::vector<double> &errors,
                      const std::vector<double> &jacobians) override;
 private:
  OptimizerType type_;
  AbstractOptimizer *opt_ = 0;
  void CreateSubOptimizer();
};

// The dumbest optimizer that just keeps guessing the answer. This can run for
// ever as DoOneIteration() always return false, so the caller should give up
// at some point and call BestError().

class RandomSearchOptimizer : public AbstractOptimizer {
 public:
  ~RandomSearchOptimizer();
  void Initialize2() override;
  bool DoOneIteration2(const std::vector<double> &errors,
                       const std::vector<double> &jacobians) override;
};

// The classic Nelder-Mead algorithm with support for simulated annealing.

class NelderMeadOptimizer : public InteractiveOptimizer {
 public:
  ~NelderMeadOptimizer();
  bool Optimize() override;

  struct Settings : public AbstractSettings {
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

    Settings *Clone() const { return new Settings(*this); }
  };

 private:
  double temperature_ = 0;
  Eigen::VectorXd best_parameters_;

  // An error and its associated thermal fluctuation.
  struct Error {
    double actual, thermal;
    bool operator<(const Error &v) const { return thermal < v.thermal; }
    bool operator<=(const Error &v) const { return thermal <= v.thermal; }
  };

  Error ObjectiveFunction(const Eigen::VectorXd &p);
  double Thermal() const;
};

// Interface to the Ceres optimizer. This is only included if
// __TOOLKIT_USE_CERES__ is defined.

#ifdef __TOOLKIT_USE_CERES__

class CeresInteractiveOptimizer : public InteractiveOptimizer {
 public:
  explicit CeresInteractiveOptimizer(OptimizerType type);
  ~CeresInteractiveOptimizer();

  struct Settings : public AbstractSettings {
    // Optimization termination criteria. The optimization will terminate when
    // the fractional change in function/parameter is less than the given
    // tolerance, or the gradient magnitude is less than the given tolerance.
    double function_tolerance = 1e-6;
    double parameter_tolerance = 1e-8;
    double gradient_tolerance = 1e-10;

    Settings *Clone() const { return new Settings(*this); }
  };

 protected:
  bool Optimize();

 private:
  OptimizerType type_;                  // Optimizer type
  friend class CeresCostFunction;
};

#endif  // __TOOLKIT_USE_CERES__

#endif
