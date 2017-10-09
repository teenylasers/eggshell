
#ifndef __OPTIMIZER_H__
#define __OPTIMIZER_H__

#include "myvector"
#include "error.h"

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

class InteractiveOptimizer {
 public:
  explicit InteractiveOptimizer();
  virtual ~InteractiveOptimizer();

  // Set optimization termination criteria. The optimization will terminate
  // when the fractional change in function/parameter is less than the given
  // tolerance, or the gradient magnitude is is less than the given tolerance.
  void SetFunctionTolerance(double x) { function_tolerance_  = x; }
  void SetParameterTolerance(double x) { parameter_tolerance_ = x; }
  void SetGradientTolerance(double x) { gradient_tolerance_  = x; }

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
  void Initialize(const std::vector<Parameter> &start);

  // Return true if Initialize() has been called (successfully or not).
  bool IsInitialized() const { return impl_ != 0; }

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

  struct ErrorsAndJacobians {
    std::vector<double> errors, jacobians;
  };

 protected:
  // Run a particular optimizer library. This will be run in a separate thread
  // and returns when optimization is finished, so it is allowed to take a long
  // time. Return true on success or false on failure.
  virtual bool Optimize(const std::vector<Parameter> &start,
                        std::vector<double> *optimized_parameters)
                        MUST_USE_RESULT = 0;

  // An implementation of Optimize() should call this function to compute the
  // errors (i.e. residuals) and jacobians for a given set of parameters. This
  // function handles the details of returning the parameters through
  // DoOneIteration().
  void Evaluate(const std::vector<double> &parameters, bool jacobians_needed,
                ErrorsAndJacobians **errors_and_jacobians) const;

  struct Implementation;
  Implementation *impl_;
  std::vector<double> parameters_;
  bool jacobians_needed_;

  // Optimization parameters.
  double function_tolerance_, parameter_tolerance_, gradient_tolerance_;
};

// Interface to the Ceres optimizer. This is only included if
// __TOOLKIT_USE_CERES__ is defined.

#ifdef __TOOLKIT_USE_CERES__

enum OptimizerType {
  LEVENBERG_MARQUARDT,
  SUBSPACE_DOGLEG
};

class CeresInteractiveOptimizer : public InteractiveOptimizer {
 public:
  explicit CeresInteractiveOptimizer(int num_errors, OptimizerType type);

 protected:
  bool Optimize(const std::vector<Parameter> &start,
                std::vector<double> *optimized_parameters);

 private:
  int num_parameters_, num_errors_;     // Problem size
  OptimizerType type_;                  // Optimizer type
  friend class CeresCostFunction;
};

#endif  // __TOOLKIT_USE_CERES__

#endif
