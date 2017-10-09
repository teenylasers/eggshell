
# Building ceres for the x86_64-w64-mingw32 toolchain on cygwin is a big pain,
# because that toolchain mixes gnu-isms, linux-isms and windows-isms in a way
# that confuses many libraries. Ceres uses cmake and cygwin cmake does not seem
# to natively support x86_64-w64-mingw32, so I used the following procedure on
# linux to extract a list of gcc flags and source files, from which I created
# this makefile:
#
#   cd /tmp ; tar fxvz ceres-solver-1.13.0.tar.gz
#   mkdir ceres-bin ; cd ceres-bin
#   cmake ../ceres-solver-1.13.0 -DLAPACK=OFF -DCXSPARSE=OFF -DEIGENSPARSE=ON
#         -DGFLAGS=OFF -DMINIGLOG=ON -DSCHUR_SPECIALIZATIONS=OFF -DOPENMP=OFF
#         -DCMAKE_VERBOSE_MAKEFILE=true -DEIGEN_INCLUDE_DIR=<eigen-include-path>
#   make >& LOG
#   # Inspect LOG, copy flags and filenames to this Makefile.
#   # Also copy out the generated file ceres/internal/config.h
#
# Ceres has a few annoying OS dependencies (annoying because it's a numerical
# engine and could be completely OS independent if it wasn't for logging and
# benchmarking code). The associated patch deals with the various issues for
# ceres version 1.13.0.

OPTIMIZE = 1
default: all
include ../../common.mk

LIBNAME = $(notdir $(CERES_LIB))

CCFLAGS += \
  -Werror=all -Werror=extra -Wno-unknown-pragmas -Wno-sign-compare \
  -Wno-unused-parameter -Wno-missing-field-initializers -Wno-ignored-qualifiers \
  -Wno-overloaded-virtual \
  -I$(CERES_DIR)/include -I$(CERES_DIR)/internal \
  -I$(CERES_DIR)/internal/ceres  -I$(CERES_DIR)/internal/ceres/miniglog \
  -I$(EIGEN_DIR) $(EIGEN_FLAGS) -I. -DANDROID

ifneq ($(PLATFORM), osx)
CCFLAGS += -Wno-maybe-uninitialized -Wno-unused-local-typedefs
endif

SRC = \
  array_utils.cc \
  blas.cc \
  block_evaluate_preparer.cc \
  block_jacobi_preconditioner.cc \
  block_jacobian_writer.cc \
  block_random_access_dense_matrix.cc \
  block_random_access_diagonal_matrix.cc \
  block_random_access_matrix.cc \
  block_random_access_sparse_matrix.cc \
  block_sparse_matrix.cc \
  block_structure.cc \
  c_api.cc \
  canonical_views_clustering.cc \
  cgnr_solver.cc \
  callbacks.cc \
  compressed_col_sparse_matrix_utils.cc \
  compressed_row_jacobian_writer.cc \
  compressed_row_sparse_matrix.cc \
  conditioned_cost_function.cc \
  conjugate_gradients_solver.cc \
  coordinate_descent_minimizer.cc \
  corrector.cc \
  covariance.cc \
  covariance_impl.cc \
  cxsparse.cc \
  dense_normal_cholesky_solver.cc \
  dense_qr_solver.cc \
  dense_sparse_matrix.cc \
  detect_structure.cc \
  dogleg_strategy.cc \
  dynamic_compressed_row_jacobian_writer.cc \
  dynamic_compressed_row_sparse_matrix.cc \
  dynamic_sparse_normal_cholesky_solver.cc \
  eigensparse.cc \
  evaluator.cc \
  file.cc \
  function_sample.cc \
  gradient_checker.cc \
  gradient_checking_cost_function.cc \
  gradient_problem.cc \
  gradient_problem_solver.cc \
  implicit_schur_complement.cc \
  inner_product_computer.cc \
  is_close.cc \
  iterative_schur_complement_solver.cc \
  levenberg_marquardt_strategy.cc \
  lapack.cc \
  line_search.cc \
  line_search_direction.cc \
  line_search_minimizer.cc \
  line_search_preprocessor.cc \
  linear_least_squares_problems.cc \
  linear_operator.cc \
  linear_solver.cc \
  local_parameterization.cc \
  loss_function.cc \
  low_rank_inverse_hessian.cc \
  minimizer.cc \
  normal_prior.cc \
  parameter_block_ordering.cc \
  partitioned_matrix_view.cc \
  polynomial.cc \
  preconditioner.cc \
  preprocessor.cc \
  problem.cc \
  problem_impl.cc \
  program.cc \
  reorder_program.cc \
  residual_block.cc \
  residual_block_utils.cc \
  schur_complement_solver.cc \
  schur_eliminator.cc \
  schur_jacobi_preconditioner.cc \
  schur_templates.cc \
  scratch_evaluate_preparer.cc \
  single_linkage_clustering.cc \
  solver.cc \
  solver_utils.cc \
  sparse_cholesky.cc \
  sparse_matrix.cc \
  sparse_normal_cholesky_solver.cc \
  split.cc \
  stringprintf.cc \
  suitesparse.cc \
  triplet_sparse_matrix.cc \
  trust_region_preprocessor.cc \
  trust_region_minimizer.cc \
  trust_region_step_evaluator.cc \
  trust_region_strategy.cc \
  types.cc \
  visibility.cc \
  visibility_based_preconditioner.cc \
  wall_time.cc \
  schur_eliminator_d_d_d.cc \
  partitioned_matrix_view_d_d_d.cc \
  logging.cc


OBJ = $(SRC:%.cc=%.o)

all: $(LIBNAME)

$(LIBNAME): $(OBJ)
	$(TOOL_PREFIX)ar rcv $(LIBNAME) $(OBJ)

%.o: $(CERES_DIR)/internal/ceres/%.cc
	$(TOOL_PREFIX)g++$(TOOL_SUFFIX) $(CCFLAGS) -o $@ -c $<

%.o: $(CERES_DIR)/internal/ceres/generated/%.cc
	$(TOOL_PREFIX)g++$(TOOL_SUFFIX) $(CCFLAGS) -o $@ -c $<

%.o: $(CERES_DIR)/internal/ceres/miniglog/glog/%.cc
	$(TOOL_PREFIX)g++$(TOOL_SUFFIX) $(CCFLAGS) -o $@ -c $<

clean:
	-rm -f *.o

realclean: clean
	-rm -f *.a
