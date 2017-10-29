# Common environment configuration, mostly directories to libraries and
# related flags. This in intended to support both simple command line programs
# and large GUI appications, so it should be quick to execute and general
# purpose. The OPTIMIZE variable is expected to be defined externally to
# select an optimized build (1) or a debug build (0), which will influence
# some of the libraries and flags that are picked.
#
# All instances of SET_ME below should be changed to match your local
# configuration.

# The following variables are defined here:
show_configuration:
	# osx, windows or linux:
	@echo "PLATFORM    = $(PLATFORM)"
	# Prefix for gcc and binutils:
	@echo "TOOL_PREFIX = $(TOOL_PREFIX)"
	# Essential gcc and g++ flags to match the optimization level.
	# No library-specific stuff is added.
	@echo "CFLAGS      = $(CFLAGS)"
	@echo "CCFLAGS     = $(CCFLAGS)"
	# Path to the wxWidgets wxconfig tool, e.g. '/foo/bar/wx-config' :
	@echo "WXCONFIG    = $(WXCONFIG)"
	# Eigen include directory:
	@echo "EIGEN_DIR   = $(EIGEN_DIR)"
	# Eigen g++ flags to match the optimization level:
	@echo "EIGEN_FLAGS = $(EIGEN_FLAGS)"
	# Path to the EIGEN BLAS and LAPACK libraries, and the eigen source
	# directory they were built from.
	@echo "EIGEN_BLAS_LAPACK_LIB = $(EIGEN_BLAS_LAPACK_LIB)"
	@echo "EIGEN_BLAS_LAPACK_LIB_DIR = $(EIGEN_BLAS_LAPACK_LIB_DIR)"
	# Matlab engine include and library directories (if available):
	@echo "MATLAB_INC  = $(MATLAB_INC)"
	@echo "MATLAB_LIB  = $(MATLAB_LIB)"
	# -I flag for zlib, if necessary:
	@echo "ZLIB_CFLAGS = $(ZLIB_CFLAGS)"
	# Path to the INNO setup compiler (on windows):
	@echo "INNO_SETUP  = $(INNO_SETUP)"
	# Source code directory for the ceres optimizer:
	@echo "CERES_DIR   = $(CERES_DIR)"
	# -I flags for the ceres optimizer:
	@echo "CERES_INC   = $(CERES_INC)"
	# Path to the ceres optimizer library:
	@echo "CERES_LIB   = $(CERES_LIB)"
	# Path to the ARPACK library source:
	@echo "ARPACK_DIR  = $(ARPACK_DIR)"
	# Path to the ARPACK library:
	@echo "ARPACK_LIB  = $(ARPACK_LIB)"
	# Path to the LAPACK library:
	@echo "LAPACK_DIR  = $(LAPACK_DIR)"
	# Path to the documentation system
	@echo "DOCCER      = $(DOCCER)"
	# Static linking flags on windows only:
	@echo "WINDOWS_STATIC_LINK = $(WINDOWS_STATIC_LINK)"
	# sed command to fix the output of wx-config --libs as necessary:
	@echo "WX_LIBS_SED = $(WX_LIBS_SED)"
	# Includes and link for a good version of readline.
	@echo "READLINE_INC = $(READLINE_INC)"
	@echo "READLINE_LIB = $(READLINE_LIB)"
	# Includes and link for FFTW.
	@echo "FFTW_INC = $(FFTW_INC)"
	@echo "FFTW_LIB = $(FFTW_LIB)"
	# Name of the most useful fortran compiler
	@echo "FORTRAN_COMPILER = $(FORTRAN_COMPILER)"

#############################################################################
# Determine the OS we're building on.

ifeq ($(OS), Windows_NT)
  # OS is defined in the environment in cygwin builds.
  PLATFORM := windows
else
  # OS X and linux don't seem to have anything useful in the environment to
  # distinguish them, so we need to run a shell command.
  OSTYPE := $(shell uname -s)
  ifeq ($(OSTYPE), Darwin)
    PLATFORM := osx
  else
    PLATFORM := linux
  endif
endif

#############################################################################
# Paths to things.

# _MYCODE is the root of the 'stuff' repository.

ifeq ($(PLATFORM), osx)
  _MYCODE := /SET_ME
  TOOLS_DIR := /SET_ME
  ifeq ($(OPTIMIZE), 1)
    WXCONFIG := $(TOOLS_DIR)/wxWidgets-3.1.0-opt/wx-config
  else
    WXCONFIG := $(TOOLS_DIR)/wxWidgets-3.1.0-dbg/wx-config
  endif
  EIGEN_DIR := $(TOOLS_DIR)/eigen-eigen-3.3.4
  CERES_DIR := $(TOOLS_DIR)/ceres-solver-1.13.0
  ARPACK_DIR := $(TOOLS_DIR)/arpack-ng
  LAPACK_DIR := $(TOOLS_DIR)/lapack-3.5.0
  EIGEN_BLAS_LAPACK_LIB := $(_MYCODE)/toolkit/eigen-blas-lapack-build/libBlasAndLapack.a
  EIGEN_BLAS_LAPACK_LIB_DIR := $(TOOLS_DIR)/eigen-eigen-3.3.4
  DOCCER := /SET_ME/doccer.exe
  READLINE_INC := -I/SET_ME/readline-6.3
  READLINE_LIB := /SET_ME/readline-6.3/libreadline.a \
                  /SET_ME/readline-6.3/libhistory.a -lcurses
  FFTW_INC := /SET_ME/fftw-3.3.4/api
  FFTW_LIB := /SET_ME/fftw-3.3.4/.libs/libfftw3.a
  FORTRAN_COMPILER := gfortran -mmacosx-version-min=10.9
endif

ifeq ($(PLATFORM), windows)
  _MYCODE := /SET_ME
  WXCONFIG = /SET_ME/wxWidgets-3.1.0/wx-config
  EIGEN_DIR := /SET_ME/eigen-eigen-3.3.2
  ZLIB_CFLAGS := -I/SET_ME/zlib-1.2.8
  CERES_DIR := /SET_ME/ceres-solver-1.12.0
  ARPACK_DIR := /SET_ME/arpack-ng-3.1.5
  LAPACK_DIR := /SET_ME/lapack-3.5.0
  EIGEN_BLAS_LAPACK_LIB := $(_MYCODE)/toolkit/eigen-blas-lapack-build/libBlasAndLapack.a
  EIGEN_BLAS_LAPACK_LIB_DIR := /SET_ME/eigen-eigen-3.3.2
  INNO_SETUP := '/c/Program Files (x86)/Inno Setup 5/Compil32.exe'
  DOCCER := /SET_ME/doccer.exe
  READLINE_LIB := -lreadline
  FORTRAN_COMPILER := x86_64-w64-mingw32-gfortran.exe
endif

ifeq ($(PLATFORM), linux)
  # Using gcc under linux.
  _MYCODE := /SET_ME
  TOOLS_DIR := /SET_ME
  ifeq ($(OPTIMIZE), 1)
    WXCONFIG := $(TOOLS_DIR)/wxWidgets-3.1.0-opt/wx-config
  else
    WXCONFIG := $(TOOLS_DIR)/wxWidgets-3.1.0-dbg/wx-config
  endif
  EIGEN_DIR := $(TOOLS_DIR)/eigen-eigen-3.3.4
  CERES_DIR := $(TOOLS_DIR)/ceres-solver-1.13.0
  ARPACK_DIR := $(TOOLS_DIR)/arpack-ng
  LAPACK_DIR := $(TOOLS_DIR)/lapack-3.7.1
  EIGEN_BLAS_LAPACK_LIB := $(_MYCODE)/toolkit/eigen-blas-lapack-build/libBlasAndLapack.a
  EIGEN_BLAS_LAPACK_LIB_DIR := $(TOOLS_DIR)/eigen-eigen-3.3.4
  DOCCER := /SET_ME/doccer.exe
  FORTRAN_COMPILER := gfortran
endif

CERES_INC := -I$(CERES_DIR)/include \
  -I$(CERES_DIR)/internal/ceres/miniglog -I$(CERES_DIR)/config
CERES_LIB := $(_MYCODE)/toolkit/ceres-build
ifeq ($(OPTIMIZE), 1)
  CERES_LIB := $(CERES_LIB)/libceres-opt.a
else
  CERES_LIB := $(CERES_LIB)/libceres-dbg.a
endif
ARPACK_LIB := $(_MYCODE)/toolkit/arpack-build/libarpack.a

#############################################################################
# Flags.

# Select standard compiler flags based on the optimization level. Note that
# _GLIBCXX_DEBUG must be defined consistently in all files and libraries that
# use STL, as it changes the layout of STL data structures.
ifeq ($(OPTIMIZE), 1)
  CFLAGS += -O2 -DNDEBUG
else
  CFLAGS += -O0 -g -D_GLIBCXX_DEBUG
endif

ifeq ($(PLATFORM), windows)
  # On some versions of GCC the -mms-bitfields option is enabled by default and
  # breaks packed structures.
  CFLAGS += -mno-ms-bitfields
endif

ifeq ($(PLATFORM), windows)
  TOOL_PREFIX := x86_64-w64-mingw32-
  CFLAGS += -mno-ms-bitfields
endif

ifeq ($(PLATFORM), osx)
  CFLAGS += -mmacosx-version-min=10.9
endif

ifeq ($(OPTIMIZE), 1)
  EIGEN_FLAGS = -DEIGEN_NO_DEBUG
endif

# -DEIGEN_DEFAULT_DENSE_INDEX_TYPE=int reduces sparse matrix memory usage in
# 64 bit builds, as the default is ptrdiff_t. IMPORTANT NOTE: This flag
# affects the Eigen ABI without actually altering the mangled names of C++
# symbols. If two source files use different values of this flag then they may
# emit incompatible eigen code. In optimized builds things will be inlined and
# this may not matter. In debug builds there could be multiple incompatible
# versions of each function generated, all but one of which will be stripped
# out by the linker, meaning some code will link with incorrect eigen
# functions. Therefore we define a single value of this flag here, for all to
# use.
EIGEN_FLAGS += -DEIGEN_DEFAULT_DENSE_INDEX_TYPE=int

ifeq ($(PLATFORM), windows)
  WINDOWS_STATIC_LINK := -static -static-libgcc -static-libstdc++
endif

ifeq ($(PLATFORM), windows)
  # wx-config seems not to get the proper library names in version 3.0.1.
  WX_LIBS_SED = | sed 's/\-3\.0\.a/-3.0-x86_64-w64-mingw32.a/g'
endif

# C flags are also C++ flags.
CCFLAGS += $(CFLAGS) -std=gnu++11
