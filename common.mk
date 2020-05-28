# Copyright (C) 2014-2020 Russell Smith.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.

# Common environment configuration, mostly directories to libraries and
# related flags. This in intended to support both simple command line programs
# and large GUI appications, so it should be quick to execute and general
# purpose. The OPTIMIZE variable is expected to be defined externally to
# select an optimized build (1) or a debug build (0), which will influence
# some of the libraries and flags that are picked.

#############################################################################
# Top level configuration that can be overridden by setting environment
# variables:

# TOOLS_DIR is where 3rd party libraries are collected.
ifndef TOOLS_DIR
  TOOLS_DIR := $(HOME)/tools
endif

#############################################################################
# The default rule prints all variables that are exported.

show_configuration:
	# Top level configuration:
	@echo "STUFF_DIR   = $(STUFF_DIR)"
	@echo "TOOLS_DIR   = $(TOOLS_DIR)"
	# osx, windows or linux:
	@echo "PLATFORM    = $(PLATFORM)"
	# Prefix for gcc and binutils:
	@echo "TOOL_PREFIX = $(TOOL_PREFIX)"
	# Essential gcc and g++ flags to match the optimization level.
	# No library-specific stuff is added.
	@echo "CFLAGS      = $(CFLAGS)"
	@echo "CCFLAGS     = $(CCFLAGS)"
	@echo "LDFLAGS     = $(LDFLAGS)"
	# Path to the Qt distribution.
	@echo "QT_DIR      = $(QT_DIR)"
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
# Paths dependent on the top level configuration. These can be changed if you
# have things in different places.

# Set STUFF_DIR to the root of the 'stuff' repository.
STUFF_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

QT_DIR := $(TOOLS_DIR)/qt5_build
EIGEN_DIR := $(TOOLS_DIR)/eigen-3.3.4
CERES_DIR := $(TOOLS_DIR)/ceres-solver-1.13.0
ARPACK_DIR := $(TOOLS_DIR)/arpack-ng
LAPACK_DIR := $(TOOLS_DIR)/lapack-3.7.1
EIGEN_BLAS_LAPACK_LIB := $(STUFF_DIR)/toolkit/eigen-blas-lapack-build/libBlasAndLapack.a
EIGEN_BLAS_LAPACK_LIB_DIR := $(TOOLS_DIR)/eigen-3.3.4
DOCCER := $(STUFF_DIR)/doccer/doccer.exe -t $(STUFF_DIR)/doccer/template.html
FFTW_INC := $(TOOLS_DIR)/fftw-3.3.6/api
FFTW_LIB := $(TOOLS_DIR)/fftw-3.3.6/.libs/libfftw3.a

# Per-OS configuration.
ifeq ($(PLATFORM), osx)
  READLINE_INC := -I$(TOOLS_DIR)/readline-6.3
  READLINE_LIB := $(TOOLS_DIR)/readline-6.3/libreadline.a \
                  $(TOOLS_DIR)/readline-6.3/libhistory.a -lcurses
  # Note the -mmacosx-version-min is passed explicitly to the assembler as well
  # since homebrew gfortran appears to have a bug that prevents passing this
  # flag. If the flag is not passed the object files will not have the correct
  # version.
  FORTRAN_COMPILER := gfortran-8 -mmacosx-version-min=10.9 -Wa,-mmacosx-version-min=10.9
endif
ifeq ($(PLATFORM), windows)
  INNO_SETUP := '/c/Program Files (x86)/Inno Setup 5/Compil32.exe'
  #MATLAB_LIB := /c/Program Files/MATLAB/R2013a/extern/lib/win64/microsoft
  #MATLAB_INC := /c/Program\ Files/MATLAB/R2013a/extern/include
  READLINE_LIB := -lreadline
  FORTRAN_COMPILER := x86_64-w64-mingw32-gfortran.exe
endif
ifeq ($(PLATFORM), linux)
  # Using gcc under linux.
  FORTRAN_COMPILER := gfortran
endif

CERES_INC := -I$(CERES_DIR)/include \
  -I$(CERES_DIR)/internal/ceres/miniglog -I$(CERES_DIR)/config
CERES_LIB := $(STUFF_DIR)/toolkit/ceres-build
ifeq ($(OPTIMIZE), 1)
  CERES_LIB := $(CERES_LIB)/libceres-opt.a
else
  CERES_LIB := $(CERES_LIB)/libceres-dbg.a
endif
ARPACK_LIB := $(STUFF_DIR)/toolkit/arpack-build/libarpack.a

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
  TOOL_PREFIX := x86_64-w64-mingw32-

  # On windows GCC the -mms-bitfields option is enabled by default. This has
  # broken packed structures on older compilers/code, so this is the option to
  # disable it if that becomes necessary.
  CFLAGS += -mno-ms-bitfields

  # Statically link windows binaries to prevent dependence on mingw DLLs that
  # wont be distributed with the application.
  LDFLAGS += -static -static-libgcc -static-libstdc++
endif

ifeq ($(PLATFORM), osx)
  CFLAGS += -mmacosx-version-min=10.12
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

# Dead code elimination.  This only works on non-OSX platforms if
# -ffunction-sections is used.
ifeq ($(PLATFORM), osx)
  LDFLAGS += -Wl,-dead_strip
else
  LDFLAGS += -Wl,--gc-sections
endif

# C flags are also C++ flags.
CCFLAGS += $(CFLAGS) -std=gnu++11

# Public mathjax server.
MATHJAX_PUBLIC=https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.2/MathJax.js?config=TeX-AMS-MML_SVG
