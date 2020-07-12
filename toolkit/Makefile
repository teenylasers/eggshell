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

default: run_all_tests

OPTIMIZE := 0
include ../common.mk

EXCLUDE_FROM_TESTS = \
  lua_util.cc \
  matlab_engine.cc \
  plot_glut_example.cc \
  viewer.cc

run_all_tests:
	g++ -o all_tests -Wall $(CCFLAGS) -D__TOOLKIT_DEFINE_TESTING_MAIN__ \
          -I. -I$(EIGEN_DIR) \
          $(filter-out $(EXCLUDE_FROM_TESTS), $(wildcard *.cc)) \
          $(EIGEN_BLAS_LAPACK_LIB) $(ARPACK_LIB) \
          -framework OpenGL
	./all_tests
	rm -f ./all_tests
