
default: run_all_tests

OPTIMIZE := 0
include ../common.mk

EXCLUDE_FROM_TESTS = \
  lua_util.cc \
  matlab_engine.cc \
  plot_glut_example.cc \
  plot_wx.cc \
  viewer.cc \
  wxgl_font.cc

run_all_tests:
	g++ -o all_tests -Wall $(CCFLAGS) -D__TOOLKIT_DEFINE_TESTING_MAIN__ \
          -I. -I$(EIGEN_DIR) \
          $(filter-out $(EXCLUDE_FROM_TESTS), $(wildcard *.cc)) \
          $(EIGEN_BLAS_LAPACK_LIB) $(ARPACK_LIB) \
          -framework OpenGL
	./all_tests
	rm -f ./all_tests
