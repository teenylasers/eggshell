
# Eigen BLAS and lapack build
#
# On OS X and linux the eigen versions of BLAS and lapack can be built easily,
# e.g. (on OS X) with
#
#    mkdir build_dir
#    cd build_dir
#    make clean && cmake ..
#    make -j4 CMAKE_CXX_FLAGS=-mmacosx-version-min=10.9 \
#      CMAKE_C_FLAGS=-mmacosx-version-min=10.9 VERBOSE=1 blas lapack
#
# On windows with mingw/cygwin this doesn't work well, as cmake takes over an
# hour to run and is difficult (for me) to configure correctly. Here is the
# alternative: we compile the needed source files directly into our own
# library with our own compiler flags. The source file list was created by
# examining the build log output by the above commands.

VPATH = $(EIGEN_BLAS_LAPACK_LIB_DIR)
.SUFFIXES:

all: make_dirs libBlasAndLapack.a
	@echo Done

OPTIMIZE := 1
include ../../common.mk

CPPSRC = \
  blas/single.cpp \
  blas/double.cpp \
  blas/complex_single.cpp \
  blas/complex_double.cpp \
  lapack/single.cpp \
  lapack/double.cpp \
  lapack/complex_single.cpp \
  lapack/complex_double.cpp \
  blas/xerbla.cpp

CSRC = \
  blas/f2c/srotm.c \
  blas/f2c/srotmg.c \
  blas/f2c/drotm.c \
  blas/f2c/drotmg.c \
  blas/f2c/lsame.c \
  blas/f2c/dspmv.c \
  blas/f2c/ssbmv.c \
  blas/f2c/chbmv.c \
  blas/f2c/sspmv.c \
  blas/f2c/zhbmv.c \
  blas/f2c/chpmv.c \
  blas/f2c/dsbmv.c \
  blas/f2c/zhpmv.c \
  blas/f2c/dtbmv.c \
  blas/f2c/stbmv.c \
  blas/f2c/ctbmv.c \
  blas/f2c/ztbmv.c \
  blas/f2c/complexdots.c

OBJ = $(CPPSRC:%.cpp=%.o) $(CSRC:%.c=%.o)

CFLAGS += -c -O3 \
  -I$(EIGEN_BLAS_LAPACK_LIB_DIR) -I$(EIGEN_BLAS_LAPACK_LIB_DIR)/blas

make_dirs:
	mkdir -p blas blas/f2c lapack

libBlasAndLapack.a: $(OBJ)
	ar rcv $@ $(OBJ)

%.o: %.cpp
	$(TOOL_PREFIX)gcc -o $@ $(CFLAGS) $(EIGEN_FLAGS) $<

%.o: %.c
	$(TOOL_PREFIX)gcc -o $@ $(CFLAGS) $(EIGEN_FLAGS) $<

#%.o: %.f
#	$(FORTRAN_COMPILER) $(CFLAGS) -o $@ $<

clean:
	-rm -rf blas lapack

realclean: clean
	-rm -f libBlasAndLapack.a
