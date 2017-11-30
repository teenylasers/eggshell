
# Directory configuration.
#
# This file is intended to be read by both qmake and regular make, so that
# limits the syntax that can be used. We just define a bunch of variables that
# are useful in both contexts. Note that variable substitution is $$VAR in
# qmake but $(var) in make, so we need to tailor that for which tool actually
# needs the variable, but both syntaxes are syntactically correct (though not
# equivalent) in both tools.

_MYCODE = SET_ME/stuff
EIGEN_DIR =  SET_ME/eigen-3.3.4
CERES_DIR =  SET_ME/ceres-solver-1.13.0
ARPACK_DIR = SET_ME/arpack-ng
LAPACK_DIR = SET_ME/lapack-3.7.1
EIGEN_BLAS_LAPACK_LIB_DIR = $${_MYCODE}/toolkit/eigen-blas-lapack-build

CERES_INC = \
  $${CERES_DIR}/include \
  $${CERES_DIR}/internal/ceres/miniglog \
  $${CERES_DIR}/config
CERES_LIB_DIR = $${_MYCODE}/toolkit/ceres-build
ARPACK_LIB_DIR = $${_MYCODE}/toolkit/arpack-build

LUA_SOURCES = \
    ../lua-5.3.0/src/lapi.c \
    ../lua-5.3.0/src/lauxlib.c \
    ../lua-5.3.0/src/lbaselib.c \
    ../lua-5.3.0/src/lbitlib.c \
    ../lua-5.3.0/src/lcode.c \
    ../lua-5.3.0/src/lcorolib.c \
    ../lua-5.3.0/src/lctype.c \
    ../lua-5.3.0/src/ldblib.c \
    ../lua-5.3.0/src/ldebug.c \
    ../lua-5.3.0/src/ldo.c \
    ../lua-5.3.0/src/ldump.c \
    ../lua-5.3.0/src/lfunc.c \
    ../lua-5.3.0/src/lgc.c \
    ../lua-5.3.0/src/linit.c \
    ../lua-5.3.0/src/liolib.c \
    ../lua-5.3.0/src/llex.c \
    ../lua-5.3.0/src/lmathlib.c \
    ../lua-5.3.0/src/lmem.c \
    ../lua-5.3.0/src/loadlib.c \
    ../lua-5.3.0/src/lobject.c \
    ../lua-5.3.0/src/lopcodes.c \
    ../lua-5.3.0/src/loslib.c \
    ../lua-5.3.0/src/lparser.c \
    ../lua-5.3.0/src/lstate.c \
    ../lua-5.3.0/src/lstring.c \
    ../lua-5.3.0/src/lstrlib.c \
    ../lua-5.3.0/src/ltable.c \
    ../lua-5.3.0/src/ltablib.c \
    ../lua-5.3.0/src/ltm.c \
    ../lua-5.3.0/src/lundump.c \
    ../lua-5.3.0/src/lutf8lib.c \
    ../lua-5.3.0/src/lvm.c \
    ../lua-5.3.0/src/lzio.c
