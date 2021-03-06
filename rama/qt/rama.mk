
# This is the top level makefile. We define add some per-target configuration
# and other rules that can't be expressed easily in qmake, then include the
# qmake-generated makefile below.

# In case this in included before the default rule of the parent makefile:
default: all

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

# Include some variables that are common between the qmake file and this one.
include ../qt/qt.pri

# Lua C files need to be compiled as C++ to make use of JetNum.
LUA_OBJECTS = $(patsubst %.c,%.o,$(notdir $(LUA_SOURCES)))
LUA_FLAGS = -x c++ -std=gnu++11 -I$(EIGEN_DIR)
ifneq ($(PLATFORM), windows)
  LUA_FLAGS += -DLUA_USE_POSIX=1
endif
$(LUA_OBJECTS): CFLAGS += $(LUA_FLAGS)

# Allow including my_jet.h:
lua_vector.o: CXXFLAGS += -I..
lua_model_viewer.o: CXXFLAGS += -I..
mesh.o: CXXFLAGS += -I..
shape.o: CXXFLAGS += -I..
solver.o: CXXFLAGS += -I..
lua_model_viewer_qt.o: CXXFLAGS += -I..
cavity_qt.o: CXXFLAGS += -I..
main_window.o: CXXFLAGS += -I..
moc_lua_model_viewer_qt.o: CXXFLAGS += -I..
sweep.o: CXXFLAGS += -I..

# Triangle library. We don't use the CPU86 flag as it tries to set the _PC_53
# precision flag in the FPU control word, which is not supported in x64.
# The -fwrapv is because part of the triangle library relies on the behavior
# of signed overflow.
triangle.o: CFLAGS += -DNO_TIMER -DTRILIBRARY \
  -DANSI_DECLARATORS -DEXTERNAL_TEST -fwrapv \
  -Wno-unused-parameter -Wno-sign-compare

text2bin.exe: ../text2bin.cc
	$(CC) $(CXXFLAGS) -o $@ $<

user_script_util.o: ../user_script_util.lua text2bin.exe
	./text2bin.exe < $< > user_script_util.c
	$(CC) $(CFLAGS) -c user_script_util.c

# Now include the qmake-generated makefile.
include Makefile
