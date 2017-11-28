
# In case this in included before the default rule of the parent makefile:
default: all

include ../qt/qt.pri

# Lua C files need to be compiled as C++ to make use of JetNum.
LUA_OBJECTS = $(patsubst %.c,%.o,$(notdir $(LUA_SOURCES)))
$(LUA_OBJECTS): CFLAGS += -x c++ -std=gnu++11 -DLUA_USE_POSIX=1 -I$(EIGEN_DIR)

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
