
#############################################################################
# Top level configuration

OPTIMIZE := 1
default: all
-include ../common.mk           # For top level invocation
-include ../../common.mk        # For invocation in the build directory

#############################################################################
# Rules that can be run from the top level directory (all other rules need
# to be run from the build directory).

# Extract the version number from version.h.
ifndef VERSION
VERSION := $(shell $(TOOL_PREFIX)cpp -dM version.h|grep __APP_VERSION__|cut -d '"' -f 2)
endif

# Build directories
DIRS=build/lua build/dialogblocks build/toolkit

# -R means no-builtin-variables and no-builtin-rules:
MAKEFLAGS += -R
MY_MAKEFLAGS = -C build -f ../Makefile VERSION=$(VERSION)

.PHONY: dirs all run lua docs test winrelease macrelease clean cleanapp

dirs:
	mkdir -p $(DIRS)

all: dirs
	$(MAKE) $(MY_MAKEFLAGS) $(APP)
	@echo Success

run: dirs
	$(MAKE) $(MY_MAKEFLAGS) $(APP)
	build/$(APP)

lua: dirs
	$(MAKE) $(MY_MAKEFLAGS) $(LUA_OBJ)

docs: dirs
	$(MAKE) $(MY_MAKEFLAGS) docs2
docs2:
	$(DOCCER) -t ../doc/template.html -Dversion=$(VERSION) ../doc/rama.doc > rama.html
	cp ../doc/*.{png,gif} .       # So locally viewing the manual works

test: all
	./build/$(APP) -test

winrelease: all test
	$(MAKE) $(MY_MAKEFLAGS) winrelease2
winrelease2: $(APP) docs2
	$(TOOL_PREFIX)strip $(APP)
	$(INNO_SETUP) /cc ../release/rama_setup.iss
	echo $(VERSION) > latest_version.txt
	scp setup_rama.exe rama.html *.png *.gif latest_version.txt $(RAMASIMULATORUSER)@ramasimulator.org:public_html/ramasimulator

macrelease: all test
	$(MAKE) $(MY_MAKEFLAGS) macrelease2
macrelease2: $(APP) docs2
	$(TOOL_PREFIX)strip $(APP)
	rm -rf Rama.app && cp -av ../release/Rama.app .
	mkdir Rama.app/Contents/MacOS
	cp $(APP) Rama.app/Contents/MacOS
	cp rama.html ../doc/*.{png,gif} Rama.app/Contents/Resources
	hdiutil create -volname Rama -srcfolder Rama.app -ov -format UDZO Rama.dmg
	echo $(VERSION) > latest_version_mac.txt
	scp Rama.dmg rama.html *.png *.gif latest_version_mac.txt $(RAMASIMULATORUSER)@ramasimulator.org:public_html/ramasimulator

cleanapp:
	-rm -f build/*.d build/*.o build/$(APP) $(subst ../,,$(PCH)) *.d

clean: cleanapp
	-rm -rf build dialogblocks/*.pjd.bak

#############################################################################
# Application files (including single-file libraries).

OBJ = toolkit/error.o toolkit/colormaps.o toolkit/mat_file.o \
      toolkit/optimizer.o toolkit/testing.o toolkit/md5.o toolkit/lua_util.o \
      toolkit/gl_utils.o toolkit/wxgl_font.o toolkit/plot.o toolkit/plot_wx.o \
      toolkit/viewer.o toolkit/camera.o toolkit/femsolver.o toolkit/mystring.o \
      toolkit/lua_model_viewer.o toolkit/lua_vector.o toolkit/trace.o \
      toolkit/shaders.o toolkit/color_based_selection.o \
      cavity.o app.o common.o shape.o mesh.o solver.o \
      clipper.o triangle.o user_script_util.o \
      dialogblocks/mainwin.o dialogblocks/aboutwin.o dialogblocks/sweep.o
APP = rama.exe

ifneq ($(PLATFORM), windows)
OBJ += toolkit/crash_handler.o
endif

# Precompiled headers, are most useful for speeding up the windows compile,
# they don't seem to have much speed impact on mac and linux.
ifeq ($(OS), Windows_NT)
PCH = ../stdwx.h.gch
endif

#############################################################################
# Compiler configuration: directories, files and flags.

# Per-OS configuration.
ifeq ($(PLATFORM), windows)
  # WX libraries. The sed command is required because wx-config seems not to
  # get the proper library names in version 3.0.1 for windows:
  CCFLAGS += -Wno-unused-local-typedefs
  LUA_EXTRA_CFLAGS =
  OBJ += rc.o
  SIZE_T_PRINTF = '"I"'
  UNDEF_X11_H =
else
  LUA_EXTRA_CFLAGS = -DLUA_USE_POSIX
  ifneq ($(PLATFORM), osx)
    CCFLAGS += -Wno-unused-local-typedefs
  endif
  SIZE_T_PRINTF = '"z"'
  UNDEF_X11_H = undef_x11.h
endif

ifdef MATLAB_INC
  ENGINE_LIBS = libmx.a libeng.a
  ENGINE_EXTRA_CFLAGS = -DMATLAB_ENGINE_AVAILABLE
  OBJ += matlab_engine.o
endif

# Compiler configuration.
CC := $(shell $(WXCONFIG) --cc)
CXX := $(shell $(WXCONFIG) --cxx)
RESCOMP := $(shell $(WXCONFIG) --rescomp)
MYFLAGS := -I. -I.. -Werror -Wall -MMD -Wno-sign-compare -Wno-inconsistent-missing-override
CFLAGS += $(MYFLAGS)
CCFLAGS += $(MYFLAGS) $(shell $(WXCONFIG) --cxxflags) -D__TOOLKIT_WXWINDOWS__=1 \
  $(ENGINE_EXTRA_CFLAGS) \
  $(ZLIB_CFLAGS) -D__TOOLKIT_MAT_FILE_USE_ZLIB__ -D__TOOLKIT_USE_CERES__ \
  $(EIGEN_FLAGS) $(CERES_INC) -I../../toolkit

#############################################################################
# Extra configuration for application files.

$(OBJ) $(PCH): CCFLAGS += -I$(LUA_DIR) -I$(EIGEN_DIR)

# Disable some warnings-as-errors for dialogblocks code.
mainwin.o: CCFLAGS += -Wno-error=unused-variable

# Triangle library. We don't use the CPU86 flag as it tries to set the _PC_53
# precision flag in the FPU control word, which is not supported in x64.
# The -fwrapv is because part of the triangle library relies on the behavior
# of signed overflow.
triangle.o: CFLAGS += -DNO_TIMER -DTRILIBRARY \
  -DANSI_DECLARATORS -DEXTERNAL_TEST -fwrapv

#############################################################################
# Lua.

LUA_DIR = ../lua-5.3.0/src
LUA_SRC = lapi.c lcode.c lctype.c ldebug.c ldo.c ldump.c lfunc.c lgc.c llex.c \
  lmem.c lobject.c lopcodes.c lparser.c lstate.c lstring.c ltable.c \
  ltm.c lundump.c lvm.c lzio.c \
  lauxlib.c lbaselib.c lbitlib.c lcorolib.c ldblib.c liolib.c \
  lmathlib.c loslib.c lstrlib.c ltablib.c lutf8lib.c loadlib.c linit.c

LUA_OBJ = $(LUA_SRC:%.c=lua/%.o)

# Lua C files need to be compiled as C++ to make use of JetNum.
$(LUA_OBJ): CFLAGS += -x c++ -std=gnu++11 $(LUA_EXTRA_CFLAGS) -I$(EIGEN_DIR)

#############################################################################
# Matlab engine.

matlab_engine.o: CCFLAGS += -I$(MATLAB_INC)

# Create Matlab DLL exports libraries for 64 bit MinGW.
# You also need to add the matlab DLL directory to the system path.

# Matrix library.
libmx.a:
	rm -f libmx.def libmx.a
	echo EXPORTS > libmx.def
	$(TOOL_PREFIX)nm "${MATLAB_LIB}/libmx.lib" | grep '^0000000000000000 T ' | \
  	  grep -v '0 T [.]text' | grep -v '0 T MX' | grep -v @ | grep -v '[?]' | \
  	  cut -d ' ' -f 3- >> libmx.def
	$(TOOL_PREFIX)dlltool -d libmx.def --dllname libmx.dll -l libmx.a
	rm -f libmx.def

# Engine library.
libeng.a:
	rm -f libeng.def libeng.a
	echo EXPORTS > libeng.def
	$(TOOL_PREFIX)nm "${MATLAB_LIB}/libeng.lib" | grep '^0000000000000000 T ' | \
  	  grep -v '0 T [.]text' | grep -v '0 T ENG' | grep -v @ | grep -v '[?]' | \
  	  cut -d ' ' -f 3- >> libeng.def
	$(TOOL_PREFIX)dlltool -d libeng.def --dllname libeng.dll -l libeng.a
	rm -f libeng.def

#############################################################################
# Rules.

.SUFFIXES:
.PRECIOUS: %.h.gch

%.o: ../%.cc $(PCH)
	$(CXX) -c $(CCFLAGS) -o $@ $<

dialogblocks/%.o: ../dialogblocks/%.cc $(PCH)
	$(CXX) -c $(CCFLAGS) -o $@ $<

toolkit/%.o: ../../toolkit/%.cc $(PCH)
	$(CXX) -c $(CCFLAGS) -o $@ $<

%.o: ../%.c
	$(CC) -c $(CFLAGS) -o $@ $<

lua/%.o: $(LUA_DIR)/%.c
	$(CC) -c $(CFLAGS) -o $@ $<

%.h.gch: %.h $(UNDEF_X11_H)
	$(CXX) $(CCFLAGS) $<

rc.o: ../dialogblocks/rama.rc
	$(RESCOMP) -i $< -o $@

$(APP): $(OBJ) $(LUA_OBJ) $(ENGINE_LIBS)
	$(CXX) $(CCFLAGS) -o $(APP) $(WINDOWS_STATIC_LINK) \
            $(OBJ) $(LUA_OBJ) $(ENGINE_LIBS) $(CERES_LIB) \
            $(ARPACK_LIB) $(EIGEN_BLAS_LAPACK_LIB) \
            $(shell $(WXCONFIG) --libs core,base,gl,aui,net $(WX_LIBS_SED))

text2bin.exe: ../text2bin.cc
	$(TOOL_PREFIX)gcc$(TOOL_SUFFIX) -o $@ $<

user_script_util.o: ../user_script_util.lua text2bin.exe
	./text2bin.exe < $< > user_script_util.c
	$(TOOL_PREFIX)gcc$(TOOL_SUFFIX) $(CFLAGS) -c user_script_util.c

# Create an include file that undefines all macros in /usr/include/X11 that
# don't start with an underscore.
undef_x11.h:
	grep -r '[ \t]*#[ \t]*define[ \t][ \t]*[A-Za-z]' /usr/include/X11/ | \
          sed 's/^[^#]*#[ \t]*define[ \t]*\([A-Za-z0-9_]*\).*/#undef \1/' > $@

# Include dependencies (only works from build directory).
-include *.d $(DIRS:build/%=%/*.d)
