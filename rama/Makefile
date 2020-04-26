
#############################################################################
# Top level configuration

# OPTIMIZE is 1 for optimizations on, or 0 for debug features on.
OPTIMIZE := 1
# GUI is either wx or qt
GUI := qt

default: all
-include ../common.mk           # For top level invocation
-include ../../common.mk        # For invocation in the build directory

# Examples distributed along with the binary.
EXAMPLES = \
  model_ALMA_coupler_Exy.lua \
  model_antenna_pattern_test.lua \
  model_horn_Ez.lua \
  model_schrodinger_test.lua \
  model_waveguide_Exy.lua \
  model_waveguide_Ez.lua \
  model_waveguide_bend.lua \
  model_waveguide_modes.lua

#############################################################################
# Rules that can be run from the top level directory (all other rules need
# to be run from the build directory).

# Make sure that any failures in piped commands are recognized.
SHELL=/bin/bash -o pipefail

# Check that GUI is set correctly.
ifeq ($(GUI), qt)
else ifeq ($(GUI), wx)
else
  $(error Bad value for GUI)
endif

# Extract the version number from version.h.
ifndef VERSION
VERSION := $(shell $(TOOL_PREFIX)cpp -dM version.h|grep __APP_VERSION__|cut -d '"' -f 2)
endif

# Build directories
DIRS = build/lua build/toolkit
ifeq ($(GUI), qt)
  DIRS += build/qt
else
  DIRS += build/dialogblocks
endif

# -R means no-builtin-variables and no-builtin-rules:
MAKEFLAGS += -R
MY_MAKEFLAGS = -C build -f ../Makefile VERSION=$(VERSION)

.PHONY: dirs all run lua docs test windeploy macdeploy clean cleanapp

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
	cp ../doc/*.{png,gif} .       # So locally building and viewing the manual works
	$(DOCCER) -Dversion=$(VERSION) ../doc/rama.doc | sed 's@__MATHJAX__@$(MATHJAX_PUBLIC)@' | sed 's@__TITLE__@Rama@' > rama.html

test: unittest script_test

unittest: all
	./build/$(APP) -unittest

script_test: all
	./build/$(APP) -test tests/test_vector.lua
	./build/$(APP) -test tests/test_complex.lua
	./build/$(APP) -test tests/test_autodiff.lua
	./build/$(APP) -test tests/test_ALMA_coupler.lua
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=1
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=2
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=3
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=4
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=5
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=6
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=7
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=8
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=9
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=10
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=11
	./build/$(APP) -test tests/test_anisotropy.lua -test_number=12
	./build/$(APP) -test tests/test_regressions.lua
	@echo
	@echo '*** Success ***'

windeploy: all test
	$(MAKE) $(MY_MAKEFLAGS) windeploy2
windeploy2: $(APP) docs2
	$(TOOL_PREFIX)strip $(APP)
	$(INNO_SETUP) /cc ../release/rama_setup.iss
	echo $(VERSION) > latest_version.txt
	scp setup_rama.exe rama.html *.png *.gif latest_version.txt $(RAMASIMULATORUSER)@ramasimulator.org:public_html/ramasimulator

macdeploy: all test
	$(MAKE) $(MY_MAKEFLAGS) macdeploy2
macdeploy2: $(APP) docs2
	$(TOOL_PREFIX)strip $(APP)
	rm -rf Rama.app && cp -av ../release/Rama.app .
	mkdir Rama.app/Contents/MacOS
	cp $(APP) Rama.app/Contents/MacOS
	cp rama.html ../doc/*.{png,gif} Rama.app/Contents/Resources
	mkdir Rama.app/Examples
	cp $(addprefix ../,$(EXAMPLES)) Rama.app/Examples
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
      toolkit/gl_utils.o toolkit/gl_font.o toolkit/plot.o toolkit/plot_gui.o \
      toolkit/viewer.o toolkit/camera.o toolkit/femsolver.o toolkit/mystring.o \
      toolkit/lua_vector.o toolkit/trace.o \
      toolkit/shaders.o \
      common.o shape.o mesh.o solver.o \
      clipper.o triangle.o user_script_util.o

ifeq ($(GUI), qt)
  OBJ += toolkit/thread.o cavity_qt.o qt/main.o qt/main_window.o qt/about.o \
         qt/sweep.o qt/error_window.o toolkit/lua_model_viewer_qt.o \
         qt/moc_main_window.o qt/moc_about.o  qt/moc_sweep.o \
         qt/moc_error_window.o qt/moc_lua_model_viewer_qt.o \
         qt/plugin_import.o
  UI_HEADERS = qt/ui_about.h qt/ui_error_window.h qt/ui_main_window.h \
               qt/ui_sweep.h
else
  OBJ += cavity.o app.o toolkit/lua_model_viewer_wx.o \
         dialogblocks/mainwin.o dialogblocks/aboutwin.o dialogblocks/sweep.o
  # Precompiled headers, are most useful for speeding up the windows compile,
  # they don't seem to have much speed impact on mac and linux.
  ifeq ($(PLATFORM), windows)
    PCH = ../stdwx.h.gch
  endif
endif

APP = rama.exe

ifneq ($(PLATFORM), windows)
  OBJ += toolkit/crash_handler.o
endif

#############################################################################
# Compiler configuration: directories, files and flags.

# Per-OS configuration that is common across GUI toolkits.
ifneq ($(PLATFORM), windows)
  LUA_EXTRA_CFLAGS = -DLUA_USE_POSIX
endif

ifeq ($(GUI), qt)
  CC := $(TOOL_PREFIX)gcc
  CXX := $(TOOL_PREFIX)g++
  GUI_DEF = -DQT_DEPRECATED_WARNINGS -DQT_DISABLE_DEPRECATED_BEFORE=0x060000 \
            -DQT_WIDGETS_LIB -DQT_GUI_LIB -DQT_NETWORK_LIB -DQT_CORE_LIB
  GUI_INC = -I$(QT_DIR)/include -I$(QT_DIR)/include/QtWidgets \
            -I$(QT_DIR)/include/QtGui -I$(QT_DIR)/include/QtNetwork \
            -I$(QT_DIR)/include/QtCore
  ifeq ($(OPTIMIZE), 1)
    CCFLAGS += -DQT_NO_DEBUG
    D :=
  else
    CCFLAGS += -DQT_QML_DEBUG
    ifeq ($(PLATFORM), osx)
      D := _debug
    else
      D := d
    endif
  endif
  GUI_LIBS = -L$(QT_DIR)/lib -L$(TOOLS_DIR)/Qt-5.9.2/plugins/platforms \
    -lQt5Widgets$(D) -lQt5Gui$(D) -lQt5Core$(D) -lQt5Network$(D) \
    -lqtlibpng$(D) -lqtharfbuzz$(D) -lqtpcre2$(D) -lqtfreetype$(D) \
    -lQt5FontDatabaseSupport$(D) -lQt5ThemeSupport$(D)
  ifeq ($(PLATFORM), osx)
    GUI_LIBS += -lqcocoa$(D) -lcups -lz -lQt5PrintSupport$(D) \
      -lQt5ClipboardSupport$(D) -lQt5AccessibilitySupport$(D) \
      -lQt5CglSupport$(D) -lQt5GraphicsSupport$(D) \
      -framework OpenGL -framework AGL -framework IOKit \
      -framework Carbon -framework AppKit -framework SystemConfiguration \
      -framework Security
    CCFLAGS += -D__TOOLKIT_MAT_FILE_USE_ZLIB__
  endif
  ifeq ($(PLATFORM), windows)
    GUI_LIBS += -Wl,-s -Wl,-subsystem,windows -lmingw32 \
      -lqtmain$(D) -lqwindows$(D) -lQt5EventDispatcherSupport$(D) \
      -lQt5FontDatabaseSupport$(D) -lqtfreetype$(D) -lQt5ThemeSupport$(D) \
      -lQt5Gui$(D) -lQt5Core$(D) \
      -ldwmapi -loleaut32 -limm32 -lopengl32 -lgdi32 -liphlpapi -lversion \
      -lws2_32 -luuid -lole32 -lwinmm
    GUI_DEF += -DUNICODE -D_UNICODE -DQT_NEEDS_QMAIN
    CFLAGS += -mthreads
    CCFLAGS += -fno-keep-inline-dllexport -fexceptions
    CCFLAGS += -Wa,-mbig-obj
  endif
  CCFLAGS += $(GUI_DEF) $(GUI_INC) -I../qt -Iqt
else
  # wxWidgets.
  CC := $(shell $(WXCONFIG) --cc)
  CXX := $(shell $(WXCONFIG) --cxx)
  RESCOMP := $(shell $(WXCONFIG) --rescomp)
  ifeq ($(PLATFORM), windows)
    CCFLAGS += -Wno-unused-local-typedefs -Wno-inconsistent-missing-override
    OBJ += rc.o
    UNDEF_X11_H =
  else
    ifeq ($(PLATFORM), linux)
      CCFLAGS += -Wno-unused-local-typedefs
      UNDEF_X11_H = undef_x11.h
    else
      CCFLAGS += -Wno-inconsistent-missing-override
      UNDEF_X11_H =
    endif
  endif
  CCFLAGS += $(shell $(WXCONFIG) --cxxflags) -D__TOOLKIT_WXWINDOWS__=1
  GUI_LIBS = $(shell $(WXCONFIG) --libs core,base,gl,aui,net $(WX_LIBS_SED))
endif

ifdef MATLAB_INC
  ENGINE_LIBS = libmx.a libeng.a
  ENGINE_EXTRA_CFLAGS = -DMATLAB_ENGINE_AVAILABLE
  OBJ += matlab_engine.o
endif

# More compiler configuration.
CFLAGS += -I. -I.. -Werror -Wall -MMD -Wno-sign-compare
CCFLAGS += $(ENGINE_EXTRA_CFLAGS) $(ZLIB_CFLAGS) \
  -D__TOOLKIT_USE_CERES__ \
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
  -DANSI_DECLARATORS -DEXTERNAL_TEST -fwrapv -Wno-uninitialized

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
.PRECIOUS: %.h.gch qt/moc_%.cc qt/ui_%.h

CC_COMPILE = $(CXX) -c $(CCFLAGS) -o $@ $<
C_COMPILE = $(CC) -c $(CFLAGS) -o $@ $<

%.o: ../%.cc $(PCH) $(UNDEF_X11_H)
	$(CC_COMPILE)

dialogblocks/%.o: ../dialogblocks/%.cc $(PCH)
	$(CC_COMPILE)

qt/%.o: ../qt/%.cc $(UI_HEADERS) $(PCH)
	$(CC_COMPILE)

qt/%.o: qt/%.cc $(UI_HEADERS) $(PCH)
	$(CC_COMPILE)

toolkit/%.o: ../../toolkit/%.cc $(PCH) $(UNDEF_X11_H)
	$(CC_COMPILE)

%.o: ../%.c
	$(C_COMPILE)

lua/%.o: $(LUA_DIR)/%.c
	$(C_COMPILE)

%.h.gch: %.h $(UNDEF_X11_H)
	$(CXX) $(CCFLAGS) $<

qt/moc_%.cc: ../qt/%.h
	$(QT_DIR)/bin/moc $(GUI_DEFS) $< -o $@

qt/moc_%.cc: ../../toolkit/%.h
	$(QT_DIR)/bin/moc $(GUI_DEFS) $< -o $@

qt/ui_%.h: ../qt/%.ui
	$(QT_DIR)/bin/uic $< -o $@

rc.o: ../dialogblocks/rama.rc
	$(RESCOMP) -i $< -o $@

# The sed command is required for wxWidgets on windows because wx-config seems
# not to get the proper library names in version 3.0.1.
$(APP): $(OBJ) $(LUA_OBJ) $(ENGINE_LIBS)
	$(CXX) $(CCFLAGS) $(LDFLAGS) -o $(APP) $(WINDOWS_STATIC_LINK) \
            $(OBJ) $(LUA_OBJ) $(ENGINE_LIBS) $(CERES_LIB) \
            $(ARPACK_LIB) $(EIGEN_BLAS_LAPACK_LIB) $(GUI_LIBS)

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
