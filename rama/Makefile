# Rama Simulator, Copyright (C) 2014-2020 Russell Smith.
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

#############################################################################
# Top level configuration

# OPTIMIZE is 1 for optimizations on, or 0 for debug features on.
OPTIMIZE := 1

default: all
-include ../common.mk           # For top level invocation
-include ../../common.mk        # For invocation in the build directory

# Where to find the images for the documentation, relative to the build dir.
MEDIA_PATH = ../../../Evenedric.github.io/images

#############################################################################
# Rules that can be run from the top level directory (all other rules need
# to be run from the build directory).

# Make sure that any failures in piped commands are recognized.
SHELL=/bin/bash -o pipefail

# Extract the version number from version.h.
ifndef VERSION
VERSION := $(shell grep __APP_VERSION__ version.h|grep __APP_VERSION__|cut -d '"' -f 2)
endif

# Build directories
DIRS = build/lua build/toolkit
DIRS += build/qt

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
ifeq ($(PLATFORM), windows)
	rm -rf images  # Since 'ln' actually does a copy on MSYS2
endif
	pushd $(MEDIA_PATH) && git pull && popd  # Make sure images up to date
	ln -f -s $(MEDIA_PATH)  # So locally building and viewing the manual works
	$(DOCCER) -Dversion=$(VERSION) $(DOCCER_DICTIONARY) ../doc/rama.doc | sed 's@__MATHJAX__@$(MATHJAX_PUBLIC)@' | sed 's@__TITLE__@Rama@' > rama.html
	$(DOCCER) -Dversion=$(VERSION) -i https://evenedric.github.io/ ../doc/rama.doc | sed 's@__MATHJAX__@$(MATHJAX_PUBLIC)@' | sed 's@__TITLE__@Rama@' > rama_pub.html

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
	./build/$(APP) -test tests/test_regressions.lua -test_number=1
	./build/$(APP) -test tests/test_regressions.lua -test_number=2
	@echo
	@echo '*** Success ***'

windeploy: all test
	$(MAKE) $(MY_MAKEFLAGS) windeploy2
windeploy2: $(APP) docs2
	strip $(APP)
	# Note two forward slashes escape to a single / in MSYS2:
	$(INNO_SETUP) //cc '..\release\rama_setup.iss'
	echo $(VERSION) > latest_version.txt
	scp setup_rama.exe latest_version.txt $(RAMASIMULATORUSER)@ramasimulator.org:public_html/ramasimulator

# Only the mac deployment copies the documentation to ramasimulator.org.
macdeploy: all test
	$(MAKE) $(MY_MAKEFLAGS) macdeploy2
macdeploy2: $(APP) docs2
	strip $(APP)
	rm -rf Rama.app && cp -av ../release/Rama.app .
	mkdir Rama.app/Contents/MacOS
	cp $(APP) Rama.app/Contents/MacOS
	cp -av ../examples Rama.app
	cp rama.html Rama.app/Contents/Resources
	cp -av $(MEDIA_PATH) Rama.app/Contents/Resources
	hdiutil create -volname Rama -srcfolder Rama.app -ov -format UDZO Rama.dmg
	echo $(VERSION) > latest_version_mac.txt
	scp Rama.dmg rama_pub.html latest_version_mac.txt $(RAMASIMULATORUSER)@ramasimulator.org:public_html/ramasimulator

linuxdeploy: all test
	$(MAKE) $(MY_MAKEFLAGS) linuxdeploy2
linuxdeploy2: $(APP) docs2
	strip $(APP)
	rm -rf rama-* && mkdir rama-$(VERSION)
	cp -av $(APP) ../examples $(MEDIA_PATH) rama.html rama-$(VERSION)
	echo $(VERSION) > latest_version_linux.txt
	tar cfvz rama-$(VERSION).tgz rama-$(VERSION)
	scp rama-$(VERSION).tgz latest_version_linux.txt $(RAMASIMULATORUSER)@ramasimulator.org:public_html/ramasimulator

cleanapp:
	-rm -f build/*.d build/*.o build/$(APP) $(subst ../,,$(PCH)) *.d

clean: cleanapp
	-rm -rf build

#############################################################################
# Application files (including single-file libraries).

OBJ = toolkit/error.o toolkit/colormaps.o toolkit/mat_file.o \
      toolkit/optimizer.o toolkit/testing.o toolkit/md5.o toolkit/lua_util.o \
      toolkit/gl_utils.o toolkit/gl_font.o toolkit/plot.o toolkit/plot_gui.o \
      toolkit/viewer.o toolkit/camera.o toolkit/femsolver.o toolkit/mystring.o \
      toolkit/lua_vector.o toolkit/trace.o \
      toolkit/shaders.o \
      common.o shape.o mesh.o solver.o \
      clipper.o triangle.o user_script_util.o license_text.o

OBJ += toolkit/thread.o cavity_qt.o qt/main.o qt/main_window.o qt/about.o \
       qt/sweep.o qt/error_window.o toolkit/lua_model_viewer_qt.o \
       qt/moc_main_window.o qt/moc_about.o  qt/moc_sweep.o \
       qt/moc_error_window.o qt/moc_lua_model_viewer_qt.o \
       qt/plugin_import.o
UI_HEADERS = qt/ui_about.h qt/ui_error_window.h qt/ui_main_window.h \
             qt/ui_sweep.h

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
ifeq ($(PLATFORM), linux)
  CCFLAGS += -fPIC
  CFLAGS += -fPIC
endif

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
ifeq ($(PLATFORM), linux)
  GUI_LIBS = -Wl,-Bstatic -static-libgcc -static-libstdc++ -Wl,--start-group
endif
GUI_LIBS += -L$(QT_DIR)/lib -L$(QT_DIR)/plugins/platforms -L$(QT_DIR)/plugins/styles \
  -lQt5Widgets$(D) -lQt5Gui$(D) -lQt5Core$(D) -lQt5Network$(D) \
  -lqtharfbuzz$(D) -lqtpcre2$(D) \
  -lQt5FontDatabaseSupport$(D) -lQt5ThemeSupport$(D)
ifeq ($(PLATFORM), osx)
  GUI_LIBS += -lqtlibpng$(D) -lqtfreetype$(D) -lqcocoa$(D) -lcups -lz -lQt5PrintSupport$(D) \
    -lQt5ClipboardSupport$(D) -lQt5AccessibilitySupport$(D) \
    -lQt5GraphicsSupport$(D) -lqmacstyle$(D) \
    -framework OpenGL -framework AGL -framework IOKit \
    -framework Carbon -framework AppKit -framework SystemConfiguration \
    -framework Security \
    -framework IOSurface -framework Metal -framework QuartzCore
  CCFLAGS += -D__TOOLKIT_MAT_FILE_USE_ZLIB__
endif
ifeq ($(PLATFORM), windows)
  GUI_LIBS += -Wl,-s -Wl,-subsystem,windows -lmingw32 \
    -lqtlibpng$(D) -lqtfreetype$(D) -lqtmain$(D) -lqwindows$(D) -lQt5EventDispatcherSupport$(D) \
    -lQt5FontDatabaseSupport$(D) -lqtfreetype$(D) -lQt5ThemeSupport$(D) \
    -lQt5Gui$(D) -lQt5Core$(D) -lQt5WindowsUiAutomationSupport$(D) \
    -ldwmapi -loleaut32 -limm32 -lopengl32 -lgdi32 -liphlpapi -lversion \
    -lws2_32 -luuid -lole32 -lwinmm -lnetapi32 -luserenv -lwtsapi32 -lz
  GUI_DEF += -DUNICODE -D_UNICODE -DQT_NEEDS_QMAIN
  CFLAGS += -mthreads
  CCFLAGS += -fno-keep-inline-dllexport -fexceptions
  CFLAGS += -Wno-int-in-bool-context  # For Eigen
  CCFLAGS += -Wa,-mbig-obj
endif
ifeq ($(PLATFORM), linux)
  GUI_LIBS += -lQt5LinuxAccessibilitySupport$(D) -lQt5AccessibilitySupport$(D) \
    -lQt5QuickParticles$(D) -lQt5FontDatabaseSupport$(D) -lQt5Purchasing$(D) \
    -lQt5DBus$(D) -lQt5ServiceSupport$(D) -lQt5ThemeSupport$(D) -lQt5XcbQpa$(D) \
    -lQt5XmlPatterns$(D) -lQt5EdidSupport$(D) -lQt5GlxSupport$(D) \
    -lqxcb -L$(QT_DIR)/plugins/xcbglintegrations -lqxcb-glx-integration \
    -pthread -lICE -lz -licui18n -licudata -licuuc -lpng -lglib-2.0 \
    -lxcb-util -lxcb-shape -lxcb-record -lxcb-randr -lxcb-xinerama -lxcb-static \
    -lxcb-xkb -lxcb-sync -lxkbcommon -lxkbcommon-x11 -lxcb -lxcb-glx -lX11-xcb -lX11 -lXrender \
    -lfontconfig -lfreetype -lSM -lICE -ldbus-1  -lexpat -lXau -lXdmcp -licui18n \
    -Wl,--end-group -Wl,-Bdynamic -lGL -ldl -lsystemd
endif
CCFLAGS += $(GUI_DEF) $(GUI_INC) -I../qt -Iqt

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

ifeq ($(PLATFORM), windows)
  clipper.o: CCFLAGS += -Wno-class-memaccess
  lua/lstring.o: CFLAGS += -Wno-stringop-overflow
  lua/lmathlib.o: CFLAGS += -Wno-overflow  # Suspicious
endif

#############################################################################
# Lua.

LUA_DIR = ../lua-5.4.0/src
LUA_SRC = lapi.c lcode.c lctype.c ldebug.c ldo.c ldump.c lfunc.c lgc.c llex.c \
  lmem.c lobject.c lopcodes.c lparser.c lstate.c lstring.c ltable.c \
  ltm.c lundump.c lvm.c lzio.c \
  lauxlib.c lbaselib.c lcorolib.c ldblib.c liolib.c \
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

$(APP): $(OBJ) $(LUA_OBJ) $(ENGINE_LIBS)
	$(CXX) $(CCFLAGS) $(LDFLAGS) -o $(APP) $(WINDOWS_STATIC_LINK) \
            $(OBJ) $(LUA_OBJ) $(ENGINE_LIBS) $(CERES_LIB) \
            $(ARPACK_LIB) $(EIGEN_BLAS_LAPACK_LIB) $(GUI_LIBS)

text2bin.exe: ../text2bin.cc
	$(TOOL_PREFIX)gcc$(TOOL_SUFFIX) -o $@ $<

user_script_util.o: ../user_script_util.lua text2bin.exe
	./text2bin.exe user_script_util_dot_lua < $< > user_script_util.c
	$(TOOL_PREFIX)gcc$(TOOL_SUFFIX) $(CFLAGS) -c user_script_util.c

license_text.o: ../COPYING text2bin.exe
	./text2bin.exe license_text < $< > license_text.c
	$(TOOL_PREFIX)gcc$(TOOL_SUFFIX) $(CFLAGS) -c license_text.c

# Create an include file that undefines all macros in /usr/include/X11 that
# don't start with an underscore.
undef_x11.h:
	grep -r '[ \t]*#[ \t]*define[ \t][ \t]*[A-Za-z]' /usr/include/X11/ | \
          sed 's/^[^#]*#[ \t]*define[ \t]*\([A-Za-z0-9_]*\).*/#undef \1/' > $@

# Include dependencies (only works from build directory).
-include *.d $(DIRS:build/%=%/*.d)
