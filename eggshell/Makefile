
#############################################################################
# Top level configuration

# OPTIMIZE is 1 for optimizations on, or 0 for debug features on.
OPTIMIZE := 1

default: all
-include ../common.mk       # For top level invocation
-include ../../common.mk    # For invocation in the build directory

CFLAGS += -fsanitize=address
CCFLAGS += -fsanitize=address

#############################################################################
# Rules that can be run from the top level directory (all other rules need
# to be run from the build directory).

# Make sure that any failures in piped commands are recognized.
SHELL=/bin/bash -o pipefail

# Build directories
DIRS = build/toolkit build/qt

# -R means no-builtin-variables and no-builtin-rules:
MAKEFLAGS += -R
MY_MAKEFLAGS = -C build -f ../Makefile

.PHONY: dirs all run test clean

dirs:
	mkdir -p $(DIRS)

all: dirs
	$(MAKE) $(MY_MAKEFLAGS) $(APP)
	@echo Success

test: dirs
	$(MAKE) $(MY_MAKEFLAGS) unittest

clean:
	-rm -rf build

#############################################################################
# Application files (including single-file libraries).

OBJ = body.o \
      contact.o \
      eggshell_view.o \
      ensembles.o \
      joints.o \
			lcp.o \
      model.o \
			sparse_iterations.o	\
			sparse_iterations_utils.o \
      utils.o \
      collision.o \
      toolkit/error.o \
      toolkit/viewer.o \
      toolkit/gl_utils.o \
      toolkit/camera.o \
      toolkit/mystring.o \
			toolkit/random.o \
      toolkit/shaders.o \
      toolkit/testing.o \
      qt/main.o \
      qt/main_window.o  \
      qt/error_window.o  \
      qt/moc_main_window.o \
      qt/moc_error_window.o \
      qt/plugin_import.o

UI_HEADERS = qt/ui_main_window.h qt/ui_error_window.h

APP = Eggshell

#############################################################################
# Compiler configuration: directories, files and flags.

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

CFLAGS += -mmacosx-version-min=10.15 -DGL_SILENCE_DEPRECATION

GUI_LIBS += -L$(QT_DIR)/lib -L$(QT_DIR)/plugins/platforms -L$(QT_DIR)/plugins/styles \
  -L$(QT_DIR)/plugins/imageformats \
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

# More compiler configuration.
CFLAGS += -I. -I.. -Werror -Wall -MMD -Wno-sign-compare -I../../toolkit $(EIGEN_FLAGS)

$(OBJ): CCFLAGS += -I$(EIGEN_DIR)

#############################################################################
# Rules.

.SUFFIXES:
.PRECIOUS: qt/moc_%.cc qt/ui_%.h

CC_COMPILE = $(CXX) -c $(CCFLAGS) -o $@ $<
C_COMPILE = $(CC) -c $(CFLAGS) -o $@ $<

%.o: ../%.cc
	$(CC_COMPILE)

qt/%.o: ../qt/%.cc $(UI_HEADERS)
	$(CC_COMPILE)

qt/%.o: qt/%.cc $(UI_HEADERS)
	$(CC_COMPILE)

toolkit/%.o: ../../toolkit/%.cc
	$(CC_COMPILE)

%.o: ../%.c
	$(C_COMPILE)

qt/moc_%.cc: ../qt/%.h
	$(QT_DIR)/bin/moc $(GUI_DEFS) $< -o $@

qt/moc_%.cc: ../../toolkit/%.h
	$(QT_DIR)/bin/moc $(GUI_DEFS) $< -o $@

qt/ui_%.h: ../qt/%.ui
	$(QT_DIR)/bin/uic $< -o $@

$(APP): $(OBJ)
	$(CXX) $(CCFLAGS) $(LDFLAGS) -lglog -o $(APP) $(OBJ) $(GUI_LIBS)

#############################################################################
# Files to run tests on

TESTS = toolkit/testing.o \
  toolkit/random.o \
	toolkit/error.o \
	utils.o \
	body.o \
	collision.o \
	contact.o \
	eggshell_view.o \
	ensembles.o \
	joints.o \
	lcp.o \
  sparse_iterations_utils.o \
	sparse_iterations.o

# TODO: fix
# $(TESTS): CCFLAGS += -D__TOOLKIT_DEFINE_TESTING_MAIN__

unittest: $(TESTS)
	$(CXX) $(CCFLAGS) -I$(EIGEN_DIR) $(LDFLAGS) -o test $(TESTS)
	./test
	rm -rf ./test

# Include dependencies (only works from build directory).
-include *.d $(DIRS:build/%=%/*.d)
