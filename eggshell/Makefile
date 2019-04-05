
#############################################################################
# Top level configuration

# OPTIMIZE is 1 for optimizations on, or 0 for debug features on.
OPTIMIZE := 1

default: all
-include ../common.mk       # For top level invocation
-include ../../common.mk    # For invocation in the build directory

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

.PHONY: dirs all run clean

dirs:
	mkdir -p $(DIRS)

all: dirs
	$(MAKE) $(MY_MAKEFLAGS) $(APP)
	@echo Success

clean:
	-rm -rf build

#############################################################################
# Application files (including single-file libraries).

OBJ = eggshell_view.o \
      model.o \
      collision.o \
      toolkit/error.o \
      toolkit/viewer.o \
      toolkit/gl_utils.o \
      toolkit/camera.o \
      toolkit/mystring.o \
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

GUI_LIBS = -L$(QT_DIR)/lib -L$(TOOLS_DIR)/Qt-5.9.2/plugins/platforms \
  -L$(TOOLS_DIR)/Qt-5.9.2/plugins/imageformats \
  -lQt5Widgets$(D) -lQt5Gui$(D) -lQt5Core$(D) -lQt5Network$(D) \
  -lqtlibpng$(D) -lqtharfbuzz$(D) -lqtpcre2$(D) -lqtfreetype$(D) \
  -lQt5FontDatabaseSupport$(D) -lQt5ThemeSupport$(D) -lqjpeg
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

# More compiler configuration.
CFLAGS += -I. -I.. -Werror -Wall -MMD -Wno-sign-compare -I../../toolkit

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
	$(CXX) $(CCFLAGS) $(LDFLAGS) -o $(APP) $(OBJ) $(GUI_LIBS)

# Include dependencies (only works from build directory).
-include *.d $(DIRS:build/%=%/*.d)
