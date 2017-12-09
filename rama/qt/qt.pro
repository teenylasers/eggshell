#-------------------------------------------------
#
# Project created by QtCreator 2017-11-01T19:31:09
#
#-------------------------------------------------

# Include other files.
include(qt.pri)

# Differences between debug and release builds.
CONFIG(release, debug|release) {
  # Release mode.
  LIBS += -lceres-opt
  DEFINES += NDEBUG
  DEFINES += EIGEN_NO_DEBUG
}
CONFIG(debug, debug|release) {
  # Debug mode.
  LIBS += -lceres-dbg
  # Note that _GLIBCXX_DEBUG must be defined consistently in all files and
  # libraries that use STL, as it changes the layout of STL data structures.
  DEFINES += _GLIBCXX_DEBUG
}

# Libraries.
LIBS += -L$${CERES_LIB_DIR} -L$${ARPACK_LIB_DIR} -L$${EIGEN_BLAS_LAPACK_LIB_DIR}
LIBS += -lz
LIBS += -larpack
LIBS += -lBlasAndLapack

# Defines.
# -DEIGEN_DEFAULT_DENSE_INDEX_TYPE=int reduces sparse matrix memory usage in
# 64 bit builds, as the default is ptrdiff_t. IMPORTANT NOTE: This flag
# affects the Eigen ABI without actually altering the mangled names of C++
# symbols. If two source files use different values of this flag then they may
# emit incompatible eigen code. In optimized builds things will be inlined and
# this may not matter. In debug builds there could be multiple incompatible
# versions of each function generated, all but one of which will be stripped
# out by the linker, meaning some code will link with incorrect eigen
# functions. Therefore we define a single value of this flag here, for all to
# use.
DEFINES += EIGEN_DEFAULT_DENSE_INDEX_TYPE=int
DEFINES += __TOOLKIT_USE_CERES__
DEFINES += __TOOLKIT_MAT_FILE_USE_ZLIB__

# Include paths.
INCLUDEPATH += $$EIGEN_DIR
INCLUDEPATH += ../lua-5.3.0/src
INCLUDEPATH += $$CERES_INC

# Compiler flags.
QMAKE_CXXFLAGS_WARN_ON += -Wno-sign-compare -Wno-unused-parameter

# Cleanup extra things.
QMAKE_CLEAN += -r text2bin.exe* user_script_util.c *.o *.app

# QT configuration.
QT += core gui network
CONFIG -= debug_and_release
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
TARGET = Rama
TEMPLATE = app

# It's an error if we use Qt features deprecated before Qt 6.0.0.
DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000

SOURCES += \
        main.cc \
        main_window.cc \
    about.cc \
    sweep.cc \
    error_window.cc \
    ../../toolkit/error.cc \
    ../../toolkit/viewer.cc \
    ../../toolkit/camera.cc \
    ../../toolkit/gl_utils.cc \
    ../../toolkit/shaders.cc \
    ../../toolkit/colormaps.cc \
    ../../toolkit/lua_util.cc \
    ../../toolkit/md5.cc \
    ../../toolkit/optimizer.cc \
    ../../toolkit/thread.cc \
    ../../toolkit/testing.cc \
    ../../toolkit/lua_vector.cc \
    ../../toolkit/trace.cc \
    ../../toolkit/mystring.cc \
    ../../toolkit/mat_file.cc \
    ../../toolkit/gl_font.cc \
    ../../toolkit/plot.cc \
    ../../toolkit/plot_gui.cc \
    ../../toolkit/femsolver.cc \
    ../common.cc \
    ../shape.cc \
    ../mesh.cc \
    ../solver.cc \
    ../clipper.cc \
    ../triangle.c \
    ../../toolkit/lua_model_viewer_qt.cc \
    ../cavity_qt.cc \
    ../../toolkit/crash_handler.cc

SOURCES += $${LUA_SOURCES}
OBJECTS += user_script_util.o

HEADERS += \
        main_window.h \
    about.h \
    sweep.h \
    error_window.h \
    ../../toolkit/lua_model_viewer_qt.h

FORMS += \
        main_window.ui \
    about.ui \
    sweep.ui \
    error_window.ui
