# Things and also stuff

For rama documentation refer to [ramasimulator.org](http://ramasimulator.org).

## Build instructions (all platforms)

Apologies in advance for the messy manual build process. This is mainly due to
the number of 3rd party libraries and tools that are used, and in particular
to the need to patch many of those things. Build automation would be nice!

* Prerequisites for OSX:
  - Install Xcode. The Xcode gcc is used to build everything here.
    Using another gcc (e.g. from MacPorts) seems to cause hard-to-debug binary
    compatibility problems.
* Prerequisites for Windows, see below.
* Download this "stuff" repository. We will assume below that the repository
  is installed into `~/stuff`.
* Create somewhere a directory into which you will put downloaded source
  for the various third party libraries that are used. The default that is
  assumed by the make files is `~/tools`. If you don't use that default,
  set an environment variable `TOOLS_DIR` to the directory you use (this
  will be read by `common.mk`).
* Download and unpack the following into `~/tools`
  - arpack-ng: `git clone https://github.com/opencollab/arpack-ng.git`
  - ceres-solver-1.13.0: `wget http://ceres-solver.org/ceres-solver-1.13.0.tar.gz`
  - eigen-3.3.8: `https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.tar.gz`.
    Unpack and rename the directory to `eigen-3.3.8`.
  - Qt5. `git clone git://code.qt.io/qt/qt5.git`
  - lapack-3.7.1: `wget http://www.netlib.org/lapack/lapack-3.7.1.tgz`
* Adjust the variables in `~/stuff/config.mk` if your library versions differ
  from those above.
* Build arpack-ng
  - In `~/tools` run `patch -p0 < ~/stuff/toolkit/arpack-build/arpack_patch.txt`
  - In `~/stuff/toolkit/arpack-build` run `make` to create the `libarpack.a` library.
* Build ceres:
  - In `~/tools` run `patch -p0 < ~/stuff/toolkit/ceres-build/ceres-patch.txt`
  - In `~/stuff/toolkit/ceres-build` run `make -j8` then `make clean` to create the
    `libceres-opt.a` library.
    Change `OPTIMIZE` to 0 in the `Makefile` then run `make -j8` then `make clean`
    to create the `libceres-dbg.a` library.
* Build the eigen version of lapack:
  - In `~/stuff/toolkit/eigen-blas-lapack-build` run `make` and then `make clean`
* Build Qt. Refer to the [Qt building-from-source](https://wiki.qt.io/Building_Qt_5_from_Git#Getting_the_source_code) guide.
  The initial steps are:
```
  cd ~/tools/qt5
  git checkout 5.12
  perl init-repository --module-subset=default,-qtwebengine
  mkdir ~/tools/qt5_build ~/tools/qt5_install
  cd ~/tools/qt5_build
```
  The configuration depends on the platform. For Linux:
```
  ~/tools/qt5/configure -prefix ~/tools/qt5_install -static -opensource -confirm-license -debug-and-release -opengl desktop -nomake examples -nomake tests -qt-xcb
```
  For OSX:
```
  ~/tools/qt5/configure -prefix ~/tools/qt5_install -static -opensource -confirm-license -debug-and-release -opengl desktop -nomake examples -nomake tests -platform macx-clang QMAKE_MACOSX_DEPLOYMENT_TARGET=10.13
```
  For Windows, see the extra instructions below.
  Finally build and install with:
```
  make -j8
  make install
```
* Build rama.
  - cd to `~/stuff/rama`
  - In the `Makefile` select an optimized build (1) or a debug build (0).
  - Run `make -j8`.

------------------------------

## Build instructions (Windows)

This is a little more annoying than on mac/linux.
We use the `x86_64-w64-mingw32` toolchain under the MSYS2 environment.
We don't use Visual C++ - even though that would compile much faster, it would
require a lot of Visual C++-specific modifications to the code.

The procedure:
* Install [MSYS2](https://www.msys2.org/)
* From the MSYS2 shell, install some extra packages that are required:
```
pacman -Syu
pacman -S --needed base-devel mingw-w64-i686-toolchain mingw-w64-x86_64-toolchain
pacman -S git patch
```
* Install [Strawberry Perl](http://strawberryperl.com/).
* Install [Inno Setup](https://jrsoftware.org/isinfo.php) if you plan to make
  Windows installers.
* Add to your path: `/c/Strawberry/perl/bin` and `/mingw64/bin`.
  Remove any other Perl distributions from your path. Here is what I do in my
  `.bashrc`:
```
export PATH=`echo $PATH | sed 's/[^:]*[Pp]erl[^:]*//g' | sed 's/::*/:/g' | sed 's/:$//'`
export PATH="/mingw64/bin:/c/Strawberry/perl/bin:$PATH"
```
* Set the following environment variable in the MSYS2 shell:
```
export TOOLS_DIR="/c/where/you/put/the/tools"
```
* The third party libraries can be downloaded and built with the instructions
  above, but Qt requires a different configuration:
```
cd $TOOLS_DIR
mkdir qt5_build ; cd qt5_build
../qt5/configure -prefix $TOOLS_DIR/qt5_install -platform win32-g++ -static -make-tool make -opensource -release -no-icu -no-openssl -opengl desktop -no-angle -skip qtwebengine -skip qtlocation -skip qtmultimedia -skip qtdeclarative -nomake examples -nomake tests -confirm-license QMAKE_LFLAGS_CONSOLE+=-static-libstdc++
```
