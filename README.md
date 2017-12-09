# Things and also stuff

For rama documentation refer to http://ramasimulator.org

## Build instructions

Apologies in advance for the messy manual build process. This is mainly due to
the number of 3rd party libraries and tools that are used, and in particular
to the need to patch many of those things. Build automation would be nice!

* Download the "stuff" repository. We will assume below that the repository
  is installed into `~/stuff`.
* On OSX:
  - Install Xcode. The Xcode gcc is used to build everything here.
    Using another gcc (e.g. from MacPorts) seems to cause hard-to-debug binary
    compatibility problems.
* Create somewhere a directory into which you will put downloaded source.
  The default is `~/tools`. If you don't use the default, set an environment
  variable `TOOLS_DIR` to the directory you use (this will be read by
  `common.mk`).
* Download and unpack the following into `~/tools`
  - arpack-ng: `git clone https://github.com/opencollab/arpack-ng.git`
  - ceres-solver-1.13.0: `wget http://ceres-solver.org/ceres-solver-1.13.0.tar.gz`
  - eigen-3.3.4: `wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2`
    Rename the unpack directory to `eigen-eigen-3.3.4`.
  - wxWidgets-3.1.0: `wget https://github.com/wxWidgets/wxWidgets/releases/download/v3.1.0/wxWidgets-3.1.0.tar.bz2`
  - lapack-3.7.1: `wget http://www.netlib.org/lapack/lapack-3.7.1.tgz`
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
* Build wxWidgets:
  - In `~/tools/wxWidgets-3.1.0` and patch with `patch -p1 -l < ~/stuff/rama/wxwidgets_patches.txt`
  - Copy `~/tools/wxWidgets-3.1.0` to `~/tools/wxWidgets-3.1.0-opt` (for the optimised build)
    and `~/tools/wxWidgets-3.1.0-dbg` (for the debug build).
  - In `~/tools/wxWidgets-3.1.0-opt` run
    `./configure --disable-shared --disable-compat28 --with-opengl --enable-http --disable-mediactrl` (on the mac add the option `--with-macosx-version-min=10.9`).
    Then run `make -j8`
  - In `~/tools/wxWidgets-3.1.0-dbg` run
    `./configure --disable-shared --disable-compat28 --with-opengl --enable-http --disable-mediactrl --enable-debug` (on the mac add the option `--with-macosx-version-min=10.9`).
    Then run `make -j8`
* Build rama.
  - cd to `~/stuff/rama`
  - In the `Makefile` select an optimized build (1) or a debug build (0).
  - Run `make`.
