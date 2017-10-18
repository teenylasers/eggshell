# Things and also stuff

For rama documentation refer to http://ramasimulator.org

## Build instructions

Apologies in advance for the messy manual build process. This is mainly due to
the number of 3rd party libraries and tools that are used, and in particular
to the need to patch many of those things. Build automation would be nice!

* On OSX:
  - Install Xcode. The Xcode gcc is used to build everything here.
    Using another gcc (e.g. from MacPorts) seems to cause hard to debug binary
    compatibility problems.
* Create somewhere a directory into which you will put downloaded source.
  Let's call it `/tools`.
* Download and unpack the following into `/tools`
  - arpack-ng: `git clone https://github.com/opencollab/arpack-ng.git`
  - ceres-solver-1.13.0: `wget http://ceres-solver.org/ceres-solver-1.13.0.tar.gz`
  - eigen-eigen-3.3.4: `wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2`
  - wxWidgets-3.1.0: `wget https://github.com/wxWidgets/wxWidgets/releases/download/v3.1.0/wxWidgets-3.1.0.tar.bz2`
* In `common.mk`, change all instances of `SET_ME` to match your local
  configuration. Many of the paths will point to somewhere in `/tools`.
* Build arpack-ng
  - In `/tools` run `patch -p0 < git/stuff/toolkit/arpack-build/arpack_patch.txt`
  - In `git/stuff/toolkit/arpack-build` run `make` to create the `libarpack.a` library.
* Build ceres:
  - In `/tools` run `patch -p0 < git/stuff/toolkit/ceres-build/ceres-patch.txt`
  - In `git/stuff/toolkit/ceres-build` run `make` then `make clean` to create the
    `libceres-opt.a` library.
* Build the eigen version of lapack:
  - In `git/stuff/toolkit/eigen-blas-lapack-build` run `make` and then `make clean`
* Build wxWidgets:
  - In `/tools/wxWidgets-3.1.0` patch with `patch -p1 -l < git/stuff/rama/wxwidgets_patches.txt`
  - Copy `/tools/wxWidgets-3.1.0` to `/tools/wxWidgets-3.1.0-opt` (for the optimised build)
    and `/tools/wxWidgets-3.1.0-dbg` (for the debug build).
  - In `/tools/wxWidgets-3.1.0-opt` run
    `./configure --disable-shared --disable-compat28 --with-opengl --enable-http --with-macosx-version-min=10.9 --disable-mediactrl`
    then run `make -j8`
  - In `/tools/wxWidgets-3.1.0-dbg` run
    `./configure --disable-shared --disable-compat28 --with-opengl --enable-http --with-macosx-version-min=10.9 --disable-mediactrl --enable-debug`
    then run `make -j8`
* Build rama.
  - cd to `git/stuff/rama`
  - In the `Makefile` select an optimized build (1) or a debug build (0).
  - Run `make`.
