# Things and also stuff

## Build instructions

Apologies in advance for the messy manual build process. This is mainly due to
the number of 3rd party libraries and tools that are used, and in particular
to the need to patch many of those things. Build automation would be nice!

* Create somewhere a directory into which you will put downloaded source.
  Let's call it `/tools`.
* Download and unpack the following into `/tools`
  - arpack-ng: `git clone https://github.com/opencollab/arpack-ng.git`
  - ceres-solver-1.13.0: `wget http://ceres-solver.org/ceres-solver-1.13.0.tar.gz`
  - eigen-eigen-3.3.4: `wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2`
  - fftw-3.3.6: `wget http://www.fftw.org/fftw-3.3.6-pl2.tar.gz`
  - lapack-3.7.1: `wget http://www.netlib.org/lapack/lapack-3.7.1.tgz`
  - readline-6.3: `wget https://ftp.gnu.org/gnu/readline/readline-6.3.tar.gz`
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
  - In `git/stuff/toolkit/eigen-blas-lapack-build` run `make`
* @@@ Build instructions for
  - fftw
  - lapack
  - readline
  - wxWidgets

