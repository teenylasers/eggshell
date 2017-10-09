
ARPACK is a fortran library for computing eigenvalues and eigenvectors of
sparse matrices. Though it dates from 1996 it is still the preferred library
for this application (Matlab uses it, for example). ARPACK-NG is a version of
it with a more up to date build system but with the same numerical engine. We
want to use this library on OS-X, and with Eigen, and with Xcode gcc 4.2. This
proves to be a little challenging:

* ARPACK is written in fortran, Xcode doesn't include a fortran compiler.
* With macports we can install gcc-mp-4.9, which includes a fortran compiler,
  but this has various subtle incompatibilities with Xcode gcc 4.2.
* With macports we can install libarpack.a directly, but this also does not
  seem compatible with Xcode gcc 4.2.
* f2c is not compatible with various intrinsics used by LAPACK and ARPACK.

The solution used here is to pick just the .f files we need from ARPACK and
LAPACK and compile them with gcc-mp-4.9. To avoid the compatibility issues we
can't compile any files that need I/O or other intrinsics. This is tricky in
only two cases: dsaupd.f is modified to not write output, and a local version
of arscnd_ is provided in util.c.

This library includes only what is necessary to compute eigenvalues and
eigenvectors of double precision sparse symmetric positive definite matrices
(e.g. laplacians).

It is assumed that the source for the libraries is unpacked into:
    arpack-ng-3.1.5  --> $(ARPACK_DIR)
    lapack-3.5.0     --> $(LAPACK_DIR)

Links:
    ARPACK: http://www.caam.rice.edu/software/ARPACK/
    ARPACK-NG: http://forge.scilab.org/index.php/p/arpack-ng/
    LAPACK: http://www.netlib.org/lapack/

A related library implemented in matlab:
    http://ftp.task.gda.pl/site/mathworks-toolbox/matlab/sparfun/speig/

A masters thesis describing the numerical techniques behind ARPACK:
    http://www.ecse.rpi.edu/~rjradke/papers/radkemathesis.pdf
    "A Matlab Implementation of the Implicitly Restarted Arnoldi Method for
     Solving Large-Scale Eigenvalue Problems" by Richard J. Radke.
