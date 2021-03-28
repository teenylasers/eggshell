
all: libarpack.a

OPTIMIZE := 1
include ../../common.mk

# Note that we can reduce the number of LAPACK source files as Eigen implements
# more of the LAPACK API.
FORTRAN_SRC = \
  $(ARPACK_DIR)/SRC/dsaitr.f \
  $(ARPACK_DIR)/SRC/dsapps.f \
  $(ARPACK_DIR)/SRC/dsaup2.f \
  $(ARPACK_DIR)/SRC/dsaupd.f \
  $(ARPACK_DIR)/SRC/dsconv.f \
  $(ARPACK_DIR)/SRC/dseupd.f \
  $(ARPACK_DIR)/SRC/dsortr.f \
  $(ARPACK_DIR)/SRC/dstats.f \
  $(ARPACK_DIR)/SRC/dsgets.f \
  $(ARPACK_DIR)/SRC/dsesrt.f \
  $(ARPACK_DIR)/SRC/dseigt.f \
  $(ARPACK_DIR)/SRC/dstqrb.f \
  $(ARPACK_DIR)/SRC/dgetv0.f \
  $(LAPACK_DIR)/SRC/dlaruv.f \
  $(LAPACK_DIR)/SRC/dlarnv.f \
  $(LAPACK_DIR)/SRC/dlaisnan.f \
  $(LAPACK_DIR)/SRC/disnan.f \
  $(LAPACK_DIR)/SRC/dlae2.f \
  $(LAPACK_DIR)/SRC/dlaev2.f \
  $(LAPACK_DIR)/SRC/dlassq.f \
  $(LAPACK_DIR)/SRC/dlanst.f \
  $(LAPACK_DIR)/SRC/dlarfg.f \
  $(LAPACK_DIR)/SRC/dlasr.f \
  $(LAPACK_DIR)/SRC/dlapy2.f \
  $(LAPACK_DIR)/SRC/dlapy3.f \
  $(LAPACK_DIR)/SRC/dlasrt.f \
  $(LAPACK_DIR)/SRC/dsteqr.f \
  $(LAPACK_DIR)/SRC/iladlr.f \
  $(LAPACK_DIR)/SRC/iladlc.f \
  $(LAPACK_DIR)/SRC/dgeqr2.f \
  $(LAPACK_DIR)/SRC/dlacpy.f \
  $(LAPACK_DIR)/SRC/dlarf.f \
  $(LAPACK_DIR)/SRC/dlartg.f \
  $(LAPACK_DIR)/SRC/dlascl.f \
  $(LAPACK_DIR)/SRC/dlaset.f \
  $(LAPACK_DIR)/SRC/dorm2r.f \
  $(LAPACK_DIR)/INSTALL/dlamch.f \
  dvout_dummy.f \
  ivout_dummy.f

# Add -fcheck=all to the command line here to enable bounds checking and other
# things.
libarpack.a: clean util.o
	$(FORTRAN_COMPILER) -c -O2 $(FFLAGS) $(FORTRAN_SRC)
	ar rcv $@ *.o
	rm -f *.o

util.o: util.cc
	$(TOOL_PREFIX)g++ -Wall -Werror $(CFLAGS) -c $<

clean:
	rm -f *.o *.a
