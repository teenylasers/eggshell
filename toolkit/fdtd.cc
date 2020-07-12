// Copyright (C) 2014-2020 Russell Smith.
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.

/*

NOTES on PML
------------

The easy part of implementing an FDTD scheme is the Yee cell discretization
and the E,H field updates in free space. The tricky part is the PML boundary,
which is complicated both in its derivation and in the book keeping required
to efficiently implement it. The implementation used here follows the
coordinate stretching / convolutional PML derivation, as described in the
papers below. We set the CPML parameters to compromise between absorbtion of
propagating waves and evanescent fields.

Primers on coordinate stretching PML / CPML (but poor on implementation
details):
  "Notes on Perfectly Matched Layers (PMLs)", Steven G. Johnson.
  http://math.mit.edu/~stevenj/18.369/pml.pdf

  "Understanding the FDTD Method",  John B. Schneider.
  http://www.eecs.wsu.edu/~schneidj/ufdtd/
  http://www.eecs.wsu.edu/~schneidj/ufdtd/chap11.pdf

Discretized implementation of CPML:
  "Convolutional PML (CPML): An Efficient FDTD Implementation of the CFS-PML
  for Arbitrary Media", J. Alan Roden and Stephen D. Gedney.
  http://www.engr.uky.edu/~gedney/roden_gedney_cpml_motl.pdf

  "The convolutional PML for FDTD analysis: Transient electromagnetic
  absorption from DC to daylight", Roden, J.A. and Kramer, T.
  http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6038435

Performance of CPML, how to set the parameters:
  "Performance of convolutional PML absorbing boundary conditions in
  finite-difference time-domain SAR calculations", Ilkka Laakso, Sami Ilvonen
  and Tero Uusitupa.
  http://lib.tkk.fi/Diss/2011/isbn9789526040035/article1.pdf

Coordinate stretching PML:
  "Improved PML for the FDTD solution of wave-structure interaction problems",
  J. P. Berenger.
  http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=558661

Uniaxial PML, for comparison:
  "An anisotropic perfectly matched layer-absorbing medium for the truncation
  of FDTD lattices", S.D. Gedney.
  http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=546249

*/

#define _USE_MATH_DEFINES       // For VC++ math.h to define M_PI and friends

#include <math.h>
#include "thread.h"

#include "fdtd.h"

namespace fdtd {

//***************************************************************************
// Misc.

const int kNumThreads = 2;                      // Number of ParallelFor threads
const double kc = 299792458.130996;             // Speed of light (m/s)
const double kEpsilon = 8.85418782e-12;         // Permittivity of free space
const double kMu = 1.25663706e-6;               // Permeability of free space
const double kZ0 = 376.730313195908;            // Impedance of free space

template <class T> T sqr(T x) {
  return x*x;
}

//***************************************************************************
// PEC.

PEC::PEC() {
  Reset();
}

PEC::PEC(int nx, int ny, int nz) {
  nx_ = nx;
  ny_ = ny;
  nz_ = nz;
  pec_.resize((nx_ + 1)*(ny_ + 1)*(nz_ + 1));
}

void PEC::Reset() {
  nx_ = 0;
  ny_ = 0;
  nz_ = 0;
  pec_.clear();
}

void PEC::Set(Range rx, Range ry, Range rz, bool value) {
  CHECK_MSG(!empty(), "Can't set ranges in empty PEC");
  CHECK_MSG(rx.Valid() && rx.Within(Range(0, nx_ - 1)) &&
            ry.Valid() && ry.Within(Range(0, ny_ - 1)) &&
            rz.Valid() && rz.Within(Range(0, nz_ - 1)), "Invalid ranges");

  const int sx = 1;                     // Array skip in the X direction
  const int sy = (nx_ + 1);             // Array skip in the Y direction
  const int sz = (nx_ + 1) * (ny_ + 1); // Array skip in the Z direction
  for (int z = rz.lo(); z <= rz.hi(); z++) {
    for (int y = ry.lo(); y <= ry.hi(); y++) {
      for (int x = rx.lo(); x <= rx.hi(); x++) {
        pec_[x*sx + y*sy + z*sz] = value;
      }
    }
  }
}

//***************************************************************************
// FDTD computational domain (CD).
//
// We use the Yee cell field representation with nx*ny*nz cubical cells in the
// computational domain (CD). This inconveniently staggers the E and H fields,
// as E field components are stored on the edges of the cells and H field
// components are stored on the faces. The storage approach used here is to
// have an array of (nx+1)*(ny+1)*(nz+1) grid cells where each cell stores
// three E and three H components of a Yee cell, but only for the edges and
// faces with smallest coordinates. This provides good data locality. The
// bottom nx*ny*nz grid cells are actually updated when the fields are stepped,
// the outer skin of cells (three planes on the positive-coordinate faces of
// the CD) always remain at zero, both to reduce the number of special cases in
// the update loops(*) and because they represent the tangential E and normal H
// on those faces of the CD (which are always zero because the CD has a PEC
// boundary). Note that there are also unused field components in this outer
// layer of cells. If a PML is used, that affects the updates in the CD but
// does not change the PEC boundaries and therefore does not change the storage
// scheme.
//
// (*) The outer skin of cells also helps to stitch together separately
//     simulated CDs.

// We define a bunch of macros here for the field stepping code, mainly to help
// ensure that the various coordinate symmetries in that code are preserved.

// Setup E/Hxy as a pointer to a plane of cells with constant z, E/Hx as a
// pointer within that to a line of cells with constant y, and E/H as a pointer
// within that to a cell with constant x.
#define SET_XY_POINTERS_FOR_Z(iz) Field *Exy = E_ + (iz) * sz; \
                                  Field *Hxy = H_ + (iz) * sz;
#define SET_X_POINTERS_FOR_Y(iy)  Field *Ex = Exy + (iy) * sy; \
                                  Field *Hx = Hxy + (iy) * sy;
#define SET_POINTERS_FOR_X(ix)    Field *E = Ex + (ix); \
                                  Field *H = Hx + (ix);

// Coordinate stretch handling logic for inner loops of the field updates. If a
// coordinate is stretched then deltas (differentials) computed in that
// direction will be first-order-filtered using the Psi variables.
#define HANDLE_STRETCH(q, delta1, delta2) \
  if (q##_stretched) { \
    Psi[0] = b##q[i##q] * Psi[0] + a##q[i##q] * delta1; \
    Psi[1] = b##q[i##q] * Psi[1] + a##q[i##q] * delta2; \
    delta1 += Psi[0]; \
    delta2 += Psi[1]; \
    Psi += 2; \
  }

// Compute a delta of the Hb,Hc fields in the coordinate direction 'a', then
// handle any coordinate stretching adjustments along the 'a' direction.
#define COMPUTE_H_DELTAS_AND_HANDLE_STRETCH(a, b, c, Hskip) \
  float D##a##H##b = H->b - H[Hskip].b; \
  float D##a##H##c = H->c - H[Hskip].c; \
  HANDLE_STRETCH(a, D##a##H##b, D##a##H##c)
#define COMPUTE_E_DELTAS_AND_HANDLE_STRETCH(a, b, c) \
  float D##a##E##b = E[s##a].b - E->b; \
  float D##a##E##c = E[s##a].c - E->c; \
  HANDLE_STRETCH(a, D##a##E##b, D##a##E##c)


CD::CD(int nx, int ny, int nz, double dx, double dy, double dz,
       int pml, int pml_depth, int toroid) : ab_(nx, ny, nz) {
  // The Field structure is assumed to have this layout:
  static_assert(sizeof(Field) == 12, "struct Field layout");
  static_assert(offsetof(Field, x) == 0, "struct Field layout");
  static_assert(offsetof(Field, y) == 4, "struct Field layout");
  static_assert(offsetof(Field, z) == 8, "struct Field layout");

  nx_ = nx;
  ny_ = ny;
  nz_ = nz;
  dx_ = dx;
  dy_ = dy;
  dz_ = dz;
  E_ = 0;
  H_ = 0;
  pml_ = pml;
  pml_depth_ = pml_depth;
  toroid_ = toroid;
  ab_dt_ = 0;
  Psi_ = 0;

  // Some checks.
  CHECK_MSG(pml_ >= 0 && pml_ <= PML_ALL, "pml must be an integer 0..PML_ALL");
  CHECK_MSG(((pml_ == 0) ^ (pml_depth_ == 0)) == 0,
            "pml and pml_depth inconsistent");
  int pmlx = (((pml_ & PML_XMIN) != 0) + ((pml_ & PML_XMAX) != 0)) * pml_depth_;
  int pmly = (((pml_ & PML_YMIN) != 0) + ((pml_ & PML_YMAX) != 0)) * pml_depth_;
  int pmlz = (((pml_ & PML_ZMIN) != 0) + ((pml_ & PML_ZMAX) != 0)) * pml_depth_;
  CHECK_MSG(nx_ > pmlx && ny_ > pmly && nz_ > pmlz,
            "PML covers entire CD (this is probably not what you want)");
  CHECK_MSG(((toroid_ & TOROID_X) == 0) ||
            ((pml_ & (PML_XMIN | PML_XMAX)) == 0),
            "X PML and X toroid symmetry is probably not what you want");
  CHECK_MSG(((toroid_ & TOROID_Y) == 0) ||
            ((pml_ & (PML_YMIN | PML_YMAX)) == 0),
            "Y PML and Y toroid symmetry is probably not what you want");
  CHECK_MSG(((toroid_ & TOROID_Z) == 0) ||
            ((pml_ & (PML_ZMIN | PML_ZMAX)) == 0),
            "Z PML and Z toroid symmetry is probably not what you want");

  // Allocate memory.
  int sz = (nx_+1) * (ny_+1) * (nz_+1);
  E_ = new Field[sz];
  H_ = new Field[sz];
  Psi_ = new float[PsiArraySize()];
  memset(E_, 0, sz * sizeof(Field));
  memset(H_, 0, sz * sizeof(Field));
  memset(Psi_, 0, PsiArraySize() * sizeof(float));
}

CD::CD(const CD &cd, bool E_field_only) : ab_(cd.nx_, cd.ny_, cd.nz_) {
  nx_ = cd.nx_;
  ny_ = cd.ny_;
  nz_ = cd.nz_;
  dx_ = cd.dx_;
  dy_ = cd.dy_;
  dz_ = cd.dz_;
  pml_ = cd.pml_;
  pml_depth_ = cd.pml_depth_;
  toroid_ = cd.toroid_;
  ab_dt_ = 0;

  int sz = (nx_+1) * (ny_+1) * (nz_+1);
  E_ = new Field[sz];
  if (E_field_only) {
    H_ = 0;
    Psi_ = 0;
  } else {
    H_ = new Field[sz];
    Psi_ = new float[PsiArraySize()];
  }
  *this = cd;
}

CD::~CD() {
  delete[] E_;
  delete[] H_;
  delete[] Psi_;
}

void CD::operator= (const CD &cd) {
  CHECK_MSG(nx_ == cd.nx_ && ny_ == cd.ny_ && nz_ == cd.nz_ &&
            pml_ == cd.pml_ && pml_depth_ == cd.pml_depth_ &&
            toroid_ == cd.toroid_,
            "Size, PML and symmetry of computational domain must be the same");
  int sz = (nx_+1) * (ny_+1) * (nz_+1);
  if (E_) {
    memcpy(E_, cd.E_, sz * sizeof(Field));
  }
  if (H_) {
    memcpy(H_, cd.H_, sz * sizeof(Field));
  }
  if (Psi_) {
    memcpy(Psi_, cd.Psi_, PsiArraySize() * sizeof(float));
  }
}

float *CD::GetFieldBox(Field *F, int c, Range rx, Range ry, Range rz,
                       int *sx_ret, int *sy_ret, int *sz_ret) const {
  CHECK_MSG(rx.Valid() && rx.Within(Range(0, nx_)) &&
            ry.Valid() && ry.Within(Range(0, ny_)) &&
            rz.Valid() && rz.Within(Range(0, nz_)), "Invalid ranges");
  CHECK_MSG(c >= 0 && c <= 2, "Invalid component");
  const int sx = 1;                      // Array skip in the X direction
  const int sy = (nx_ + 1);              // Array skip in the Y direction
  const int sz = (nx_ + 1) * (ny_ + 1);  // Array skip in the Z direction
  Field *e = F + rx.lo()*sx + ry.lo()*sy + rz.lo()*sz;
  *sx_ret = 3 * sx;
  *sy_ret = 3 * sy;
  *sz_ret = 3 * sz;
  return (&e->x) + c;
}

float *CD::GetEBox(int c, Range rx, Range ry, Range rz,
                   int *sx_ret, int *sy_ret, int *sz_ret) const {
  return GetFieldBox(E_, c, rx, ry, rz, sx_ret, sy_ret, sz_ret);
}

float *CD::GetHBox(int c, Range rx, Range ry, Range rz,
                   int *sx_ret, int *sy_ret, int *sz_ret) const {
  return GetFieldBox(H_, c, rx, ry, rz, sx_ret, sy_ret, sz_ret);
}

float *CD::GetEMatlab(int32_t dims[4]) const {
  if (dims) {
    dims[0] = 3;
    dims[1] = nx_ + 1;
    dims[2] = ny_ + 1;
    dims[3] = nz_ + 1;
  }
  return &E_[0].x;
}

double CD::GetCourantStep() const {
  return (1.0 / kc / sqrt(1.0 / sqr(dx_) + 1.0 / sqr(dy_) + 1.0 / sqr(dz_)));
}

void CD::Step(double dt, bool step_H) {
  StepRanges(dt, step_H, 0, Range(0, nx_-1), Range(0, ny_-1), Range(0, nz_-1));
}

float *CD::StepRanges(double dt, bool step_H, float *starting_Psi,
                      Range rangex, Range rangey, Range rangez) {
  // Function dispatch table for Step*_Helper()s for different combinations of
  // stretching parameters and field type.
  static Step_Helper_fn_t helpers[16] = {
    &CD::StepE_Helper<false, false, false>,
    &CD::StepE_Helper<true,  false, false>,
    &CD::StepE_Helper<false, true,  false>,
    &CD::StepE_Helper<true,  true,  false>,
    &CD::StepE_Helper<false, false, true >,
    &CD::StepE_Helper<true,  false, true >,
    &CD::StepE_Helper<false, true,  true >,
    &CD::StepE_Helper<true,  true,  true >,
    &CD::StepH_Helper<false, false, false>,
    &CD::StepH_Helper<true,  false, false>,
    &CD::StepH_Helper<false, true,  false>,
    &CD::StepH_Helper<true,  true,  false>,
    &CD::StepH_Helper<false, false, true >,
    &CD::StepH_Helper<true,  false, true >,
    &CD::StepH_Helper<false, true,  true >,
    &CD::StepH_Helper<true,  true,  true >
  };

  // Compute 'a' and 'b' arrays for this timestep.
  if (ab_dt_ != dt) {
    ab_dt_ = dt;
    SetupAB(dt, dx_, nx_, 0.0, ab_.axE.data(), ab_.bxE.data());
    SetupAB(dt, dy_, ny_, 0.0, ab_.ayE.data(), ab_.byE.data());
    SetupAB(dt, dz_, nz_, 0.0, ab_.azE.data(), ab_.bzE.data());
    SetupAB(dt, dx_, nx_, 0.5, ab_.axH.data(), ab_.bxH.data());
    SetupAB(dt, dy_, ny_, 0.5, ab_.ayH.data(), ab_.byH.data());
    SetupAB(dt, dz_, nz_, 0.5, ab_.azH.data(), ab_.bzH.data());
  }

  // Start using 'E' or 'H' part of Psi array.
  float *Psi = starting_Psi ? starting_Psi :
               (Psi_ + step_H * (PsiArraySize() / 2));

  // Step over the (up to) 3^3 regions that have (potentially) different
  // combinations of PML coordinate stretching parameters.
  int xcoords[4], ycoords[4], zcoords[4];
  bool xstretch[3], ystretch[3], zstretch[3];
  int xcount = CoordHelper(PML_XMIN, PML_XMAX, nx_, rangex, xcoords, xstretch);
  int ycount = CoordHelper(PML_YMIN, PML_YMAX, ny_, rangey, ycoords, ystretch);
  int zcount = CoordHelper(PML_ZMIN, PML_ZMAX, nz_, rangez, zcoords, zstretch);
  for (int rx = 0; rx < xcount; rx++) {         // rx,ry,rz = region indexes
    for (int ry = 0; ry < ycount; ry++) {
      for (int rz = 0; rz < zcount; rz++) {
        // Each Z-slice of Yee cells in this region is processed in parallel.
        // @@@ Currently we only go a little bit faster with multiple threads,
        // probably because we're saturating memory bandwidth.

        // Because of parallelism we must precompute the offsets into the Psi
        // array that each slice will use. Fortunately this is constant per
        // slice within a region. pps = Psi per slice:
        int pps = 2 * (xstretch[rx] + ystretch[ry] + zstretch[rz]) *
                      (xcoords[rx + 1] - xcoords[rx]) *
                      (ycoords[ry + 1] - ycoords[ry]);
        int z1 = zcoords[rz];
        int z2 = zcoords[rz + 1];
        ParallelFor(z1, z2-1, kNumThreads, [&](int z) mutable {
          float *slice_Psi = Psi + (z - z1)*pps;
          float *last_Psi = (this->*(helpers[xstretch[rx] + ystretch[ry]*2 +
                                             zstretch[rz]*4 + step_H*8]))
            (dt, xcoords[rx], xcoords[rx + 1], ycoords[ry], ycoords[ry + 1],
             z, z+1, slice_Psi);
          if (last_Psi > slice_Psi + pps) {
            // The slice update overran its allocated portion of the Psi
            // buffer.
            Panic("INTERNAL ERROR: PER-Z-SLICE PSI ARRAY OVERFLOW");
          }
        });
        Psi += (z2 - z1) * pps;
      }
    }
  }

  // Make sure we did not overflow the half of the Psi array we are using.
  CHECK_MSG(Psi <= Psi_ + (1 + step_H) * (PsiArraySize() / 2),
            "Internal error: Psi array overflow");
  return Psi;
}

void CD::PECVoxels(const vector<int> &voxel_indexes) {
  // Total size of E_ array.
  const size_t size = (nx_ + 1)*(ny_ + 1)*(nz_ + 1);

  const int sx = 1;                     // Array skip in the X direction
  const int sy = (nx_ + 1);             // Array skip in the Y direction
  const int sz = (nx_ + 1) * (ny_ + 1); // Array skip in the Z direction

  // Check voxel indexes, set E fields to zero on all 12 edges of the voxel
  // cube.
  for (int i = 0; i < voxel_indexes.size(); i++) {
    int index = voxel_indexes[i];
    CHECK_MSG(index >= 0 && (index + sx + sy + sz) < size,
              "'voxel_indexes' values out of range");
    E_[index].x = 0;
    E_[index].y = 0;
    E_[index].z = 0;
    E_[index + sx].y = 0;
    E_[index + sx].z = 0;
    E_[index + sy].x = 0;
    E_[index + sy].z = 0;
    E_[index + sz].x = 0;
    E_[index + sz].y = 0;
    E_[index + sy + sz].x = 0;
    E_[index + sx + sz].y = 0;
    E_[index + sx + sy].z = 0;
  }
}

double CD::EFieldChange(const CD &cd_snapshot) {
  double Esum = 0, Dsum = 0;
  int sz = (nx_+1) * (ny_+1) * (nz_+1);
  for (int i = 0; i < sz; i++) {
    Esum += sqr(E_[i].x) + sqr(E_[i].y) + sqr(E_[i].z);
    Dsum += sqr(E_[i].x - cd_snapshot.E_[i].x) +
            sqr(E_[i].y - cd_snapshot.E_[i].y) +
            sqr(E_[i].z - cd_snapshot.E_[i].z);
  }
  return sqrt(Dsum) / sqrt(Esum);
}

int CD::PsiArraySize() {
  // Each coordinate that is stretched takes four extra field variables per
  // Yee cell (two per E/H field).
  size_t xsides = ((pml_ & PML_XMIN) != 0) + ((pml_ & PML_XMAX) != 0);
  size_t ysides = ((pml_ & PML_YMIN) != 0) + ((pml_ & PML_YMAX) != 0);
  size_t zsides = ((pml_ & PML_ZMIN) != 0) + ((pml_ & PML_ZMAX) != 0);
  size_t sz = ((xsides * pml_depth_) * ny_ * nz_) * 4 +
              ((ysides * pml_depth_) * nx_ * nz_) * 4 +
              ((zsides * pml_depth_) * nx_ * ny_) * 4;
  return int(sz);       //@@@ really need to use size_t for all indexes&sizes
}

void CD::SetupAB(double dt, double dx, int n, double ofs, float *a, float *b) {
  // Array values are set assuming a PML on both sides of the axis, non-PML
  // sides won't actually used the computed values.

  // The maximum-strength parameters to use. There are mostly taken from Ilkka
  // Laakso's paper, where they were found by experiment. From Laakso's paper:
  // "alpha governs the absorption of evanescent felds. Typically it should be
  // larger for problems with strongly evanescent waves, and smaller for
  // problems with only weakly evanescent felds. The optimal value for
  // parameter a is approximately independent of the frequency, being inversely
  // proportional to the size of the problem." There is not much guidance for
  // setting kappa. Values not close to 1 tend to destroy the PML. Laakso
  // recommends 5, which doesn't work here for some reason.
  const double m = 3;           // depth raised to this power
  const double m_alpha = 1;
  const double sigma_max = 0.8 * (m + 1) / (dx * kZ0);
  const double alpha_max = 0.05;
  const double kappa_max = 1;

  // Step through Yee cells:
  for (int i = 0; i < n; i++) {
    // Compute the PML depth for the left (ofs=0) or middle (ofs=0.5) of a Yee
    // cell. A depth of <= 0 means inside the non-PML CD, a depth of 1 means
    // the PEC border of the CD.
    double depth = 0;
    if (i < pml_depth_) {
      depth = 1.0 - (double(i) + ofs) / double(pml_depth_);
    }
    if (i >= n - pml_depth_) {
      depth = 1.0 + (double(i - n) + ofs) / double(pml_depth_);
    }

    // Compute the sigma,alpha and kappa value from the depth.
    double depth_to_m = pow(depth, m);
    double sigma = sigma_max * depth_to_m;
    double alpha = pow(1 - depth, m_alpha) * alpha_max;
    double kappa = 1 + depth_to_m * (kappa_max - 1);

    // Compute the 'a' and 'b' value from sigma,alpha and kappa.
    if (depth <= 0) {
      a[i] = 0;
      b[i] = 1;
    } else {
      b[i] = float(exp(-(sigma/kappa + alpha) * dt / kEpsilon));
      a[i] = float((sigma / (sigma*kappa + sqr(kappa)*alpha)) * (b[i] - 1.0));
    }
  }
}

int CD::CoordHelper(int minflag, int maxflag, int n, Range r,
                    int coords[4], bool stretch[3]) const {
  // Note that the PML is known not to cover the entire CD (there is always a
  // non-PML gap between the PMLs) thanks to a constructor check.
  // For n=10 and pml_depth=3 we have the following structure:
  //
  //     0--1--2--3--4--5--6--7--8--9--10--  <-- Field structs
  //     PPPPPPPPPP           PPPPPPPPPP     <-- Yee cells where PML exists
  CHECK_MSG(r.Valid() && r.size() >= 1 && r.hi()+1 <= n,
            "Invalid or empty range");
  int rlo = r.lo();
  int rhi = r.hi() + 1;         // Range is [rlo,rhi)
  int pml_lo = (pml_ & minflag) ? pml_depth_ : -1;
  int pml_hi = (pml_ & maxflag) ? (n - pml_depth_) : n + 1;
  int i = 0;
  coords[i] = rlo;
  i++;
  if (rlo < pml_depth_ && rhi > pml_depth_) {
    coords[i] = pml_depth_;
    i++;
  }
  if (rlo < pml_hi && rhi > pml_hi) {
    coords[i] = pml_hi;
    i++;
  }
  coords[i] = rhi;
  for (int j = 0; j < i; j++) {
    stretch[j] = coords[j] < pml_lo || coords[j] >= pml_hi;
  }
  return i;
}

template<bool x_stretched, bool y_stretched, bool z_stretched>
float* CD::StepE_Helper(double dt, int x1, int x2, int y1, int y2,
        int z1, int z2, float *Psi) {
  // The x_stretched (etc) parameter tests are assumed be be optimized out by
  // templated function expansion. Don't update the tangential E field for the
  // boundary cells at the -ve coordinate faces (it's a PEC boundary, the field
  // should remain at zero, and some of the H field needed is missing anyway).
  // Don't update the last cell along each coordinate direction because that
  // represents the PEC boundary also.

  // For toroidal symmetry we take care to reference the high-coordinate H
  // field when updating the E field on the zero-coordinate faces, then we copy
  // the zero-coordinate E field to the high coordinate "PEC boundary" region.
  // We thus end up with two copies of the E field on the shared face.

  // Constants that scale differences of field components along the x,y,z
  // coordinates.
  const float kx = float(dt / kEpsilon / dx_);
  const float ky = float(dt / kEpsilon / dy_);
  const float kz = float(dt / kEpsilon / dz_);

  // There is a (minor) speed savings to be had by converting the a and b
  // vectors to float pointers.
  const float *ax = ab_.axE.data();
  const float *ay = ab_.ayE.data();
  const float *az = ab_.azE.data();
  const float *bx = ab_.bxE.data();
  const float *by = ab_.byE.data();
  const float *bz = ab_.bzE.data();

  const int sx = 1;                     // Array skip in the X direction
  const int sy = (nx_ + 1);             // Array skip in the Y direction
  const int sz = (nx_ + 1) * (ny_ + 1); // Array skip in the Z direction

  int x1_adjust = (x1 == 0) ? 1 : x1;

  // @@@ We need E field copies for these other TOROID modes:
  CHECK_MSG((toroid_ & TOROID_X) == 0, "TOROID_X not fully implemented yet");
  CHECK_MSG((toroid_ & TOROID_Z) == 0, "TOROID_Z not fully implemented yet");

  for (int iz = z1; iz < z2; iz++) {
    // The skip values here and below allow us to handle the coordinate==0
    // faces specially, because they need to reference H fields from lower
    // index cells that are either assumed to be zero or stored elsewhere. Note
    // that setting skip=0 ensures that the derivative for that direction will
    // be zero.
    int zskip = (iz == 0) ? ((toroid_ & TOROID_Z) ? (sz * (nz_-1)) : 0)
                          : (-sz);
    SET_XY_POINTERS_FOR_Z(iz)
    for (int iy = y1; iy < y2; iy++) {
      int yskip = (iy == 0) ? ((toroid_ & TOROID_Y) ? (sy * (ny_-1)) : 0)
                            : (-sy);
      SET_X_POINTERS_FOR_Y(iy)
      if (x1 == 0) {
        // Handle the x1 == 0 case specially here so we don't have to deal with
        // it in the inner loop below. If TOROID_X was not set this could be
        // simplified even further, but there's little speed advantage of
        // special casing that.
        const int ix = 0;
        int xskip = ((toroid_ & TOROID_X) ? (sx * (nx_-1)) : 0);
        SET_POINTERS_FOR_X(ix)
        COMPUTE_H_DELTAS_AND_HANDLE_STRETCH(x, y, z, xskip)
        COMPUTE_H_DELTAS_AND_HANDLE_STRETCH(y, x, z, yskip)
        COMPUTE_H_DELTAS_AND_HANDLE_STRETCH(z, x, y, zskip)
        E->x += DyHz * ky - DzHy * kz;
        E->y += DzHx * kz - DxHz * kx;
        E->z += DxHy * kx - DyHx * ky;
      }
      for (int ix = x1_adjust; ix < x2; ix++) {
        SET_POINTERS_FOR_X(ix)
        COMPUTE_H_DELTAS_AND_HANDLE_STRETCH(x, y, z, -sx)
        COMPUTE_H_DELTAS_AND_HANDLE_STRETCH(y, x, z, yskip)
        COMPUTE_H_DELTAS_AND_HANDLE_STRETCH(z, x, y, zskip)
        E->x += DyHz * ky - DzHy * kz;
        E->y += DzHx * kz - DxHz * kx;
        E->z += DxHy * kx - DyHx * ky;
      }
    }

    // Copy y=0 E field to the opposite face if necessary. Do this here rather
    // than in a separate loop to improve memory locality.
    if (toroid_ & TOROID_Y) {
      for (int ix = 0; ix < nx_; ix++) {
        E_[ix*sx + ny_*sy + iz*sz] = E_[ix*sx + iz*sz];
      }
    }
  }

  return Psi;
}

template<bool x_stretched, bool y_stretched, bool z_stretched>
float* CD::StepH_Helper(double dt, int x1, int x2, int y1, int y2,
        int z1, int z2, float *Psi) {
  // The x_stretched (etc) parameter tests are assumed be be optimized out by
  // templated function expansion. We are careful to not update the last cell
  // along each coordinate direction because that represents the PEC boundary.
  // The H updates below will refer to the zero values in the layer though.

  // For toroidal symmetry we don't have to take any special action here as the
  // H field faces of the Yee cells are entirely encompassed by the E field
  // edges.

  // Constants that scale differences of field components along the x,y,z
  // coordinates.
  const float kx = float(dt / kMu / dx_);
  const float ky = float(dt / kMu / dy_);
  const float kz = float(dt / kMu / dz_);

  // There is a (minor) speed savings to be had by converting the a and b
  // vectors to float pointers.
  const float *ax = ab_.axH.data();
  const float *ay = ab_.ayH.data();
  const float *az = ab_.azH.data();
  const float *bx = ab_.bxH.data();
  const float *by = ab_.byH.data();
  const float *bz = ab_.bzH.data();

  // Field update loops.
  const int sx = 1;                        // Array skip in the X direction
  const int sy = (nx_ + 1);                // Array skip in the Y direction
  const int sz = (nx_ + 1) * (ny_ + 1);    // Array skip in the Z direction
  for (int iz = z1; iz < z2; iz++) {
    SET_XY_POINTERS_FOR_Z(iz)
    for (int iy = y1; iy < y2; iy++) {
      SET_X_POINTERS_FOR_Y(iy)
      for (int ix = x1; ix < x2; ix++) {
        SET_POINTERS_FOR_X(ix)
        COMPUTE_E_DELTAS_AND_HANDLE_STRETCH(x, y, z)
        COMPUTE_E_DELTAS_AND_HANDLE_STRETCH(y, x, z)
        COMPUTE_E_DELTAS_AND_HANDLE_STRETCH(z, x, y)
        //@@@ This can be optimized to use kx==ky==kz==k for cubical cells,
        //    but only provides a slight speedup.
        H->x += DzEy * kz - DyEz * ky;
        H->y += DxEz * kx - DzEx * kz;
        H->z += DyEx * ky - DxEy * kx;
      }
    }
  }
  return Psi;
}

} // namespace fdtd
