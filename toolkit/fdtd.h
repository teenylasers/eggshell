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

// FDTD computational domain class (CD) and associated support classes. These
// classes contain only the computational core of the FDTD method, elaborations
// such as excitation, measurement, convergence checks, visualization and file
// I/O are provided elsewhere.

#ifndef __TOOLKIT_FDTD_H__
#define __TOOLKIT_FDTD_H__

#include "myvector"
#include "error.h"

namespace fdtd {

using std::vector;

//***************************************************************************
// A range-of-integers class, more or less the same as Matlab a:b ranges.
// The range of values is lo(),lo()+1,...,hi().

class Range {
 public:
  Range() { lo_ = 0; hi_ = 0; }
  explicit Range(int value) : lo_(value), hi_(value) {}
  Range(int lo, int hi) : lo_(lo), hi_(hi) {}

  int lo() const { return lo_; }
  int hi() const { return hi_; }
  int size() const { return hi_ - lo_ + 1; }
  double center() const { return (lo_ + hi_) * 0.5; }
  void Set(int lo, int hi) { lo_ = lo; hi_ = hi; }

  bool Valid() const { return lo_ <= hi_; }
  bool Within(Range r) const { return lo_ >= r.lo() && hi_ <= r.hi(); }

  Range operator+(int offset) const { return Range(lo_+offset, hi_+offset); }
  Range operator-(int offset) const { return Range(lo_-offset, hi_-offset); }
  bool operator==(Range r) const { return lo_ == r.lo_ && hi_ == r.hi_; }

 public:
  int lo_, hi_;                         // Range is [lo_,hi_]
};

//***************************************************************************
// A 3D boolean array that indicates the PEC voxels in a computational domain.
// All indexes passed in and out of this class are 0-based.

class PEC {
 public:
  // Create an empty (zero size) PEC.
  PEC();

  // Create a voxel array corresponding to a computational domain of nx*ny*nz
  // Yee cells.
  PEC(int nx, int ny, int nz);

  // Reset to an empty (zero size) PEC.
  void Reset();

  // Default copy constructor and assignment operator are fine.

  // Accessors.
  bool empty() const { return pec_.empty(); }
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nz() const { return nz_; }
  void CheckIndexes(int x, int y, int z) const {
    DBG_CHECK_MSG(x >= 0 && x <= nx_ && y >= 0 && y <= ny_ &&
                  z >= 0 && z <= nz_, "Invalid indexes");
  }
  bool Get(int x, int y, int z) const {
    CheckIndexes(x, y, z);
    return pec_[x + y*(nx_ + 1) + z*(nx_ + 1)*(ny_ + 1)];
  }
  void Set(int x, int y, int z, int value) {
    CheckIndexes(x, y, z);
    pec_[x + y*(nx_ + 1) + z*(nx_ + 1)*(ny_ + 1)] = value;
  }

  // Return the PEC as a bool vector of size (nx_+1)*(ny_+1)*(nz_+1).
  const vector<bool> &GetBoolVector() const { return pec_; }

  // Set the given range of voxels to the value.
  void Set(Range rx, Range ry, Range rz, bool value);

 private:
  int nx_, ny_, nz_;
  vector<bool> pec_;    // Of size (nx_+1)*(ny_+1)*(nz_+1)
};

//***************************************************************************
// Box shaped FDTD computational domain. All indexes passed in and out of this
// class are 0-based.

class CD {
 public:
  enum {
    // Bits to indicate which walls of the CD have a PML.
    PML_XMIN = 1,
    PML_XMAX = 2,
    PML_YMIN = 4,
    PML_YMAX = 8,
    PML_ZMIN = 16,
    PML_ZMAX = 32,
    PML_ALL = PML_XMIN | PML_XMAX | PML_YMIN | PML_YMAX | PML_ZMIN | PML_ZMAX,

    // Bits to indicate which faces of the CD have discrete translational
    // (toroid) symmetry.
    TOROID_X = 1,
    TOROID_Y = 2,
    TOROID_Z = 4,
  };

  // Create a new standalone computational domain. The size of the
  // computational domain in Yee cells is nx,ny,nz. The size of each yee cell
  // is dx,dy,dz. The walls that have a PML are indicated by bits in 'pml'. The
  // depth of the PML is cells is 'pml_depth'. Toriodal symmetries are
  // indicated by bits in 'toroid'.
  CD(int nx, int ny, int nz, double dx, double dy, double dz,
     int pml, int pml_depth, int toroid = 0);

  // Create a copy of a computational domain. If E-only is true then just copy
  // the E-field, don't allocate or copy the H or Psi data (e.g. for
  // convergence checks).
  CD(const CD &cd, bool E_field_only = false);

  // Deallocate field and Psi memory only if the standalone constructor was
  // called.
  ~CD();

  // Operators.
  void operator= (const CD &cd);

  // Accessors.
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nz() const { return nz_; }
  double dx() const { return dx_; }
  double dy() const { return dy_; }
  double dz() const { return dz_; }
  int pml() const { return pml_; }
  int pml_depth() const { return pml_depth_; }

  // Access individual field components. The Yee cell index is (x,y,z), the
  // field component is 'c'.
  float GetE(int c, int x, int y, int z) const {
    DBG_CHECK_MSG(x >= 0 && x <= nx_ && y >= 0 && y <= ny_ &&
                  z >= 0 && z <= nz_ && c >= 0 && c <= 2,
                  "Arguments out of range");
    Field *e = E_ + x + (nx_+1)*y + (nx_+1)*(ny_+1)*z;
    return (&e->x)[c];          // Assuming struct Field layout tested in CD::CD
  }

  // Access one field component in a range of Yee cells. Return an error if the
  // ranges are invalid. The returned pointer is the lo() end of the ranges,
  // the sx,sy,sz are the skips in a float array to access the other components
  // in the box. The caller must guarentee not to go outside the box.
  float *GetEBox(int c, Range rx, Range ry, Range rz,
                 int *sx, int *sy, int *sz) const;
  float *GetHBox(int c, Range rx, Range ry, Range rz,
                 int *sx, int *sy, int *sz) const;

  // Access all E field components as a 3D matrix in 'matlab' format, returning
  // the size of each dimension. This is for export to matlab files only.
  float *GetEMatlab(int32_t dims[4]) const;

  // Return the step size dictated by the Courant condition. The dt value
  // passed to Step() should be no larger than this.
  double GetCourantStep() const;

  // Take a single time step for the E or H field (of time dt).
  void Step(double dt, bool step_H);

  // Like Step(), but only update the Yee cells given by the ranges. It is the
  // caller's responsibility to ensure that all appropriate cells are updated.
  // NOTE! Currently, the order in which the Psi variables are allocated
  // internally will depend on the exact order of the ranges passed to this
  // function. The order must be consistent for a single run. In particular,
  // Step() and StepRanges() can not be used interchangably. The function must
  // be called like:
  //   float *Psi = 0;
  //   for (...) {
  //     Psi = StepRanges(..., Psi, ...);
  //   }
  float *StepRanges(double dt, bool step_H, float *starting_Psi,
                    Range rangex, Range rangey, Range rangez);

  // Zero out the tangential E field of all the voxels whos linear indexes are
  // mentioned in voxel_indexes.
  void PECVoxels(const vector<int> &voxel_indexes);

  // Compute the fractional change in RMS field strength between the current E
  // field and the field in cd_snapshot. In matlab terms this computes
  // (assuming that the entire E field is contained in the column vector 'E'):
  //   sqrt(sum((E - cd_snapshot.E).^2)) / sqrt(sum(E.^2))
  double EFieldChange(const CD &cd_snapshot);

  // Compute the size of the Psi array needed for a PML.
  int PsiArraySize();

 private:
  // Size of the computational domain in Yee cells.
  int nx_, ny_, nz_;

  // Size of each Yee cell (in meters).
  double dx_, dy_, dz_;

  // Arrays of [nx+1][ny+1][nz+1] grid cells that store E and H vectors. We
  // store E and H in separate areas of memory. Colocating them slows field
  // stepping down by about 20%.
  struct Field {                // Field vector (E or H)
    float x, y, z;
  };
  Field * __restrict E_;        // Main E field  @@@ See if __restrict helps
  Field * __restrict H_;        // Main H field

  // PML and symmetry parameters.
  int pml_;                     // Combination of PML_* constants
  int pml_depth_;               // How deep (in Yee cells) the PML is
  int toroid_;                  // Combination of TOROID_* constants

  // The cached 'a' and 'b' filter constant arrays for Psi variables.
  struct AB_Arrays {
    vector<float> axE, ayE, azE, bxE, byE, bzE;         // For E steps
    vector<float> axH, ayH, azH, bxH, byH, bzH;         // For H steps
    AB_Arrays(int nx, int ny, int nz) :
        axE(nx), ayE(ny), azE(nz), bxE(nx), byE(ny), bzE(nz),
        axH(nx), ayH(ny), azH(nz), bxH(nx), byH(ny), bzH(nz) {}
  };
  AB_Arrays ab_;                // Results from SetupAB().
  double ab_dt_;                // The timestep used to set the ab_ (0=none)

  // Psi field variables, of size PsiArraySize(). The first half of the array
  // is for the H deltas, the second half is for the E deltas. The variables
  // are packed into the array in as-used order, so we don't store these values
  // for non-PML cells.
  float *__restrict Psi_;

  // Setup the 'a' and 'b' filter constant arrays for Psi variables, for a
  // single axis. The arrays have size 'n'. The coordinate is offset by 'ofs',
  // which is 0 to compute the values for the left side of the cells and 0.5
  // for the middle. For details see equations 8 and 13 of
  // http://www.engr.uky.edu/~gedney/roden_gedney_cpml_motl.pdf
  void SetupAB(double dt, double dx, int n, double ofs, float *a, float *b);

  // Utility function to set coordinate ranges and stretching flags for the
  // field stepping functions. The minflag and maxflag values are set to e.g.
  // PML_XMIN and PML_XMAX and 'n' is set to e.g. nx_. The return value is the
  // number of coordinate ranges, the range values are put in 'coords' and the
  // stretching flags are put in 'stretch'. Only coordinates that intersect the
  // range r are returned.
  int CoordHelper(int minflag, int maxflag, int n, Range r,
                  int coords[4], bool stretch[3]) const;

  // A field stepping function that knows which coordinates are stretched. This
  // matches StepE_Helper() and StepH_Helper().
  typedef float* (CD::*Step_Helper_fn_t)(double dt, int x1, int x2,
      int y1, int y2, int z1, int z2, float *Psi);

  // Step part of the E field, in the Field cells [x1..x2),[y1..y2),[z1..z2).
  template<bool x_stretched, bool y_stretched, bool z_stretched>
  float* StepE_Helper(double dt, int x1, int x2, int y1, int y2,
      int z1, int z2, float *Psi);

  // Step part of the H field, in the Field cells [x1..x2),[y1..y2),[z1..z2).
  template<bool x_stretched, bool y_stretched, bool z_stretched>
  float* StepH_Helper(double dt, int x1, int x2, int y1, int y2,
      int z1, int z2, float *Psi);

  // To support GetEBox(), GetHBox().
  float *GetFieldBox(Field *F, int c, Range rx, Range ry, Range rz,
                     int *sx_ret, int *sy_ret, int *sz_ret) const;

  // Testing friends.
  friend void TestCoordHelper(const CD &cd, int p[]);
};

} // namespace fdtd

#endif
