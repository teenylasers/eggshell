
// Solve for u the PDE: Diamond[u] + g(x,y)*u = f(x,y)
// Where Diamond[u] = sigma_xx d^2u/dx^2 +
//                    sigma_yy d^2u/dy^2 +
//                    2 sigma_xy d^2u/dxdy
// Emphasis is on the common case where sigma_xx = sigma_yy = 1, sigma_xy = 0,
// which makes Diamond[u] into the Laplacian.

// @@@ TODO
// * Interpolate k instead of k^2 ?
// * Fix the discontinuous gradient stuff
// * Naming consistency: k^2, g(), C. Rename g as 'k2'?
// * Is there a way we can avoid the copy of triplets on SolveSystem.
// * Use an 'Index' type instead of 'int', for when we need more than 2^31
//   points
// * Linear interpolation of f,g,u for complex phasor fields is maybe less
//   sensible than some kind of polar interpolation?

#ifndef __TOOLKIT_FEMSOLVER_H__
#define __TOOLKIT_FEMSOLVER_H__

#include "myvector"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "error.h"
#include "eigensolvers.h"

namespace FEM {

// Debugging mode.
inline bool kDebug() { return false; }

using std::vector;

//***************************************************************************
// Test facilities used by ExampleFEMProblem below.

inline static double RandDouble() {
  return double(std::rand()) / double(RAND_MAX);
}

// A numeric class with some of the semantics of 'double'. This is used to make
// some double-like but mutually incompatible types to test the FEMSolver
// handling of Number, GNumber and MNumber. This also illustrates the minimum
// set of overloaded operators required.
#define CREATE_NUMBER_CLASS(name) \
struct name { \
  double value, derivative; \
  name() : value(0), derivative(0) {} \
  explicit name(double v) : value(v), derivative(0) {} \
  name & operator=(double v) { value = v; derivative = 0; return *this; } \
  name operator-() const { return name(-value); } \
  name operator+(const name &a) const { return name(value + a.value); } \
  name operator-(const name &a) const { return name(value - a.value); } \
  name operator*(const name &a) const { return name(value * a.value); } \
  name operator/(const name &a) const { return name(value / a.value); } \
  void operator+=(const name &a) { value += a.value; } \
  void operator-=(const name &a) { value -= a.value; } \
  name operator*(double a) const { return name(value * a); } \
  name operator/(double a) const { return name(value / a); } \
  bool operator==(const name &a) const { return value == a.value; } \
  bool operator!=(const name &a) const { return value != a.value; } \
  bool operator>(double a) const { return value > a; } \
  bool operator>(const name &a) const { return value > a.value; } \
}; \
inline name sqrt(const name &a) { return name(::sqrt(a.value)); }

CREATE_NUMBER_CLASS(MyNumber)
CREATE_NUMBER_CLASS(MyGNumber)
#undef CREATE_NUMBER_CLASS

//***************************************************************************
// A simple example of a FEM problem description class. A class with this
// signature must be given to FEMSolver. This class is also used for testing.
// The following grid is used:
//
//     0---1---2---3---4        Points and triangles are numbered from 0
//     |  /|  /|  /|  /|        at the top left.
//     |0/1|2/3|4/5|6/7|
//     |/  |/  |/  |/  |        The top and left edges are Dirichlet, the
//     5---6---7---8---9        others are Robin.
//     |  /|10/|12/|14/|
//     |8/9| / | / | / |
//     |/  |/11|/13|/15|        y
//     10--11--12--13--14       |
//     |16/|18/|20/|22/|        |
//     | / | / | / | / |        +---x
//     |/17|/19|/21|/23|
//     15--16--17--18--19
//     |24/|26/|28/|30/|
//     | / | / | / | / |
//     |/25|/27|/29|/31|
//     20--21--22--23--24

struct FEMProblem {
  enum EType {          // Edge type constants
    INTERIOR,           // Interior (non-boundary) point
    DIRICHLET,          // Dirichlet boundary, u = 0
    ROBIN,              // Robin boundary, du/dnormal + alpha*u = beta,
  };                    //   the normal is outward-pointing
};

struct ExampleFEMProblem : public FEMProblem {
  // Types. For each numeric type an explicit cast from double must be
  // possible. You must make sure that the numeric types have a proper value
  // initialization, i.e. that e.g. Number() is a zero.
  typedef MyNumber Number;      // Problem input: f, g, alpha, beta, triplets
  typedef MyGNumber GNumber;    // Geometry number: x,y for points
  typedef double MNumber;       // For the matrix that is factorized
  typedef Eigen::Triplet<Number> Triplet;
  typedef Eigen::Matrix<GNumber, 2, 1> Point;   // Point x,y
  typedef Eigen::Matrix<Number, Eigen::Dynamic, 1> NumberVector;
  typedef Eigen::Matrix<MNumber, Eigen::Dynamic, 1> MNumberVector;

  // The sparse matrix factorizer that will be used. When solving the Helmholtz
  // equation and MNumber is complex the SparseLU should be used as the system
  // matrix is often symmetric, but not hermitian. When MNumber is real then
  // the problem can be symmetric positive definite and SimplicialLLT or
  // SimplicialLDLT can be used. Even for symmetric problems we set the full
  // system matrix but the factorizer may only read e.g. the lower triangle.
  // COLAMDOrdering is the only ordering that makes sense for SparseLU, the AMD
  // and METIS-based orderings are only effective for symmetric factorizations
  // and will slow down SparseLU. For efficiency of the factorizer and
  // Eigen-compatibility reasons, MNumber should be a regular real or complex
  // type.
  typedef Eigen::SimplicialLLT<Eigen::SparseMatrix<MNumber>, Eigen::Lower,
                               Eigen::AMDOrdering<int> > Factorizer;
  // Or this:
  //     typedef Eigen::SparseLU<Eigen::SparseMatrix<MNumber>,
  //                             Eigen::COLAMDOrdering<int> > Factorizer;

  // Return true if we should add the dielectric forcing term, that is one way
  // to get discontinuous gradients at dielectric boundaries in Exy cavities.
  bool AddDielectricForcingTerm() const { return false; }

  // Conversions.
  MNumber MNumberFromNumber(Number n) const { return MNumber(n.value); }
  Number GNumberToNumber(GNumber n) const { return Number(n.value); }

  // Cast an MNumberVector to a NumberVector (if Number and MNumber are the
  // same type) or return 0 otherwise. This will return either 0 or v.
  MNumberVector *CastMNumberToNumberVector(NumberVector *v) { return 0; }

  // The number of points and triangles in the mesh.
  int NumPoints() const { return 25; }
  int NumTriangles() const { return 32; }

  // The coordinates of point 'i'.
  Point PointXY(int i) const { return Point(GNumber(i % 5), -GNumber(i / 5)); }

  // The index of point j (0,1,2) for triangle i.
  int Triangle(int i, int j) const {
    return i/8*5 + i%8/2 + j + (j==2)*3 + (i&1)*(5 - (j==2)*9);
  }

  // The triangle index of the neighbor of edge j for triangle i, or -1 if this
  // is a boundary triangle. Edge j is from point j to ((j+1) mod 3).
  int Neighbor(int i, int j) const { return -1; }

  // The 'g' and 'f' values at point 'j' of triangle 'i'. These are linearly
  // interpolated across the triangle points. Specifying these values per
  // triangle rather than just by point index allows you to specify either
  // smoothly interpolated values across triangles or step changes.
  Number PointG(int i, int j) const { return Number(test_g.at(i*3 + j)); }
  Number PointF(int i, int j) const { return Number(test_f.at(i*3 + j)); }

  // The type of edge j (0,1,2) of triangle i. Edge j is between points j and
  // ((j+1) mod 3) of the triangle.
  EType EdgeType(int i, int j) const {
    if ((i & 1) == 0 && i <= 6 && j == 0) return DIRICHLET;
    if ((i % 8) == 0 && j == 2) return DIRICHLET;
    if ((i & 1) == 1 && i >= 25 && j == 0) return ROBIN;
    if ((i % 8) == 7 && j == 1) return ROBIN;
    return INTERIOR;
  }

  // Return the 'alpha' and 'beta' for ROBIN edge j of triangle i, at point k.
  // The j and k are 0,1,2. This corresponds to EdgeType(i,j). The point k is
  // either j or (j+1) mod 3. The corresponding PointG(i,k) value is given as a
  // convenience. Both alpha and beta are linearly interpolated over the edge.
  // Both j and k are given so that adjacent robin edges with discontinuous but
  // linearly interpolated values can be specified.
  void Robin(int i, int j, int k, const Number &g0,
             Number *alpha, Number *beta) const {
    *alpha = Number(test_a.at(i*3 + k));
    *beta = Number(test_b.at(i*3 + k));
  }

  // Return false if Diamond[u] is the Laplacian for triangle i. Otherwise
  // return true and set the sigma values. Each return vector will have 3
  // preallocated slots for the three points in triangle i.
  bool AnisotropicSigma(int i, NumberVector *sigma_xx, NumberVector *sigma_yy,
                        NumberVector *sigma_xy) {
    return false;
  }

  // Return the absolute value of 'a'. We call this instead of a global
  // abs(GNumber) because there might be confusion with the C standard
  // library's abs(int) if GNumber is double. However we do call a global
  // sqrt(GNumber).
  GNumber Absolute(const GNumber &a) { return GNumber(fabs(a.value)); }

  // Some Number types will know their derivative with respect to a parameter.
  // Extract this information, for ComputeSolutionDerivative().
  MNumber Derivative(const Number &a) {
    return a.derivative;
  }

  // Class for performance tracing. Each instance of the class should measure
  // the amount of time it is alive and associate that with the description.
  struct DoTrace {
    DoTrace(const char *description) {}
  };

  // Non-geometry parameters, for testing only (these are not part of the
  // required signature).
  vector<double> test_f, test_g, test_a, test_b;
  ExampleFEMProblem() : test_f(NumTriangles() * 3), test_g(NumTriangles() * 3),
                        test_a(NumTriangles() * 3), test_b(NumTriangles() * 3) {
    for (int i = 0; i < NumTriangles() * 3; i++) {
      // g and a are positive to help keep the system matrix positive definite.
      test_f[i] = (RandDouble() * 2 - 1) + i / 20.0;
      test_g[i] = RandDouble();
      test_a[i] = RandDouble();
      test_b[i] = RandDouble() * 2 - 1;
    }
    // We can make e.g. g piecewise constant like this:
    //   for (int i = 0; i < NumTriangles(); i++) {
    //     test_g[i*3+1] = test_g[i*3];
    //     test_g[i*3+2] = test_g[i*3];
    //   }
  };
};

//***************************************************************************
// A solver for FEM problems. T must have the same signature as
// ExampleFEMProblem. This object is designed to compute one solution to one
// problem, i.e. to cache the intermediate results for reuse, however functions
// are provided to invalidate the various caches and explicitly recompute
// things.

// A base class for FEMSolver with virtual functions allowing a variety of
// FEMSolver's to be interrogated through a single interface. We can only
// include functions that do not have parameters dependent on the problem
// types. Also there is no mention of EigenSystem() here as it can not be
// instantiated for all kinds of problems.
// @@@ Check which of the functions below are still needed.
class FEMSolverBase {
 public:
  virtual ~FEMSolverBase() {}

  // The number of elements in the solution vector.
  virtual size_t SystemSize() const = 0;

  // If the internal data structures are consistent then return 0, or return an
  // error message otherwise.
  virtual const char *Verify() const MUST_USE_RESULT = 0;

  // Generate internal maps.
  virtual void CreateIndexMaps() = 0;

  // Reset internal caches.
  virtual void UnCreateSystem() = 0;

  // Create the system. Return true on success, false otherwise.
  virtual bool CreateSystem(bool create_Ctriplets = false) MUST_USE_RESULT = 0;

  // Compute the solution. Return true on success, false otherwise.
  virtual bool SolveSystem() MUST_USE_RESULT = 0;
};

template<class T> class FEMSolver : public FEMSolverBase, public T {
 public:
  // Import some types from T (just so we can use shorter names below).
  typedef typename T::Number Number;
  typedef typename T::MNumber MNumber;
  typedef typename T::GNumber GNumber;
  typedef typename T::NumberVector NumberVector;
  typedef typename T::MNumberVector MNumberVector;
  typedef typename T::Point Point;
  typedef typename T::Triplet Triplet;
  typedef typename T::Factorizer Factorizer;
  typedef typename T::DoTrace DoTrace;

  typedef eigensolvers::LaplacianEigenSolver EigenSolver;

  // Various outputs of the functions below.
  //
  // The index_map_ is a representation of the mesh with all Dirichlet points
  // removed. The index_map_ maps the original point indexes into indexes into
  // a smaller system matrix with no Dirichlet points. Dirichlet points have
  // value -1. The size of the system matrix and the right hand side is the
  // size of reverse_index_map_.
  //
  // The factorizer is kept around so that some clients can update derivative
  // information.
  vector<int> index_map, reverse_index_map;     // Created by CreateIndexMaps()
  vector<Triplet> triplets, Ctriplets;          // Created by CreateSystem()
  NumberVector rhs;                             // Created by CreateSystem()
  Factorizer *factorizer = 0;                   // Created by SolveSystem()
  MNumberVector solution;                       // Created by SolveSystem()
  int solvesystem_retval = -1;                  // Set by SolveSystem()
  EigenSolver *eigensolver = 0;                 // Created by EigenSystem()
  int eigensystem_retval = -1;                  // Set by EigenSystem()

  FEMSolver() {}
  ~FEMSolver() {
    delete factorizer;
    delete eigensolver;
  }

  size_t SystemSize() const override { return reverse_index_map.size(); }

  // Check the mesh for consistency. Return 0 if the mesh looks good, or an
  // error message otherwise.
  const char *Verify() const override MUST_USE_RESULT {
    if (T::NumPoints() <= 0) {
      return "Mesh has zero points";
    }
    if (T::NumTriangles() <= 0) {
      return "Mesh has zero triangles";
    }

    // Check all triangle point indexes are in range and that all points are
    // used by at least one triangle.
    vector<bool> used(T::NumPoints());
    for (int i = 0; i < T::NumTriangles(); i++) {
      for (int j = 0; j < 3; j++) {
        int pindex = T::Triangle(i, j);
        if (pindex < 0 || pindex >= T::NumPoints()) {
          return "Triangle point indexes out of bounds";
        }
        used[pindex] = true;
      }
    }
    for (int i = 0; i < T::NumPoints(); i++) {
      if (!used[i]) {
        return "Some points not used by triangles";
      }
    }

    // Each edge in the mesh should be of INTERIOR type if it has one triangle
    // on each side, or of a boundary type if it has just one triangle.
    std::map<std::pair<int, int>, int> edge_count;
    std::map<std::pair<int, int>, int> edge_type;
    for (int i = 0; i < T::NumTriangles(); i++) {
      for (int j = 0; j < 3; j++) {
        int p1 = T::Triangle(i, j);
        int p2 = T::Triangle(i, (j + 1) % 3);
        auto index = std::make_pair(std::min(p1, p2), std::max(p1, p2));
        if (edge_type.count(index) > 0) {
          if (edge_type[index] != T::EdgeType(i, j)) {
            return "Inconsistent edge type";
          }
        }
        edge_type[index] = T::EdgeType(i, j);
        edge_count[index]++;
      }
    }
    for (auto it : edge_count) {
      if (!(it.second == 1 || it.second == 2)) {
        return "Edge does not bound just one or two triangles";
      }
      if (it.second == 1) {
        if (edge_type[it.first] == T::INTERIOR) {
          return "Exterior edge is not marked as such";
        }
      } else {
        if (edge_type[it.first] != T::INTERIOR) {
          return "Interior edge is not marked as such";
        }
      }
    }

    return 0;
  }

  // All Dirichlet points will be zero in the solution and will not be
  // represented in the system matrix or right hand side. Generate index_map
  // and reverse_index_map, the mappings from non-Dirichlet point indexes to
  // system matrix indexes, and back.
  bool CreateIndexMapsNeedsCalling() const {
    return index_map.empty();
  }
  void CreateIndexMaps() override {
    if (!CreateIndexMapsNeedsCalling()) {
      return;
    }

    // Make a list of all Dirichlet-boundary points.
    vector<bool> dirichlet_points(T::NumPoints());
    for (int i = 0; i < T::NumTriangles(); i++) {
      for (int j = 0; j < 3; j++) {
        if (T::EdgeType(i, j) == T::DIRICHLET) {
          dirichlet_points[T::Triangle(i, j)] = true;
          dirichlet_points[T::Triangle(i, (j + 1) % 3)] = true;
        }
      }
    }
    int system_size = T::NumPoints();
    for (int i = 0; i < T::NumPoints(); i++) {
      if (dirichlet_points[i]) {
        system_size--;
      }
    }
    CHECK(system_size >= 0);

    index_map.resize(T::NumPoints());
    reverse_index_map.resize(system_size);
    int offset = 0;
    for (int i = 0; i < T::NumPoints(); i++) {
      if (dirichlet_points[i]) {
        index_map[i] = -1;
      } else {
        reverse_index_map[offset] = i;
        index_map[i] = offset++;
      }
    }
    CHECK(offset == system_size);
  }

  // Pad a "just solved" solution vector 's' with zeros, as necessary (i.e. if
  // Dirichlet points were removed from the system then put them back).
  void PadSolution(MNumberVector *s) {
    CHECK(s->size() == reverse_index_map.size());
    if (CreateIndexMapsNeedsCalling()) {
      CreateIndexMaps();
    }
    if (s->size() < T::NumPoints()) {
      s->conservativeResize(T::NumPoints());   // Keeps existing values
      for (int i = T::NumPoints() - 1; i >= 0 ; i--) {
        if (index_map[i] == -1) {
          (*s)[i] = 0;
        } else {
          (*s)[i] = (*s)[index_map[i]];
        }
      }
    }
  }

  // Create the system matrix A (as a list in 'triplets') and the right hand
  // side 'rhs' for the FEM problem. If create_Ctriplets is true, separate out
  // the contribution of the nonzero g() function into a separate matrix C
  // stored in Ctriplets, which is useful for solving eigenproblems.
  bool CreateSystemNeedsCalling() const {
    return triplets.empty();
  }
  void UnCreateSystem() override {
    triplets.clear();
    Ctriplets.clear();
    rhs.resize(0);
  }
  bool CreateSystem(bool create_Ctriplets = false) override {
    // Prerequisites.
    DoTrace trace(__func__);
    if (!CreateSystemNeedsCalling()) {
      return true;
    }
    if (CreateIndexMapsNeedsCalling()) {
      CreateIndexMaps();
    }
    triplets.clear();
    Ctriplets.clear();

    // Handle zero system size. This can happen if the mesh is 100% Dirichlet
    // boundary points, i.e. there are no nonzero solution points.
    int system_size = reverse_index_map.size();
    if (system_size == 0) {
      rhs.resize(0);
      return true;
    }

    // We put each triangle's sparse matrix assignments separately into
    // 'triplets', with the assumption that repeated (row,col) indexes will be
    // added together. The last entries are reserved for the diagonal, but we
    // accumulate diagonal values into the 'diagonal' vector first to save
    // space and time.
    int triplets_size = T::NumTriangles() * 6 + system_size;
    triplets.reserve(triplets_size);
    vector<Number> diagonal(system_size);
    vector<Number> Cdiagonal(create_Ctriplets ? system_size : 0);
    if (create_Ctriplets) {
      Ctriplets.reserve(triplets_size);
    }

    // Buffers used below, declared here so we don't keep reallocating.
    NumberVector sigma_xx(3), sigma_yy(3), sigma_xy(3);

    // Build the triplets and right hand side.
    rhs.resize(system_size);
    rhs.setZero();
    for (int i = 0; i < T::NumTriangles(); i++) {
      // Copy triangle points.
      Point pt[3];
      for (int j = 0; j < 3; j++) {
        pt[j] = T::PointXY(T::Triangle(i, j));
      }

      // Compute the area (multiplied by 2) of this triangle.
      GNumber area2;
      {
        Point d1 = pt[1] - pt[0];
        Point d2 = pt[2] - pt[0];
        area2 = T::Absolute(d1[0]*d2[1] - d1[1]*d2[0]);
      }

      // Collect the g() values.
      Number g[3];
      for (int j = 0; j < 3; j++) {
        g[j] = T::PointG(i, j);
      }

      // See if the triangle is anisotropic and collect the sigma values.
      bool anisotropic = T::AnisotropicSigma(i, &sigma_xx, &sigma_yy,
                                             &sigma_xy);

      // Compute system matrix and right hand side contributions for each
      // vertex and edge of the triangle. The signs of everything that goes in
      // the system matrix and the right hand side are inverted here compared
      // to the documented FEM derivation. The reason is so that, in the cases
      // we end up with symmetric definite matrices, the matrices are positive
      // (and not negative) definite so that e.g. Cholesky solvers can be used.
      for (int j0 = 0; j0 < 3; j0++) {
        // Triangle indexes (j0,j1,j2 has the sequence 0,1,2 -> 1,2,0 ->
        // 2,0,1).
        int j1 = (j0 + 1) % 3;
        int j2 = (j0 + 2) % 3;
        // Correspond indexes into the system matrix and RHS, or -1 if none.
        int sj0 = index_map[T::Triangle(i, j0)];
        int sj1 = index_map[T::Triangle(i, j1)];
        int sj2 = index_map[T::Triangle(i, j2)];

        // Compute cot(theta) for the triangle internal angle opposite the
        // j0->j1 edge.
        GNumber cot;
        {
          Point d1 = pt[j0] - pt[j2];
          Point d2 = pt[j1] - pt[j2];
          cot = d1.dot(d2) / area2;
        }

        // Off-diagonal contribution to the C matrix.
        Number g0 = g[j0], g1 = g[j1], g2 = g[j2];
        Number Cij_value = -((g0 + g1) * 2.0 + g2) *
                            T::GNumberToNumber(area2 / 120.0);

        // Add contribution to on-diagonal entry C(sj2, sj2).
        GNumber opplen2 = (pt[j0] - pt[j1]).squaredNorm();
        if (sj2 >= 0) {
          (create_Ctriplets ? Cdiagonal : diagonal)[sj2] -=
              (g0 + g1 + g2 * 3.0) * T::GNumberToNumber(area2 / 60.0);
        }

        // Contributions to A(sj0, sj1) and A(sj1, sj0), and the diagonal.
        Number Aij_value;          // Adds to A(sj0, sj1) and A(sj1, sj0)
        Number Aji_correction(0);  // Adds only to A(sj1, sj0)
        Number sigma_xx_avg, sigma_yy_avg, sigma_xy_avg;  // Triangle averages
        if (!anisotropic) {
          // Isotropic Laplacian.
          Aij_value = -T::GNumberToNumber(cot / 2.0);
          if (sj2 >= 0) {  // Handle diagonal entry at sj2
            diagonal[sj2] -= T::GNumberToNumber(-opplen2 / (area2 * 2.0));
          }
        } else {
          // Handle the anisotropic Laplacian. Rotate the sigma values so that
          // the edge j2-->j0 is effectively in the +X direction (to match the
          // derivation in fem.nb). We take the average of the three vertex
          // sigmas and assume this applies across the entire triangle. This
          // approximation is necessary for now to create a symmetric 'A'
          // matrix.
          // @@@ TODO interpolate sigmas across the triangle.
          Point j2j0 = pt[j0] - pt[j2];
          GNumber j2j0_length2 = j2j0.squaredNorm();  // x1^2 in docs
          j2j0 = j2j0  / sqrt(j2j0_length2);          // j2j0 is now normalized
          // Cosine and sine of angle of j2->j0.
          const Number c = T::GNumberToNumber(j2j0[0]);
          const Number s = T::GNumberToNumber(j2j0[1]);
          // Compute sigma averages.
          sigma_xx_avg = (sigma_xx[0] + sigma_xx[1] + sigma_xx[2]) / 3.0;
          sigma_yy_avg = (sigma_yy[0] + sigma_yy[1] + sigma_yy[2]) / 3.0;
          sigma_xy_avg = (sigma_xy[0] + sigma_xy[1] + sigma_xy[2]) / 3.0;
          // Rotate sigmas.
          Number a1 = c*sigma_xy_avg - s*sigma_xx_avg;
          Number a2 = c*sigma_yy_avg - s*sigma_xy_avg;
          Number rsigma_xy = c*a1 + s*a2;
          Number rsigma_yy = c*a2 - s*a1;
          // Compute matrix entries. Aij and Aji are symmetric.
          const Number x2_by_y2 = T::GNumberToNumber(cot);
          Aij_value = -( (x2_by_y2 * rsigma_yy)/2.0 - rsigma_xy/2.0 );
          if (sj1 >= 0) {
            // Compute diagonal entry at sj1. Note that this is a different
            // entry than the one computed for the isotropic case above, but
            // all the entries get covered in the end.
            // Note that x1/y2 = x1^2 / (x1*y2) == j2j0_length2 / area2
            diagonal[sj1] -= rsigma_yy *
                             T::GNumberToNumber(-j2j0_length2 / (area2 * 2.0));
          }
        }

        // Contribution of 'f' to right hand side.
        if (sj0 >= 0) {
          rhs[sj0] -= (T::PointF(i, j0) * 2.0 + T::PointF(i, j1) +
                       T::PointF(i, j2)) * T::GNumberToNumber(area2 / 24.0);
        }

        // Add contributions for triangle edges with robin boundary conditions,
        // i.e. grad u . n + alpha*u = beta.
        if (T::EdgeType(i, j0) == T::ROBIN) {
          // Points j0->j1 are on the boundary.
          GNumber side_length = sqrt(opplen2);          // Length j0->j1
          Number alpha0, alpha1, beta0, beta1;
          T::Robin(i, j0, j0, g0, &alpha0, &beta0);
          T::Robin(i, j0, j1, g1, &alpha1, &beta1);

          // For anisotropic materials scale alpha so the boundary conditions
          // make sense. @@@ This currently only results in a well-matched port
          // in Exy cavities and when the port is aligned with an anisotropy
          // principle axis.
          if (anisotropic) {
            // Use a propagation direction normal to the boundary.
            Point boundary = (pt[j1] - pt[j0]).normalized();
            auto px = boundary[1];      // px,py is a unit length vector
            auto py = -boundary[0];     //   normal to the boundary
            Number scale = sqrt(T::GNumberToNumber(px*px)*sigma_xx_avg +
                                T::GNumberToNumber(px*py*2.0)*sigma_xy_avg +
                                T::GNumberToNumber(py*py)*sigma_yy_avg);
            alpha0 = alpha0 * scale;
            alpha1 = alpha1 * scale;
            beta0 = beta0 * scale;
            beta1 = beta1 * scale;
          }

          Number sl = T::GNumberToNumber(side_length);
          Aij_value += (alpha0 + alpha1) * sl / 12.0;              // L(sj0,sj1)
          if (sj0 >= 0) {
            diagonal[sj0] += (alpha0 / 4.0 + alpha1 / 12.0) * sl;  // L(sj0,sj0)
            rhs[sj0] += (beta0 * 2.0 + beta1) * sl / 6.0;
          }
          if (sj1 >= 0) {
            diagonal[sj1] += (alpha1 / 4.0 + alpha0 / 12.0) * sl;  // L(sj1,sj1)
            rhs[sj1] += (beta0 + beta1 * 2.0) * sl / 6.0;
          }
        }

        // If requested add the dielectric forcing term. This is one way to get
        // discontinuous gradients at dielectric boundaries. This triangle must
        // have all 3 vertices represented in the solution. This does not yet
        // account for nonisotropic materials.
        if (T::AddDielectricForcingTerm() &&
            sj0 >= 0 && sj1 >= 0 && sj2 >= 0) {
          // This triangle must have a neighbor on edge j0, on the other side
          // of the dielectric boundary, otherwise there can't be a
          // discontinuity.
          int ni = T::Neighbor(i, j0);
          if (ni != -1) {
            // Discontinuous gradients are only produced where we have
            // discontinuities in epsilon. Detect this by looking for step
            // changes in g() across triangles.
            Number e1 = T::PointG(i, 0);   // \propto epsilon of this triangle
            Number e2 = T::PointG(ni, 0);  // \propto epsilon of neighbor
            // Require that all vertices have the same epsilon, as will happen
            // when we use Paint().
            if (e1 != e2 &&
                e1 == T::PointG(i, 1) && e1 == T::PointG(i, 2) &&
                e2 == T::PointG(ni, 1) && e2 == T::PointG(ni, 2)) {
              // Add contributions for the edge j0 --> j1.
              // Compute q=(e2-e1)/(e2+e1). Only if this edge is at an epsilon
              // discontinuity (i.e. a dielectric boundary) will q be nonzero.
              Number qfactor = (e2 - e1) / (e1 + e2);
              if (qfactor != Number(0)) {
                // Compute cot(theta) for the triangle internal angle opposite
                // the j0->j1 edge.
                Number cot0, cot1;              // cotN is angle at point jN
                Point d1 = pt[j1] - pt[j0];
                Point d2 = pt[j2] - pt[j0];
                cot0 = T::GNumberToNumber(d1.dot(d2) / area2);
                d1 = pt[j0] - pt[j1];
                d2 = pt[j2] - pt[j1];
                cot1 = T::GNumberToNumber(d1.dot(d2) / area2);
                // Add terms to the system matrix. Note that this is not
                // symmetric!
                diagonal[sj0] += qfactor * cot1 / 2.0;
                diagonal[sj1] += qfactor * cot0 / 2.0;
                Aij_value += qfactor * cot0 / 2.0;
                Aji_correction += qfactor * (cot1 - cot0) / 2.0;
                triplets.push_back(Triplet(sj0, sj2,
                    qfactor * T::GNumberToNumber(-opplen2 / (area2 * 2.0))));
                triplets.push_back(Triplet(sj1, sj2,
                    qfactor * T::GNumberToNumber(-opplen2 / (area2 * 2.0))));
                triplets_size += 2;     // Ensure check below passes
                if (create_Ctriplets) {
                  // @@@ It's not clear how well this works in eigensystems
                  // when we have to compute a separate matrix for C, since
                  // it's not clear if these additional terms go in the main
                  // system matrix or in C.
                  return false;
                }
              }
            }
          }
        }

        // Add off-diagonal matrix entries to triplets.
        if (sj0 >= 0 && sj1 >= 0) {
          if (!create_Ctriplets) {
            Aij_value += Cij_value;
          }
          // If Number is not the same as MNumber but the problem is
          // symmetric we might consider creating just a lower triangle here
          // and converting it to a full matrix in SolveSystem(). However
          // this is not likely to be a bottleneck.
          triplets.push_back(Triplet(sj0, sj1, Aij_value));
          triplets.push_back(Triplet(sj1, sj0, Aij_value + Aji_correction));
          if (create_Ctriplets) {
            Ctriplets.push_back(Triplet(sj0, sj1, Cij_value));
            Ctriplets.push_back(Triplet(sj1, sj0, Cij_value));
          }
        }
      }
    }

    // Add diagonal entries to triplets.
    for (int i = 0; i < system_size; i++) {
      triplets.push_back(Triplet(i, i, diagonal[i]));
      if (create_Ctriplets) {
        Ctriplets.push_back(Triplet(i, i, Cdiagonal[i]));
      }
    }
    // We may not have used all entries in triplets if we have Dirichlet
    // boundaries, but check that we didn't use more than we reserved.
    CHECK(triplets.size() <= triplets_size);
    return true;
  }

  // Factor and solve the system created by CreateSystem(). Return true on
  // success or false if the factorization failed. This only does work the
  // first time it is called, subsequent times it simply returns the same
  // return code as the first time.
  bool SolveSystem() override MUST_USE_RESULT {
    // Prerequisites.
    DoTrace trace(__func__);
    if (solvesystem_retval >= 0) {
      return solvesystem_retval;
    }
    if (CreateSystemNeedsCalling()) {
      if (!CreateSystem()) {
        return (solvesystem_retval = false);
      }
    }
    CHECK(Ctriplets.empty());   // 'triplets' must contain the whole problem

    // Initialize the FEM system matrix 'A' from the triplets. Convert from
    // Number to MNumber if necessary.
    Eigen::SparseMatrix<MNumber> A;
    GetSystemMatrix(triplets, &A);

    // Convert from Number to MNumber for the right hand side b, if necessary.
    const int system_size = reverse_index_map.size();
    const MNumberVector *b = 0;
    MNumberVector bstorage;
    if ((b = T::CastMNumberToNumberVector(&rhs)) == 0) {
      b = &bstorage;
      bstorage.resize(system_size);
      for (int i = 0; i < system_size; i++) {
        bstorage[i] = T::MNumberFromNumber(rhs[i]);
      }
    }

    // Factor 'A'. Return false if A can not be factored. Then solve.
    CHECK(!factorizer)
    factorizer = new Factorizer;
    CHECK(A.isCompressed());      // Otherwise factorizer might make a copy
    {
      DoTrace trace("Analyze");
      factorizer->analyzePattern(A);
    }
    {
      DoTrace trace("Factorize");
      factorizer->factorize(A);
    }
    if (factorizer->info() != Eigen::Success) {
      return (solvesystem_retval = false);
    }
    {
      DoTrace trace("Solve");
      solution = factorizer->solve(*b);
    }

    // Pad the solution vector with zeros as necessary.
    PadSolution(&solution);
    CHECK(solution.size() == T::NumPoints());
    return (solvesystem_retval = true);
  }

  // Compute the derivative of the solution with respect to some parameter.
  // Return true on success or false if factorization failed.
  bool ComputeSolutionDerivative(MNumberVector *solution_derivative)
                                 MUST_USE_RESULT {
    DoTrace trace(__func__);
    if (!SolveSystem()) {       // Also ensures triplets and RHS created
      return false;
    }

    // Multiply d(system_matrix)/dparameter by the existing solution. We do the
    // multiplication directly from 'triplets' without converting into sparse
    // matrix form, since we're just doing it once. Note that we multiply by
    // the solution vector that is padded with zeros so we need to go through
    // the index map.
    const int system_size = SystemSize();
    MNumberVector tmp(system_size);
    tmp.setZero();
    for (int i = 0; i < triplets.size(); i++) {
      MNumber deriv = T::Derivative(triplets[i].value());
      int col = reverse_index_map[triplets[i].col()];
      tmp[triplets[i].row()] -= deriv * solution[col];
    }

    // Add in d(right_hand_side)/dparameter.
    for (int i = 0; i < system_size; i++) {
      tmp[i] += T::Derivative(rhs[i]);
    }

    // Solve for the solution derivative.
    {
      DoTrace trace("Solve");
      *solution_derivative = factorizer->solve(tmp);
      PadSolution(solution_derivative);
    }
    return true;
  }

  // Compute eigenvalues and eigenvectors of the system matrix. See
  // LaplacianEigenSolver for the definition of sigma. This only does work the
  // first time it is called, subsequent times it simply returns the same
  // return code as the first time.
  bool EigenSystem(int eigenpair_count, double sigma) MUST_USE_RESULT {
    // Prerequisites.
    DoTrace trace(__func__);
    if (eigensystem_retval >= 0) {
      return eigensystem_retval;
    }
    if (!CreateSystem(true)) {
      return (eigensystem_retval = false);
    }
    CHECK(!Ctriplets.empty());

    // Initialize the FEM system matrix 'A' from the triplets. Convert from
    // Number to MNumber.
    Eigen::SparseMatrix<MNumber> A, B;
    GetSystemMatrix(triplets, &A);
    GetSystemMatrix(Ctriplets, &B);

    // Compute the smallest eigenvalues, with eigenvectors.
    eigensolver = new EigenSolver(A, &B, eigenpair_count, sigma);
    if (eigensolver->Status() != Eigen::Success) {
      return (eigensystem_retval = false);
    }

    // Check the number of returned eigenvalues and eigenvectors.
    const Eigen::VectorXd &val = eigensolver->GetEigenValues();
    CHECK(val.size() <= eigenpair_count);
    const Eigen::MatrixXd &vecs = eigensolver->GetEigenVectors();
    CHECK(vecs.rows() == SystemSize());
    CHECK(vecs.cols() <= eigenpair_count);

    // Check that if sigma<0 its magnitude is less than the lowest nonzero
    // eigenvalue. If its not this might have impacted convergence and
    // accuracy.
    CHECK(sigma <= 0);
    if (sigma < 0 && val.size() >= 2 && sigma < -0.5 * val[1]) {
      Warning("EigenSystem |sigma| (%e) too close to lambda1 (%e), accuracy "
              "might be affected", -sigma, val[1]);
    }

    // For debugging, check the eigenvalues and eigenvectors.
    if (kDebug()) {
      for (int i = 0; i < vecs.cols(); i++) {
        double error = (A * vecs.col(i) - val[i] * B * vecs.col(i)).norm();
        printf("lambda%d = %e, Ax-lBx error=%e\n", i, val[i], error);
      }
    }
    return (eigensystem_retval = true);
  }

  // Get eigenvalue 'n'. Clamp 'n' to the range of available values.
  MNumber GetEigenvalue(int n) const {
    CHECK(eigensolver);
    n = std::max(0, std::min(eigensolver->GetEigenValues().size() - 1, n));
    return eigensolver->GetEigenValues()[n];
  }

  // Get eigenvector 'n'. Clamp 'n' to the range of available values.
  template<class Tvec>
  void GetEigenvector(int n, Tvec *vec) const {
    CHECK(eigensolver);
    const Eigen::MatrixXd &vecs = eigensolver->GetEigenVectors();
    n = std::max(0, std::min(vecs.cols() - 1, n));
    vec->resize(T::NumPoints());
    for (int i = T::NumPoints() - 1; i >= 0 ; i--) {
      if (index_map[i] == -1) {
        (*vec)[i] = 0;
      } else {
        (*vec)[i] = vecs(index_map[i], n);
      }
    }
  }

  // For debug/test:
  const Eigen::MatrixXd GetRawEigenvector(int n) const {
    CHECK(eigensolver);
    return eigensolver->GetEigenVectors().col(n);
  }

  // Utility: Initialize the FEM system matrix 'A' from the triplets. Convert
  // from Number to MNumber.
  void GetSystemMatrix(const std::vector<Triplet> &triplets,
                       Eigen::SparseMatrix<MNumber> *A) {
    const int system_size = reverse_index_map.size();
    A->resize(system_size, system_size);
    typedef Eigen::Triplet<MNumber> Trip;
    vector<Trip> trips;
    trips.reserve(triplets.size());
    for (int i = 0; i < triplets.size(); i++) {
      trips.push_back(Trip(triplets[i].row(), triplets[i].col(),
                           T::MNumberFromNumber(triplets[i].value())));
    }
    A->setFromTriplets(trips.begin(), trips.end());
  }
};

}  // namespace FEM

#endif
