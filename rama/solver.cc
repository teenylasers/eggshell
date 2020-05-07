// Rama Simulator, Copyright (C) 2014-2020 Russell Smith.
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

#include "solver.h"
#include "../toolkit/gl_utils.h"
#include "../toolkit/gl_font.h"
#include "../toolkit/mat_file.h"
#include "../toolkit/testing.h"
#include "../toolkit/femsolver.h"
#include "../toolkit/shaders.h"
#include "../toolkit/thread.h"

const double kSpeedOfLight = 299792458;         // m/s
const int kFarFieldPoints = 500;                // Pattern points to compute

using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::VectorXcd;         // Eigen vector of complex double
using Eigen::Vector2cd;         // Eigen vector of two complex doubles

//***************************************************************************
// Add some operators that my_jet.h is missing.
// @@@ We can implement these inside my_jet.h a bit more efficiently.

namespace rama {
  inline JetComplex operator*(const JetComplex &a, double b) {
    return a * JetComplex(b);
  }

  inline JetComplex operator/(const JetComplex &a, double b) {
    return a / JetComplex(b);
  }
}

//***************************************************************************
// Interface to FEMSolver for Ez and Exy cavities.
//
// The FEM system matrix depends on the shape, mesh and config. The
// SimplicialLDLT factorizer can not be used because the system matrix is
// symmetric but not hermitian.
//
// Ports use a generalized Neumann ("Robin") boundary condition to specify an
// impedance match with an incoming wave. For a boundary x=0 and a plane wave
//     E = exp(i*k*(x*cos(theta) + y*sin(theta)))
// for x > 0 and angle of incidence theta, at the boundary we have:
//     dE/dx - i*k*cos(theta) * E = 0
// so in the boundary condition we have alpha = -i*k*cos(theta). This is only a
// perfect absorber for one value of cos(theta). For a free space radiation
// condition we could take theta=0 (normal incidence). In an Ez cavity, for a
// waveguide port TE10 mode we use:
//     dEz/dnormal + alpha*Ez = 0, where
//     alpha = -1i*beta_guide * (S11+1)/(S11-1)
// In an Exy cavity we use cos(theta)=1.

struct HelmholtzFEMProblem : FEM::FEMProblem {
  Solver *s;
  double vacuum_k2;
  explicit HelmholtzFEMProblem() : s(0), vacuum_k2(0) {}

  void LinkToSolver(Solver *_s) {
    s = _s;
    vacuum_k2 = s->ComputeKSquared();
  }

  typedef JetComplex Number;
  typedef JetNum GNumber;
  typedef Complex MNumber;
  typedef Eigen::Triplet<Number> Triplet;
  typedef Eigen::Matrix<GNumber, 2, 1> Point;   // Point x,y
  typedef Eigen::Matrix<Number, Eigen::Dynamic, 1> NumberVector;
  typedef Eigen::Matrix<MNumber, Eigen::Dynamic, 1> MNumberVector;
  typedef Eigen::SparseLU<Eigen::SparseMatrix<MNumber>,
                          Eigen::COLAMDOrdering<int> > Factorizer;
  bool AddDielectricForcingTerm() const {
    return false;  // We use sigma steps to achieve the same thing
    // Or: return s->config_.type == ScriptConfig::EXY;
  }
  MNumber MNumberFromNumber(Number n) const { return ToComplex(n); }
  Number GNumberToNumber(GNumber n) const { return Number(n); }
  MNumberVector *CastMNumberToNumberVector(NumberVector *v) { return 0; }
  int NumPoints() const { return s->points_.size(); }
  int NumTriangles() const { return s->triangles_.size(); }
  Point PointXY(int i) const { return s->points_[i].p * s->config_.unit; }
  int Triangle(int i, int j) const { return s->triangles_[i].index[j]; }
  int Neighbor(int i, int j) const { return s->triangles_[i].neighbor[j]; }

  Number PointG(int i, int j) const {
    // Compute the k^2 for this point based on its dielectric constant.
    // For EM cavities, k^2 = omega^2*mu_0*epsilon_0*epsilon_r = k0^2*epsilon_r.
    JetComplex epsilon;
    const Material &material = s->materials_[s->triangles_[i].material];
    if (!s->mat_params_.empty() && material.callback.Valid()) {
      // This material has a dielectric field.
      epsilon = s->mat_params_[s->triangles_[i].index[j]].epsilon;
    } else {
      epsilon = material.epsilon;
    }
    if (s->config_.schrodinger) {
      // For Schrodinger cavities, "epsilon" is actually the potential.
      epsilon = JetComplex(1.0) - epsilon / (2.0 * M_PI * s->frequency_);
    }
    return JetComplex(vacuum_k2) * epsilon;
  }

  Number PointF(int i, int j) const {
    const Material &material = s->materials_[s->triangles_[i].material];
    if (!s->mat_params_.empty() && material.callback.Valid()) {
      // This material has a dielectric field.
      return s->mat_params_[s->triangles_[i].index[j]].excitation;
    } else {
      return material.excitation;
    }
  }

  EType EdgeType(int i, int j) const {
    if (s->triangles_[i].neighbor[j] != -1) return INTERIOR;
    // DIRICHLET if EZ and non-port boundary, otherwise ROBIN.
    int p0 = s->triangles_[i].index[j];
    int p1 = s->triangles_[i].index[(j + 1) % 3];
    EdgeKind kind = s->points_[p0].e.SharedKind(s->points_[p1].e, 0, 0);
    if (s->config_.type == ScriptConfig::EZ && kind.IsDefault()) {
      return DIRICHLET;
    }
    return ROBIN;
  }

  void Robin(int i, int j, int point_number, const Number &k2,
             Number *alpha, Number *beta) const {
    // Note that k2 == PointG(i, point_number).
    JetComplex k = sqrt(k2);
    if (k2.real() < 0) {
      // For nonpropagating modes in Ez cavities we need a different branch
      // cut for the sqrt(k2) to ensure we get exponential decay at the port
      // boundary conditions and not exponential increase.
      k = -k;
    }

    // If the boundary parameters for this point have been precomputed by a
    // callback function then just return those.
    auto it = s->boundary_params_.find(Mesh::RobinArg(i, j, point_number));
    if (it != s->boundary_params_.end()) {
      *alpha = std::get<0>(it->second) * k;
      *beta = std::get<1>(it->second) * k;
      return;
    }

    // Identify the edge type and port (if any) for edge j.
    int p0 = s->triangles_[i].index[j];
    int p1 = s->triangles_[i].index[(j + 1) % 3];
    float dist1, dist2;
    EdgeKind kind = s->points_[p0].e.SharedKind(s->points_[p1].e,
                                                &dist1, &dist2);
    int port_number = kind.PortNumber();
    if (port_number) {
      float dist = (point_number == j) ? dist1 : dist2;
      // The Robin condition at the port is: (grad u) . n + alpha*u = beta
      // The value of beta is nonzero for the excited port only.
      JetComplex excitation = s->config_.PortExcitation(port_number);
      if (s->config_.type == ScriptConfig::EXY) {
        *alpha = JetComplex(0, 1) * k;          // Works for all ports and ABC
        *beta = JetComplex(0, 2.0) * excitation * k;
      } else if (s->config_.type == ScriptConfig::EZ) {
        JetComplex S11(0, 0);       // Desired S11 at port @@@ allow this to be changeable
        // Compute the propagation constant 'beta0' for the TE10 mode. It can
        // be imaginary for too-narrow ports. For ABC ports we assume large
        // port length so that normal-incidence waves are the ones absorbed.
        JetNum port_length = 1e9;   // Default for special ports like ABC
        if (port_number) {
          port_length = s->port_lengths_[port_number];
        }
        JetComplex effective_k2 = k2 - sqr(M_PI / port_length);
        JetComplex beta0 = sqrt(effective_k2);
        if (effective_k2.real() < 0) {
          // For nonpropagating modes at Ez ports we need a different
          // branch cut for the sqrt(k2) to ensure we get exponential decay
          // at the port boundary conditions and not exponential increase.
          beta0 = -beta0;
        }
        *alpha = JetComplex(0, -1) * beta0 *
                 (S11 + JetNum(1.0)) / (S11 - JetNum(1.0));
        *beta = JetComplex(0, (2.0 * sin(dist * M_PI)) * abs(*alpha)) *
                excitation;
      } else {
        Panic("Unsupported type");
      }
    } else if (kind.IsABC()) {
      *alpha = JetComplex(0, 1) * k;          // Works for all ports and ABC
      *beta = JetComplex(0);
    } else {
      // At non-port robin boundaries we have alpha = beta = 0.
      *alpha = JetComplex(0.0);
      *beta = JetComplex(0.0);
    }
  }

  bool AnisotropicSigma(int i, NumberVector *sigma_xx, NumberVector *sigma_yy,
                        NumberVector *sigma_xy) {
    const Material &material = s->materials_[s->triangles_[i].material];
    if (!s->mat_params_.empty() && material.callback.Valid()) {
      // This material has a material parameters field.
      bool is_isotropic = true;
      for (int j = 0; j < 3; j++) {
        is_isotropic &= s->mat_params_[s->triangles_[i].index[j]].is_isotropic;
      }
      if (is_isotropic) {
        return false;
      }
      for (int j = 0; j < 3; j++) {
        (*sigma_xx)[j] = s->mat_params_[s->triangles_[i].index[j]].sigma_xx;
        (*sigma_yy)[j] = s->mat_params_[s->triangles_[i].index[j]].sigma_yy;
        (*sigma_xy)[j] = s->mat_params_[s->triangles_[i].index[j]].sigma_xy;
      }
      return true;
    } else {
      if (material.is_isotropic) {
        return false;
      }
      for (int j = 0; j < 3; j++) {
        (*sigma_xx)[j] = material.sigma_xx;
        (*sigma_yy)[j] = material.sigma_yy;
        (*sigma_xy)[j] = material.sigma_xy;
      }
      return true;
    }
  }

  GNumber Absolute(const GNumber &a) { return abs(a); }
  MNumber Derivative(const Number &a) {
    return Complex(a.real().Derivative(), a.imag().Derivative());
  }
  typedef ::Trace DoTrace;
};

//***************************************************************************
// Interface to FEMSolver for waveguide mode computation.

struct WaveguideModeFEMProblem : FEM::FEMProblem {
  Solver *s;
  explicit WaveguideModeFEMProblem() : s(0) {}

  void LinkToSolver(Solver *_s) {
    s = _s;
  }

  typedef JetNum Number;
  typedef JetNum GNumber;
  typedef double MNumber;
  typedef Eigen::Triplet<Number> Triplet;
  typedef Eigen::Matrix<GNumber, 2, 1> Point;   // Point x,y
  typedef Eigen::Matrix<Number, Eigen::Dynamic, 1> NumberVector;
  typedef Eigen::Matrix<MNumber, Eigen::Dynamic, 1> MNumberVector;
  typedef Eigen::SimplicialLLT<Eigen::SparseMatrix<MNumber>, Eigen::Lower,
                               Eigen::AMDOrdering<int> > Factorizer;
  bool AddDielectricForcingTerm() const { return false; }
  MNumber MNumberFromNumber(Number n) const { return ToDouble(n); }
  Number GNumberToNumber(GNumber n) const { return n; }
  MNumberVector *CastMNumberToNumberVector(NumberVector *v) { return 0; }
  int NumPoints() const { return s->points_.size(); }
  int NumTriangles() const { return s->triangles_.size(); }
  Point PointXY(int i) const { return s->points_[i].p * s->config_.unit; }
  int Triangle(int i, int j) const { return s->triangles_[i].index[j]; }
  int Neighbor(int i, int j) const { return s->triangles_[i].neighbor[j]; }
  Number PointG(int i, int j) const { return Number(-1.0); }
  Number PointF(int i, int j) const { return Number(0.0); }
  EType EdgeType(int i, int j) const {
    if (s->triangles_[i].neighbor[j] != -1) return INTERIOR;
    return (s->config_.type == ScriptConfig::TE) ? ROBIN : DIRICHLET;
  }
  void Robin(int i, int j, int point_number, const Number &k2,
             Number *alpha, Number *beta) const {
    *alpha = 0.0;
    *beta = 0.0;
  }
  bool AnisotropicSigma(int i, NumberVector *sigma_xx, NumberVector *sigma_yy,
                        NumberVector *sigma_xy) {
    return false;
  }
  GNumber Absolute(const GNumber &a) { return abs(a); }
  MNumber Derivative(const Number &a) {
    return a.Derivative();
  }
  typedef ::Trace DoTrace;
};

//***************************************************************************
// Utility.

// Compute the gradient (Dx,Dy) of a triangle (p1,p2,p3) with the given vertex
// values.
template<class T>
static void TriangleGradient(const JetPoint &p1, const JetPoint &p2,
                             const JetPoint &p3, const T &value1,
                             const T &value2, const T &value3, T *Dx, T *Dy) {
  JetNum a = p1[0] - p3[0];
  JetNum b = p2[0] - p3[0];
  JetNum c = p1[1] - p3[1];
  JetNum d = p2[1] - p3[1];
  JetNum det = a * d - b * c;
  *Dx = (value3*(p1[1] - p2[1]) + value1*d - value2*c) / det;
  *Dy = (value3*(p2[0] - p1[0]) + value2*a - value1*b) / det;
}

// Compute f(x,y) = Dx*x + Dy*y + c, where f(x1,y1)=value1.
template<class T>
static inline void LinearFunction(JetNum x, JetNum y, T Dx, T Dy,
                                  const JetPoint &p1, const T &value1,
                                  T *result) {
  *result = Dx * x + Dy * y + value1 - (Dx * p1[0] + Dy * p1[1]);
}

// Linearly interpolate at x,y on the triangle (p1,p2,p3) with the given vertex
// values. If 'T' is a Jet-based type then derivatives will be propagated.
template<class T>
static void InterpolateTriangle(JetNum x, JetNum y,
                                const JetPoint &p1, const JetPoint &p2,
                                const JetPoint &p3,
                                const T &value1, const T &value2,
                                const T &value3, T *result) {
  T Dx, Dy;
  TriangleGradient(p1, p2, p3, value1, value2, value3, &Dx, &Dy);
  LinearFunction(x, y, Dx, Dy, p1, value1, result);
}

//***************************************************************************
// Incrementally compute A such that either A or A*sin(pi*x) is the best fit to
// a series of points x,y.

class Fitter {
 public:
  Fitter(bool fitsin) : fitsin_(fitsin), numer_(0), denom_(0) {}
  void Add(JetNum x, JetComplex y) {
    JetNum s = fitsin_ ? sin(M_PI * x) : 1;
    numer_ += y * s;
    denom_ += sqr(s);
  }
  JetComplex GetFit() const {
    return (denom_ == 0.0) ? JetNum(0.0) : (numer_ / denom_);
  }
 private:
  bool fitsin_;         // true to fit A*sin(pi*x), false to fit A
  JetComplex numer_;
  JetNum denom_;
};

//***************************************************************************
// ScriptConfig.

ScriptConfig::Type ScriptConfig::StringToType(const char *name) {
  if (name == 0)
    return UNKNOWN;
  if (strcmp(name, "S") == 0)
    return SCHRODINGER;
  if (strcmp(name, "Ez") == 0)
    return EZ;
  if (strcmp(name, "Exy") == 0)
    return EXY;
  if (strcmp(name, "TE") == 0)
    return TE;
  if (strcmp(name, "TM") == 0)
    return TM;
  return UNKNOWN;
}

//***************************************************************************
// Solver.

void Solver::Setup(int frequencies_index) {
  solver_ = 0;
  ed_solver_ = 0;
  mode_solver_ = 0;
  solver_solution_ = 0;
  frequency_ = 0;
  if (frequencies_index >= 0 &&
      frequencies_index < config_.frequencies.size()) {
    frequency_ = config_.frequencies[frequencies_index];
  }

  if (config_.TypeIsElectrodynamic()) {
    ed_solver_ = new EDSolverType;
    solver_ = ed_solver_;
    ed_solver_->LinkToSolver(this);
  } else {
    mode_solver_ = new ModeSolverType;
    solver_ = mode_solver_;
    mode_solver_->LinkToSolver(this);
  }
  if (!IsValidMesh()) {
    return;
  }

  // Compute port_lengths_ (the total length of all ports) from the mesh.
  // @@@ NOTE that the total length only makes sense for ports that are all one
  // piece, if a port N has been cut into two separate pieces then we need some
  // way to handle that, or disallow that situation.
  for (BoundaryIterator it(this); !it.done(); ++it) {
    int pnum = it.kind().PortNumber();
    if (pnum) {
      port_lengths_.resize(std::max(port_lengths_.size(), size_t(pnum + 1)));
      JetNum len = (points_[it.pindex1()].p - points_[it.pindex2()].p).norm();
      port_lengths_[pnum] += len;
    }
  }
  for (int i = 1; i < port_lengths_.size(); i++) {
    port_lengths_[i] *= config_.unit;
    if (fabs(port_lengths_[i].Derivative()) > 1e-9) {
      // The derivative is often not precisely zero when it's intended to be
      // due to numerical imprecision.
      ERROR_ONCE("Port length %d can not currently depend on optimized "
                 "parameters (len=%f,d/dp=%f).", i,
                 ToDouble(port_lengths_[i]), port_lengths_[i].Derivative());
    }
  }

  // Compute index maps.
  solver_->CreateIndexMaps();
  if (solver_->SystemSize() == 0) {
    // This might happen if we have a Dirichlet boundary and the mesh is 100%
    // boundary points so that there are no nonzero solution points. We could
    // actually return the zero solution just fine but for now it might confuse
    // other code. IsValid() will return false.
    Error("Can not solve zero sized system");
  }
}

Solver::Solver(const Shape &s, const ScriptConfig &config, Lua *lua,
               int frequencies_index)
    : Mesh(s, config.mesh_edge_length, lua), shape_(s), config_(config)
{
  Setup(frequencies_index);
}

Solver::Solver(Solver *solver, int frequencies_index)
    : Mesh(*solver), shape_(solver->shape_), config_(solver->config_)
{
  Setup(frequencies_index);
}

Solver::~Solver() {
  delete ed_solver_;
  delete mode_solver_;
  if (config_.TypeIsWaveguideMode()) {
    delete solver_solution_;
  }
}

bool Solver::IsValid() const {
  return Mesh::IsValidMesh() &&                 // Mesh must be valid
         solver_->SystemSize() > 0;             // System size must be > 0
}

bool Solver::SameAs(const Shape &s, const ScriptConfig &config, Lua *lua) {
  if (s != shape_ || config != config_) {
    return false;
  }
  // At this point the shape and config are the same. The material values and
  // material callback functions *might* be the same. The port callback
  // functions *might* be the same. However we don't know if the material
  // parameters field computed by the material callback functions are the same,
  // because those functions can use parameter values (and maybe upvalues) that
  // we can't see here. Similarly for the port callbacks. Therefore build a new
  // material parameters field and a new boundary parameters map, and compare
  // them with the existing ones. If the functions compute the same values,
  // then everything is actually the same.
  if (!mat_params_.empty()) {
    vector<MaterialParameters> d;
    DeterminePointMaterial(lua, &d);
    CHECK(mat_params_.size() == d.size());
    if (mat_params_ != d) {
      return false;
    }
  }
  if (!boundary_params_.empty()) {
    std::map<RobinArg, RobinRet> b;
    DetermineBoundaryParameters(lua, &b);
    if (boundary_params_ != b) {
      return false;
    }
  }
  return true;
}

bool Solver::UpdateDerivatives(const Shape &s) {
  Trace trace(__func__);
  if (!Solve()) {
    return false;
  }

  // Propagate updated shape derivatives to the mesh points.
  Mesh::UpdateDerivatives(s);

  // Recreate the system (this will use the updated derivatives in the mesh
  // points and materials).
  solver_->UnCreateSystem();            // Resets triplets and rhs
  if (!solver_->CreateSystem()) {
    return false;
  }

  // Recompute derivatives.
  solution_derivative_.resize(0);
  if (!ComputeDerivatives()) {
    return false;
  }
  return true;
}

void Solver::DrawSolution(DrawMode draw_mode, ColorMap::Function colormap,
                          int brightness, double phase_offset,
                          Solvers *solvers) {
  if (!Solve()) {
    return;
  }
  if (solvers && !solvers->Solve()) {
    return;
  }

  // Fix up some draw modes that don't make sense for a wideband pulse.
  if (solvers && draw_mode == DRAW_AMPLITUDE_REAL) {
    draw_mode = DRAW_AMPLITUDE;
  }

  // Create the phasor that all solution points get multiplied by.
  Complex phasor = exp(Complex(0, phase_offset));

  // If we have a single solver then use the already-computed solution. If we
  // have multiple solvers then add together all solutions, scaled by a
  // different phasor for each frequency. We do this on-demand, below.
  #define CREATE_SOLUTION \
    VectorXcd multi_solution; \
    if (solvers) { \
      solvers->CombinedField(&multi_solution, phase_offset); \
      phasor = 1; \
    } \
    const auto &solution = solvers ? multi_solution : *solver_solution_;

  // Ditto for the spatial gradient.
  #define CREATE_SPATIAL_GRADIENT \
    Eigen::MatrixXcd multi_Pgradient; \
    if (solvers) { \
      if (!solvers->CombinedSpatialGradient(&multi_Pgradient, phase_offset)) { \
        return; \
      } \
      phasor = 1; \
    } else {\
      if (!ComputeSpatialGradient()) return; \
    } \
    const auto &Pgradient = solvers ? multi_Pgradient : Pgradient_;

  // Ditto for the spatial gradient max amplitude.
  #define CREATE_SPATIAL_GRADIENT_MAX_AMPLITUDE \
    Eigen::VectorXd multi_Mgradient; \
    if (solvers) { \
      if (!solvers->CombinedSpatialGradientMaxAmplitude(&multi_Mgradient)) { \
        return; \
      } \
    } else { \
      if (!ComputeSpatialGradientMaxAmplitude()) return; \
    } \
    const auto &Mgradient = solvers ? multi_Mgradient : Mgradient_;

  // For electrodynamic cavities we scale the gradient so it is similar to the
  // scalar field. Assuming the scalar field has a form sin(2*pi*x/lambda_g),
  // the maximum gradient is 2*pi/lambda_g. In a rectangular waveguide
  // operating at 1.5 times the cutoff frequency we have
  // lambda_g=(3*c)/(sqrt(5)*f).
  // @@@ NOTE: This rescaling is frequency dependent, so colors can not be
  // directly compared across different frequencies.
  double gscale = 1;
  if (config_.TypeIsElectrodynamic()) {
    double max_gradient = (2 * sqrt(5) * M_PI * frequency_) /
                          (3*kSpeedOfLight);
    gscale = 1.0 / max_gradient;
  }

  // See if we're drawing vectors. If not we're drawing colored triangles.
  if (draw_mode == DRAW_GRADIENT_VECTORS ||
      draw_mode == DRAW_ROTATED_GRADIENT_VECTORS ||
      draw_mode == DRAW_POYNTING_VECTORS ||
      draw_mode == DRAW_POYNTING_VECTORS_TA) {
    CREATE_SPATIAL_GRADIENT
    // Compute scale.
    double arbitrary_scale = pow((brightness + 1) / 250.0, 3);

    gscale *= arbitrary_scale;
    // Draw the gradient vector from all mesh triangle vertices.
    gl::SetUniform("color", 0, 0, 1);
    vector<Vector3f> points;
    #define DRAWVECTOR(vx, vy) \
      points.push_back(Vector3f(ToDouble(points_[i].p[0]), \
                                 ToDouble(points_[i].p[1]), 0)); \
      points.push_back(Vector3f(ToDouble(points_[i].p[0]) + (vx) * gscale, \
                                ToDouble(points_[i].p[1]) + (vy) * gscale, 0));
    if (draw_mode == DRAW_GRADIENT_VECTORS) {
      for (int i = 0; i < points_.size(); i++) {
        Vector2cd gradient = Pgradient.row(i) * phasor;
        DRAWVECTOR(gradient[0].real(), gradient[1].real());
      }
    } else if (draw_mode == DRAW_ROTATED_GRADIENT_VECTORS) {
      for (int i = 0; i < points_.size(); i++) {
        Vector2cd gradient = Pgradient.row(i) * phasor;
        DRAWVECTOR(-gradient[1].real(), gradient[0].real());
      }
    } else if (draw_mode == DRAW_POYNTING_VECTORS) {
      CREATE_SOLUTION
      for (int i = 0; i < points_.size(); i++) {
        Complex field = solution[i] * phasor;               // In Z
        Vector2cd gradient = Pgradient.row(i) * phasor;    // In X,Y
        DRAWVECTOR(field.imag() * gradient[0].real(),
                   field.imag() * gradient[1].real());
      }
    } else {  // if draw_mode == DRAW_POYNTING_VECTORS_TA
      CREATE_SOLUTION
      for (int i = 0; i < points_.size(); i++) {
        DRAWVECTOR((conj(Pgradient(i, 0)) * solution[i]).imag(),
                   (conj(Pgradient(i, 1)) * solution[i]).imag());
      }
    }
    gl::Draw(points, GL_LINES);
    #undef DRAWVECTOR
  } else {
    // Create color map.
    const int kNumColors = 256;           // Should be even
    float rgb[kNumColors][3];
    for (int i = 0; i < kNumColors; i++) {
      colormap(float(i) / (kNumColors - 1), rgb[i]);
    }

    // Map the brightness to a scale used for min/max values.
    double scale = pow(10, -(brightness - 500.0) / 500.0);

    // Draw all mesh triangles.
    gl::PushShader push_shader(gl::SmoothShader());
    vector<Vector3f> points, colors;
    #define DRAWLOOP(compute_value, minval, maxval) \
      for (int i = 0; i < triangles_.size(); i++) { \
        for (int j = 0; j < 3; j++) { \
          int k = triangles_[i].index[j]; \
          double value = compute_value; \
          int c = std::max(0, std::min(kNumColors - 1, \
            int(round((value - minval) * (kNumColors / (maxval-minval)))))); \
          colors.push_back(Vector3f(rgb[c][0], rgb[c][1], rgb[c][2])); \
          points.push_back(Vector3f(ToDouble(points_[k].p[0]), \
                                    ToDouble(points_[k].p[1]), 0)); \
        } \
      }
    if (draw_mode == DRAW_REAL) {
      CREATE_SOLUTION
      DRAWLOOP(phasor.real() * solution[k].real() -
               phasor.imag() * solution[k].imag(), -scale, scale)
    } else if (draw_mode == DRAW_AMPLITUDE) {
      CREATE_SOLUTION
      DRAWLOOP(abs(solution[k]), 0.0, scale)
    } else if (draw_mode == DRAW_AMPLITUDE_REAL) {
      CREATE_SOLUTION
      DRAWLOOP(fabs(phasor.real() * solution[k].real() -
                    phasor.imag() * solution[k].imag()), 0.0, scale)
    } else if (draw_mode == DRAW_GRADIENT_AMPLITUDE) {
      scale /= gscale;
      CREATE_SPATIAL_GRADIENT_MAX_AMPLITUDE
      DRAWLOOP( Mgradient[k], 0.0, scale)
    } else if (draw_mode == DRAW_GRADIENT_AMPLITUDE_REAL) {
      scale /= gscale;
      CREATE_SPATIAL_GRADIENT
      DRAWLOOP( sqrt(sqr(phasor.real() * Pgradient(k, 0).real() -
                         phasor.imag() * Pgradient(k, 0).imag()) +
                     sqr(phasor.real() * Pgradient(k, 1).real() -
                         phasor.imag() * Pgradient(k, 1).imag())), 0.0, scale)
    } else {
      Panic("Unsupported DrawMode");
    }
    gl::Draw(points, colors, GL_TRIANGLES);
    #undef DRAWLOOP
  }
}

bool Solver::ComputePortOutgoingField1(vector<JetComplex> *result) {
  if (!config_.TypeIsElectrodynamic()) {
    return false;
  }
  Trace trace(__func__);

  // We depend on the solver solution and solution_derivative_ in SolutionJet
  // below, so make sure they're up to date.
  if (!Solve() || !ComputeDerivatives()) {
    return false;
  }

  // This subtracts the excitation from the field at the ports, then assumes a
  // TE10 E or H field distribution for what remains (i.e. a sine variation for
  // the E field in the Ez cavity and a constant H field for the Exy cavity).
  vector<Fitter> ports;         // Entry [0] corresponds to port 1
  for (BoundaryIterator it(this); !it.done(); ++it) {
    int pnum = it.kind().PortNumber();
    if (pnum) {
      while (ports.size() < pnum) {
        ports.push_back(Fitter(config_.type == ScriptConfig::EZ));
      }
      JetComplex excitation1 = config_.PortExcitation(pnum);
      JetComplex excitation2 = excitation1;
      if (config_.type == ScriptConfig::EZ) {
        excitation1 *= sin(M_PI * it.dist1());
        excitation2 *= sin(M_PI * it.dist2());
      }
      // We will end up counting each point in the middle twice but the
      // ends only once. For both kinds of fitters this does not affect the
      // result.
      JetComplex s1 = SolutionJet(it.pindex1());
      JetComplex s2 = SolutionJet(it.pindex2());
      ports[pnum-1].Add(it.dist1(), s1 - JetComplex(excitation1));
      ports[pnum-1].Add(it.dist2(), s2 - JetComplex(excitation2));
    }
  }
  result->resize(ports.size());
  for (int i = 0; i < ports.size(); i++) {
    (*result)[i] = ports[i].GetFit();
  }
  return true;
}

bool Solver::ComputePortOutgoingField2(vector<JetComplex> *result) {
  // Return for each port the field amplitude that, when squared and scaled in
  // ComputePortOutgoingPower() will correspond to the power exiting the port.
  // We compute this field amplitude by integrating the field^2 across the port
  // and scaling. When the field is a well behaved TE10 this gives the same
  // result as ComputePortOutgoingField2(). When the field is more complex,
  // this method tends to produce a less nonsensical number that is more in
  // line with the actual field strength at the port. Note that it will not be
  // the actual power exiting the port as we are not really integrating the
  // Poynting vector. Integrating the Poynting vector involves computing the
  // field derivative, which adds extra numerical noise.

  if (!config_.TypeIsElectrodynamic()) {
    return false;
  }
  Trace trace(__func__);

  // We depend on the solver solution and solution_derivative_ in SolutionJet
  // below, so make sure they're up to date.
  if (!Solve() || !ComputeDerivatives()) {
    return false;
  }

  // Integrate the phase and squared field across each port, first subtracting
  // any excitation field.
  vector<JetComplex> sum, sum2;
  for (BoundaryIterator it(this); !it.done(); ++it) {
    int pnum = it.kind().PortNumber();
    if (pnum) {
      sum.resize(std::max(pnum, int(sum.size())));
      sum2.resize(sum.size());
      JetComplex excitation1 = config_.PortExcitation(pnum);
      JetComplex excitation2 = excitation1;
      if (config_.type == ScriptConfig::EZ) {
        excitation1 *= sin(M_PI * it.dist1());
        excitation2 *= sin(M_PI * it.dist2());
      }
      JetComplex value1 = SolutionJet(it.pindex1()) - JetComplex(excitation1);
      JetComplex value2 = SolutionJet(it.pindex2()) - JetComplex(excitation2);
      JetComplex power1 = conj(value1) * value1;
      JetComplex power2 = conj(value2) * value2;
      JetComplex avg_power = (power1 + power2) / 2.0;
      sum[pnum - 1] += (value1 + value2) / 2.0;
      JetPoint p1 = points_[it.pindex1()].p;
      JetPoint p2 = points_[it.pindex2()].p;
      sum2[pnum - 1] += avg_power * (p2 - p1).norm() * config_.unit;
    }
  }
  result->resize(sum.size());
  for (int i = 0; i < sum.size(); i++) {
    if (config_.type == ScriptConfig::EZ) {
      sum2[i] *= 2.0;
    }
    (*result)[i] = sqrt(sum2[i] / port_lengths_[i + 1]);
    (*result)[i] *= sum[i] / abs(sum[i]);       // Use the phases in 'sum'
  }
  return true;
}

bool Solver::ComputePortOutgoingPower(vector<JetComplex> *result) {
  // Take the result of A=ComputePortOutgoingField() and compute the (complex)
  // power coming out of each port. The result is scaled to be proportional to
  // the actual waveguide power transferred out of each port. The scaling is
  // according to equation 3.92 in Pozar:
  //
  //   Power = frequency * mu * a^3 * |A|^2 * b * Re(beta) / (2*pi)
  //
  // This is dependent on the port length, frequency and propagation constant.
  // Note one subtlety: The modeled field for an Ez cavity is Ey in Pozar's
  // coordinate system, which we need to convert to Hz to use equation 3.92,
  // which results in a scaling by 'a' (or a^2 for power).

  if (!config_.TypeIsElectrodynamic()) {
    return false;
  }

  // Return any previously computed solution.
  if (!port_outgoing_power_.empty()) {
    *result = port_outgoing_power_;
    return true;
  }
  Trace trace(__func__);

  vector<JetComplex> field;
  if (!ComputePortOutgoingField2(&field)) {     // Select method 1 or 2
    return false;
  }
  result->resize(field.size());
  JetNum overall_scale = 0;
  for (int i = 0; i < field.size(); i++) {
    int port_number = i + 1;
    // The power compensation required depends on the cavity type.
    JetNum power_scale = 1;
    if (config_.type == ScriptConfig::EZ) {
      JetNum a = port_lengths_[port_number];     // Waveguide A-dimension
      double k = 2.0 * M_PI * frequency_ / kSpeedOfLight;
      JetComplex beta = sqrt(JetComplex(sqr(k) - sqr(M_PI / a)));
      power_scale = a * beta.real();
    } else if (config_.type == ScriptConfig::EXY) {
      // @@@ This does not account for depth differences at the ports - but if
      // the model contains depth changes the final field is approximate
      // anyway:
      power_scale = port_lengths_[port_number];  // Waveguide B-dimension
    }
    (*result)[i] = field[i] * abs(field[i]) * power_scale;
    if (config_.PortExcitation(port_number) != JetComplex(0.0)) {
      overall_scale = power_scale;
    }
  }

  // Final scaling so power scale of excited port is 1. The overall_scale might
  // be 0 if e.g. all ports in an Ez cavity are below cutoff.
  if (overall_scale != 0) {
    for (int i = 0; i < result->size(); i++) {
      (*result)[i] /= overall_scale;
    }
  }
  port_outgoing_power_ = *result;
  return true;
}

void Solver::GetField(JetNum x, JetNum y, JetComplex *value) {
  CHECK(Solve());
  CHECK(ComputeDerivatives());          // For SolutionJet()
  int t = FindTriangle(ToDouble(x), ToDouble(y));
  if (t < 0) {
    *value = 0;
    return;
  }
  int i0 = triangles_[t].index[0];
  int i1 = triangles_[t].index[1];
  int i2 = triangles_[t].index[2];
  JetComplex value0 = SolutionJet(i0);
  JetComplex value1 = SolutionJet(i1);
  JetComplex value2 = SolutionJet(i2);
  // Note that x,y and points_[].p have not been scaled by config_.unit here,
  // but this is okay because they are in the same units.
  InterpolateTriangle<JetComplex>
      (x, y, points_[i0].p, points_[i1].p, points_[i2].p,
       value0, value1, value2, value);
}

void Solver::GetFieldGradient(JetNum x, JetNum y,
                              JetComplex *dx, JetComplex *dy) {
  // @@@ Derivatives of the gradient are not computed correctly here!
  // Pgradient_ is a point gradient that contains a mix of gradients in
  // adjacent triangles. It is smooth and convenient for visualization but
  // tricky when propagating derivatives: we interpolate the three Pgradient_
  // values at the corners of a triangle, so we could be ultimately pulling in
  // 10 or more of the solution values in the neighborhood. We could instead
  // compute the gradient from just the triangle vertices. This would result in
  // a discontinuous gradient that is less accurate but easier to compute
  // derivatives for.
  CHECK(ComputeSpatialGradient());
  int t = FindTriangle(ToDouble(x), ToDouble(y));
  if (t < 0) {
    *dx = 0;
    *dy = 0;
    return;
  }
  int i0 = triangles_[t].index[0];
  int i1 = triangles_[t].index[1];
  int i2 = triangles_[t].index[2];
  // Note that x,y and points_[].p have not been scaled by config_.unit here,
  // but this is okay because they are in the same units.
  InterpolateTriangle<JetComplex>
      (x, y, points_[i0].p, points_[i1].p, points_[i2].p,
       Pgradient_(i0, 0), Pgradient_(i1, 0), Pgradient_(i2, 0), dx);
  InterpolateTriangle<JetComplex>
      (x, y, points_[i0].p, points_[i1].p, points_[i2].p,
       Pgradient_(i0, 1), Pgradient_(i1, 1), Pgradient_(i2 ,1), dy);
}

void Solver::GetFieldPoynting(JetNum x, JetNum y, JetPoint *poynting) {
  // @@@ See the comment for GetFieldGradient(). We compute the gradient within
  // a single triangle only, so that derivatives come along with the ride. This
  // produces a somewhat noisier poynting vector than if we had used a
  // Pgradient_-style gradient.
  CHECK(Solve());
  CHECK(ComputeDerivatives());
  int t = FindTriangle(ToDouble(x), ToDouble(y));
  if (t < 0) {
    poynting->setZero();
    return;
  }
  int i0 = triangles_[t].index[0];
  int i1 = triangles_[t].index[1];
  int i2 = triangles_[t].index[2];
  JetComplex value0 = SolutionJet(i0);
  JetComplex value1 = SolutionJet(i1);
  JetComplex value2 = SolutionJet(i2);
  JetComplex Dx, Dy, value;
  TriangleGradient(points_[i0].p, points_[i1].p, points_[i2].p,
                   value0, value1, value2, &Dx, &Dy);
  LinearFunction<JetComplex>(x, y, Dx, Dy, points_[i0].p, value0, &value);
  JetComplex k(config_.unit);
  (*poynting)[0] = (conj(Dx) * value / k).imag();
  (*poynting)[1] = (conj(Dy) * value / k).imag();
}

bool Solver::ComputeAntennaPattern(vector<double> *azimuth,
                                   vector<JetComplex> *field) {
  Trace trace(__func__);
  if (!config_.TypeIsElectrodynamic() || !ComputeSpatialGradient()) {
    return false;
  }

  // We depend on the solver solution and solution_derivative_ in SolutionJet
  // below, so make sure they're up to date.
  if (!Solve() || !ComputeDerivatives()) {
    return false;
  }

  // If k^2 <= 0 then there is no wave propagation in this cavity, so an
  // antenna pattern does not make sense. Otherwise, plane waves will have the
  // form exp(i*k*(x*cos(theta)+y*sin(theta)).
  const double k2 = ComputeKSquared();
  if (k2 <= 0) {
    return false;
  }
  const double k = sqrt(k2);

  // Return a previously cached result.
  if (!antenna_field_.empty()) {
    *azimuth = antenna_azimuth_;
    *field = antenna_field_;
    return true;
  }

  // Precompute equally spaced azimuths for far field result.
  azimuth->resize(kFarFieldPoints);
  for (int i = 0; i < kFarFieldPoints ; i++) {
    (*azimuth)[i] = (2.0 * M_PI * double(i)) / kFarFieldPoints - M_PI;
  }

  // For all boundary triangles find field values and field gradients, then
  // update the far field. See near_to_far_field.nb for the rationale. There is
  // approximation error because the field and gradient are sampled at discrete
  // points on a piecewise linear boundary. The error shows up as imperfect
  // back-lobe cancellation for plane waves. The positions of the isotropic and
  // dipole radiators have a big influence on this. The best results seem to be
  // obtained when both kinds of radiators are positioned at the centroid of
  // boundary triangles, with field values and gradients computed from the
  // solution values at the triangle vertices.
  field->clear();
  field->resize(kFarFieldPoints);       // Far field values over angle
  int radiator_count = 0;
  bool use_ff_material = (config_.antenna_pattern == config_.AT_FF_MATERIAL);
  uint32_t color_mask = use_ff_material ? Material::FAR_FIELD : 0;
  for (BoundaryIterator it(this, color_mask); !it.done(); ++it) {
    if (!( use_ff_material || it.kind().IsABC() ||
           (config_.antenna_pattern == config_.AT_BOUNDARY) )) {
      continue;
    }
    radiator_count++;

    // Triangle vertices scaled to meters.
    JetPoint p1 = points_[it.pindex1()].p * config_.unit;
    JetPoint p2 = points_[it.pindex2()].p * config_.unit;
    JetPoint p3 = points_[it.pindex3()].p * config_.unit;

    // Triangle centroid.
    JetPoint center = (p1 + p2 + p3) / 3.0;

    // Interpolate the solution to the center of the triangle, and compute the
    // triangle gradient.
    JetComplex z, gradX, gradY;
    {
      JetComplex z1 = SolutionJet(it.pindex1());
      JetComplex z2 = SolutionJet(it.pindex2());
      JetComplex z3 = SolutionJet(it.pindex3());
      z = (z1 + z2 + z3) / JetComplex(3.0);
      TriangleGradient(p1, p2, p3, z1, z2, z3, &gradX, &gradY);
    }

    // Find the the normal to the boundary edge (outward-pointing).
    JetPoint normal(p1[1] - p2[1], p2[0] - p1[0]);
    normal.normalize();
    // Ensure the normal is outward-pointing by making sure that the dot
    // product of p1->p3 and the normal is negative.
    if (normal.dot(p3 - p1) > 0) {
      normal *= -1;
    }
    JetNum nangle = atan2(normal[1], normal[0]);

    // Update far field values.
    for (int i = 0; i < kFarFieldPoints; i++) {
      double phi = -(*azimuth)[i] + config_.boresight * M_PI / 180.0;
      (*field)[i] += (k*cos(phi - nangle)*z +
                JetComplex(0,1) * (sin(nangle)*gradY + cos(nangle)*gradX)) *
          exp(JetComplex(0, k * (center[0] * cos(phi) + center[1] * sin(phi))));
    }
  }

  // Scale field by the number of radiators, i.e. the number of times we added
  // to each element of field[] in the inner loop.
  for (int i = 0; i < kFarFieldPoints; i++) {
    (*field)[i] /= double(radiator_count);
  }

  // Cache the result (note that the return vectors might already be the cache
  // vectors).
  antenna_azimuth_ = *azimuth;
  antenna_field_ = *field;
  return true;
}

void Solver::AdjustAntennaPhaseCenter(JetPoint phase_center,
                                      const vector<double> &azimuth,
                                      vector<JetComplex> *field) {
  CHECK(field->size() == azimuth.size());
  const double k2 = ComputeKSquared();
  if (k2 <= 0) {
    return;
  }
  const double k = sqrt(k2);
  for (int i = 0; i < azimuth.size(); i++) {
    double phi = azimuth[i] + config_.boresight * M_PI / 180.0;
    (*field)[i] *= exp(JetComplex(0, -k *
            (cos(phi) * phase_center[0] + sin(phi) * phase_center[1])));
  }
}

bool Solver::LookupAntennaPattern(JetNum theta, JetNum *magnitude) {
  if (!ComputeAntennaPattern(&antenna_azimuth_, &antenna_field_)) {
    return false;
  }
  CHECK(antenna_azimuth_.size() == kFarFieldPoints);
  CHECK(antenna_field_.size() == kFarFieldPoints);
  // Search the azimuth array for theta. Account for the wrap-around gap
  // between the last and first azimuth.
  theta = NormalizeAngle(theta);
  auto it = std::lower_bound(antenna_azimuth_.begin(),
                             antenna_azimuth_.end(), theta);
  int i = (it - antenna_azimuth_.begin()) % antenna_azimuth_.size();
  int i0 = (i + antenna_azimuth_.size() - 1) % antenna_azimuth_.size();
  JetNum alpha = NormalizeAngle(theta - antenna_azimuth_[i0]) /
                 NormalizeAngle(antenna_azimuth_[i] - antenna_azimuth_[i0]);
  CHECK(alpha >= 0 && alpha <= 1);
  // Linearly interpolate the power (magnitude^2), not the field magnitude.
  *magnitude = sqrt(
                 sqr(abs(antenna_field_[i0])) +
                 alpha * sqr(abs(antenna_field_[i]) - abs(antenna_field_[i0]))
               );
  return true;
}

void Solver::SaveMeshAndSolutionToMatlab(const char *filename) {
  if (!Solve()) {
    return;
  }
  MatFile mat(filename);

  // Write mesh points.
  {
    int dims[2] = {(int) points_.size(), 2};
    vector<double> p(2 * points_.size());
    for (int i = 0; i < points_.size(); i++) {
      p[i] = ToDouble(points_[i].p[0]);
      p[i + points_.size()] = ToDouble(points_[i].p[1]);
    }
    mat.WriteMatrix("p", 2, dims, MatFile::mxDOUBLE_CLASS, p.data(), 0);
  }

  // Write triangles.
  {
    int dims[2] = {(int) triangles_.size(), 3};
    vector<int32> t(3 * triangles_.size());
    for (int i = 0; i < triangles_.size(); i++) {
      for (int j = 0; j < 3; j++) {
        t[i + j*triangles_.size()] = triangles_[i].index[j] + 1;
      }
    }
    mat.WriteMatrix("t", 2, dims, MatFile::mxINT32_CLASS, t.data(), 0);
  }

  // Write solution.
  {
    const auto &solution = *solver_solution_;
    int dims[2] = {(int) solution.size(), 1};
    vector<double> sr(solution.size()), si(solution.size());
    for (int i = 0; i < solution.size(); i++) {
      sr[i] = solution[i].real();
      si[i] = solution[i].imag();
    }
    mat.WriteMatrix("u", 2, dims, MatFile::mxDOUBLE_CLASS,
                    sr.data(), si.data());
  }

  // Write mesh dielectric properties (if available).
  if (!mat_params_.empty()) {
    int dims[2] = {(int) mat_params_.size(), 1};
    vector<double> pr(mat_params_.size()), pi(mat_params_.size());
    for (int i = 0; i < mat_params_.size(); i++) {
      pr[i] = ToDouble(mat_params_[i].epsilon.real());
      pi[i] = ToDouble(mat_params_[i].epsilon.imag());
    }
    mat.WriteMatrix("pp", 2, dims, MatFile::mxDOUBLE_CLASS,
                    pr.data(), pi.data());
  }

  if (!mat.Valid()) {
    // An error will already have been emitted.
  }
}

bool Solver::Solve() {
  if (config_.TypeIsElectrodynamic()) {
    bool status = ed_solver_->SolveSystem();
    solver_solution_ = &ed_solver_->solution;
    return status;
  } else if (config_.TypeIsWaveguideMode()) {
    // For TE modes the A matrix in the eigenproblem will be singular so we
    // need an estimate 'sigma' of the lowest nonzero eigenvalue, for the
    // ARPACK shift-and-invert method. For the estimate we use the bounding box
    // of the model as a rectangular cavity and compute its lowest order mode.
    // The estimate that we feed to the eigensolver is 1% of this mode, in case
    // the actual model has extra-low frequency modes due to very windy
    // internal channels.
    double sigma = 0;
    if (config_.type == ScriptConfig::TE) {
      JetNum min_x, min_y, max_x, max_y;
      shape_.GetBounds(&min_x, &min_y, &max_x, &max_y);
      double width = ToDouble(max_x - min_x);
      double height = ToDouble(max_y - min_y);
      double lambda1_estimate =
          sqr(M_PI) / sqr(std::max(width, height) * config_.unit);
      sigma = -lambda1_estimate * 0.01;
    }

    if (mode_solver_->EigenSystem(std::max(1, config_.max_modes), sigma)) {
      // Eigenvectors are stored in locally allocated space for display.
      if (!solver_solution_) {
        solver_solution_ = new VectorXcd;
        mode_solver_->GetEigenvector(0, solver_solution_);
        // Scale the mode solution so that largest magnitude is 1.
        (*solver_solution_) /= solver_solution_->cwiseAbs().maxCoeff();
      }
      return true;
    } else {
      Error("Waveguide mode solver failed");
      return false;
    }
  } else {
    return false;
  }
}

void Solver::SelectWaveguideMode(int n) {
  // Make sure we have an eigen solution available. Don't do anything if we're
  // not computing waveguide modes or if the mode solver has failed.
  if (!config_.TypeIsWaveguideMode() || !Solve()) {
    return;
  }

  // Get the eigenvector. The eigenvector is real valued but for compatibility
  // with the rest of this code we convert it into a complex vector.
  n = std::max(0, std::min(config_.max_modes - 1, n));
  mode_solver_->GetEigenvector(n, solver_solution_);

  // Scale the mode solution so that largest magnitude is 1 (for display).
  (*solver_solution_) /= solver_solution_->cwiseAbs().maxCoeff();

  // Invalidate any gradients already computed for other modes.
  Pgradient_.resize(0, 0);
  Mgradient_.resize(0);
}

bool Solver::ComputeModeCutoffFrequencies(vector<JetComplex> *cutoff) {
  if (!config_.TypeIsWaveguideMode() || !Solve()) {
    return false;
  }
  cutoff->resize(config_.max_modes);
  for (int i = 0; i < config_.max_modes; i++) {
    // For TE modes the lowest eigenvalue is zero, but numerical error might
    // give it some other small value. Force it to zero.
    double lambda = (i == 0 && config_.type == ScriptConfig::TE) ? 0 :
                    mode_solver_->GetEigenvalue(i);
    cutoff->at(i) = kSpeedOfLight * sqrt(lambda) / (2.0 * M_PI);
  }
  return true;
}

bool Solver::ComputeSpatialGradient() {
  if (Pgradient_.rows() > 0) {
    // Assume that we've already computed the gradient.
    return true;
  }
  Trace trace(__func__);
  if (!Solve()) {
    return false;
  }
  CHECK(triangles_.size() > 0 && points_.size() > 0);
  vector<Vector2cd> Tgradient;
  Tgradient.resize(triangles_.size());
  Pgradient_.resize(points_.size(), 2);
  Pgradient_.setZero();

  // Compute the gradient at all triangles.
  const auto &solution = *solver_solution_;
  for (int i = 0; i < triangles_.size(); i++) {
    int pj0 = triangles_[i].index[0];
    int pj1 = triangles_[i].index[1];
    int pj2 = triangles_[i].index[2];
    Eigen::Vector2d p0 = ToVector2d(points_[pj0].p);
    Eigen::Vector2d d1 = (ToVector2d(points_[pj1].p) - p0) * config_.unit;
    Eigen::Vector2d d2 = (ToVector2d(points_[pj2].p) - p0) * config_.unit;
    Complex b0 = solution[pj0];
    Complex b1 = solution[pj1];
    Complex b2 = solution[pj2];
    Complex denom = 1.0 / (d2[0]*d1[1] - d1[0]*d2[1]);
    Tgradient[i][0] = -(b0*(d1[1] - d2[1]) + b1*d2[1] - b2*d1[1]) * denom;
    Tgradient[i][1] =  (b0*(d1[0] - d2[0]) + b1*d2[0] - b2*d1[0]) * denom;
  }

  // Distribute the triangle gradients to triangle vertices.
  vector<int> count(points_.size());
  for (int i = 0; i < triangles_.size(); i++) {
    int pj0 = triangles_[i].index[0];
    int pj1 = triangles_[i].index[1];
    int pj2 = triangles_[i].index[2];
    Pgradient_.row(pj0) += Tgradient[i];
    Pgradient_.row(pj1) += Tgradient[i];
    Pgradient_.row(pj2) += Tgradient[i];
    count[pj0]++;
    count[pj1]++;
    count[pj2]++;
  }
  for (int i = 0; i < points_.size(); i++) {
    Pgradient_.row(i) /= count[i];
  }

  // For waveguide modes the scaling is arbitrary.
  if (config_.TypeIsWaveguideMode()) {
    Pgradient_ /= Pgradient_.cwiseAbs().maxCoeff();
  }
  return true;
}

bool Solver::ComputeSpatialGradientMaxAmplitude() {
  if (Mgradient_.size() > 0) {
    // Assume that we've already computed the gradient.
    return true;
  }
  Trace trace(__func__);
  if (!ComputeSpatialGradient()) {
    return false;
  }
  Mgradient_.resize(Pgradient_.rows());
  for (int i = 0; i < Pgradient_.rows(); i++) {
    Complex gx = Pgradient_(i, 0);
    Complex gy = Pgradient_(i, 1);
    double q = -gy.imag()*gy.real() - gx.imag()*gx.real();
    double r = (sqr(gy.real()) - sqr(gy.imag()) +
                sqr(gx.real()) - sqr(gx.imag())) / 2;
    double theta = atan2(q, r) / 2;
    Mgradient_[i] = sqrt(sqr(cos(theta)*gx.real() - sin(theta)*gx.imag()) +
                         sqr(cos(theta)*gy.real() - sin(theta)*gy.imag()));
  }
  return true;
}

double Solver::ComputeKSquared() {
  double k2 = 0;
  double lambda = kSpeedOfLight / frequency_;
  if (config_.schrodinger) {
    k2 = 2.0 * M_PI * frequency_;
  } else if (config_.type == ScriptConfig::EXY) {
    // For Exy cavities we need to compute the effective k, which is a function
    // of the depth. If k^2 is negative then k is imaginary and travelling
    // waves will not propagate at this frequency.
    k2 = sqr(2.0 * M_PI / lambda) - sqr(M_PI / (config_.depth * config_.unit));
  } else if (config_.type == ScriptConfig::EZ) {
    k2 = sqr(2.0 * M_PI / lambda);
  } else {
    Panic("Unsupported type");
  }
  return k2;
}

bool Solver::ComputeDerivatives() {
  if (!config_.TypeIsElectrodynamic()) {
    // Currently we only know how to compute derivatives for electrodynamic
    // cavities.
    return false;
  }
  if (solution_derivative_.size() > 0) {
    // Derivatives already computed.
    return true;
  }
  if (!ed_solver_->ComputeSolutionDerivative(&solution_derivative_)) {
    return false;
  }

  // Clear out things that depend on solution_derivative_ so that they'll be
  // regenerated on demand.
  port_outgoing_power_.clear();
  return true;
}

JetComplex Solver::SolutionJet(int i) const {
  JetNum realpart = (*solver_solution_)[i].real();
  JetNum imagpart = (*solver_solution_)[i].imag();
  realpart.Derivative() = solution_derivative_[i].real();
  imagpart.Derivative() = solution_derivative_[i].imag();
  return JetComplex(realpart, imagpart);
}

//***************************************************************************
// Solvers.

const int kNumThreads = 8;

bool Solvers::Solve() {
  // If there is no work for the Solver to do, we can run this paraller solve
  // several thousands times per second (at 50 solves per iteration). That is
  // the thread setup overhead, and it seems acceptable compared to the other
  // overheads. If it becomes a problem we might want to first check if there
  // is any actual solving work to do before launching all these threads.
  bool ok = true;
  ParallelFor(0, solvers_.size()-1, kNumThreads, [&](int i) mutable {
    if (!solvers_[i]->Solve()) {
      ok = false;
    }
  });
  return ok;
}

bool Solvers::UpdateDerivatives(const Shape &s) {
  bool ok = true;
  ParallelFor(0, solvers_.size()-1, kNumThreads, [&](int i) mutable {
    if (!solvers_[i]->UpdateDerivatives(s)) {
      ok = false;
    }
  });
  return ok;
}

void Solvers::CombinedField(VectorXcd *f, double phase_offset) {
  if (solvers_.empty() || solvers_[0]->solver_solution_->size() == 0) {
    return;
  }
  vector<Complex> phasors;
  Phasors(phase_offset, &phasors);
  f->resize(solvers_[0]->solver_solution_->size());
  f->setZero();
  for (int i = 0; i < solvers_.size(); i++) {
    *f += *solvers_[i]->solver_solution_ * phasors[i];
  }
}

bool Solvers::CombinedSpatialGradient(Eigen::MatrixXcd *f,
                                      double phase_offset) {
  if (solvers_.empty() || solvers_[0]->solver_solution_->size() == 0) {
    return false;
  }
  vector<Complex> phasors;
  Phasors(phase_offset, &phasors);
  for (int i = 0; i < solvers_.size(); i++) {
    if (!solvers_[i]->ComputeSpatialGradient()) {
      return false;
    }
  }
  f->resize(solvers_[0]->Pgradient_.rows(), solvers_[0]->Pgradient_.cols());
  f->setZero();
  for (int i = 0; i < solvers_.size(); i++) {
    *f += solvers_[i]->Pgradient_ * phasors[i];
  }
  return true;
}

bool Solvers::CombinedSpatialGradientMaxAmplitude(Eigen::VectorXd *f) {
  if (solvers_.empty() || solvers_[0]->solver_solution_->size() == 0) {
    return false;
  }
  for (int i = 0; i < solvers_.size(); i++) {
    if (!solvers_[i]->ComputeSpatialGradientMaxAmplitude()) {
      return false;
    }
  }
  f->resize(solvers_[0]->Mgradient_.size());
  f->setZero();
  for (int i = 0; i < solvers_.size(); i++) {
    *f += solvers_[i]->Mgradient_;      // Note that phasor not used (assumed 1)
  }
  *f /= solvers_.size();
  return true;
}

// Return a vector of complex numbers to scale each solution by.
void Solvers::Phasors(double phase_offset, vector<Complex> *n) {
  // Compute the center frequency.
  double fcenter = 0;
  for (int i = 0; i < solvers_.size(); i++) {
    fcenter += solvers_[i]->frequency_ / solvers_.size();
  }

  // Compute phasors.
  auto window = solvers_[0]->config_.wideband_window;
  n->resize(solvers_.size());
  for (int i = 0; i < solvers_.size(); i++) {
    (*n)[i] = exp(Complex(0, phase_offset * solvers_[i]->frequency_ / fcenter))
              / double(solvers_.size());
    if (window == ScriptConfig::HAMMING) {
      double scale = 0.5 - 0.5*cos((i+1) * 2*M_PI / (solvers_.size()+1));
      (*n)[i] *= scale;
    }
  }
}

//***************************************************************************
// Testing.

TEST_FUNCTION(InterpolateTriangle) {
  for (int i = 0; i < 1000; i++) {
    // Random triangle.
    JetPoint p1(RandDouble(), RandDouble());
    JetPoint p2(RandDouble(), RandDouble());
    JetPoint p3(RandDouble(), RandDouble());
    JetNum value1 = RandDouble();
    JetNum value2 = RandDouble();
    JetNum value3 = RandDouble();
    // Random barycentric coordinates, which are easy to interpolate from.
    double lambda1 = RandDouble();
    double lambda2 = RandDouble();
    double lambda3 = 1.0 - lambda1 - lambda2;
    // Convert barycentric coordinates to Cartesian coordinates.
    JetPoint xy = lambda1 * p1 + lambda2 * p2 + lambda3 * p3;
    // Interpolate.
    JetNum result;
    InterpolateTriangle<JetNum>(xy[0], xy[1], p1, p2, p3,
                                value1, value2, value3, &result);
    JetNum expected = lambda1 * value1 + lambda2 * value2 + lambda3 * value3;
    CHECK(fabs(result - expected) < 1e-9);
  }
}

TEST_FUNCTION(GetField_and_Friends) {
  // Create a simple simulation: a short section of WR-12 waveguide at 70 GHz.
  Shape s;
  s.AddPoint(0, 0);
  s.AddPoint(500, 0);
  s.AddPoint(500, 120);
  s.AddPoint(0, 120);
  CHECK(s.AssignPort(0, 3, EdgeKind(1)));       // Edge 3 has port number 1
  CHECK(s.AssignPort(0, 1, EdgeKind(2)));       // Edge 1 has port number 2
  ScriptConfig config;
  config.type = ScriptConfig::EZ;
  config.unit = 2.54e-5;
  config.mesh_edge_length = 10;
  config.port_excitation.resize(2);
  config.port_excitation[0] = 1;
  config.frequencies.push_back(70e9);

  // Solve.
  Solver solver(s, config, NULL, 0);

  // Check the interpolated solution matches what we expect.
  double max_value_error = 0, max_dx_error = 0, max_dy_error = 0,
         max_dx = 0, max_dy = 0, max_perror_x = 0, max_perror_y = 0;
  for (int i = 1; i < 120; i++) {
    for (int j = 1; j < 500; j++) {
      JetComplex value, dx, dy;
      JetPoint poynting;
      solver.GetField(j, i, &value);
      solver.GetFieldGradient(j, i, &dx, &dy);
      solver.GetFieldPoynting(j, i, &poynting);

      double xx = double(j - 1) / 498.0;
      double yy = double(i) / 120.0;
      const double p1[5] = {0.9961, 0.0022, 13.1368, 0.0006, 3.1410};
      double value_error = p1[0] * cos((xx + p1[1]) * p1[2]) *
          sin((yy + p1[3]) * p1[4]) - ToDouble(value.real());
      const double p2[5] = {-13.1544, 13.0536, 0.0054, 3.1228, 0.0038};
      double dx_error = (p2[0] * sin(p2[1] * (p2[2] + xx)) *
          sin(p2[3] * (p2[4] + yy))) / (500.0 * config.unit)
          - ToDouble(dx.real());
      const double p3[5] = {3.0986, 13.1435, 0.0021, 3.1118, 0.0019};
      double dy_error = (p3[0] * cos(p3[1] * (p3[2] + xx)) *
          cos(p3[3] * (p3[4] + yy))) / (120.0 * config.unit)
          - ToDouble(dy.real());
      double perror_x = 1030 * (0.5 - 0.5*cos(yy * 2.0 * M_PI))
          - ToDouble(poynting[0]);
      double perror_y = ToDouble(poynting[1]);
      max_value_error = std::max(max_value_error, fabs(value_error));
      max_dx_error = std::max(max_dx_error, fabs(dx_error));
      max_dy_error = std::max(max_dy_error, fabs(dy_error));
      max_dx = std::max(max_dx, fabs(ToDouble(dx.real())));
      max_dy = std::max(max_dy, fabs(ToDouble(dy.real())));
      max_perror_x = std::max(max_perror_x, fabs(perror_x));
      max_perror_y = std::max(max_perror_y, fabs(perror_y));
    }
  }
  CHECK(max_value_error < 0.008);
  CHECK(max_dx_error / max_dx < 0.065);
  CHECK(max_dy_error / max_dy < 0.04);
  CHECK(max_perror_x < 120);    //@@@ Can tighten up these limits if
  CHECK(max_perror_y < 120);    //    GetFieldPoynting() uses smoother gradient
}
