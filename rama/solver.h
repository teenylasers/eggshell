
// The computational domain.

#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <string.h>
#include "../toolkit/myvector"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "shape.h"
#include "mesh.h"
#include "../toolkit/colormaps.h"

using std::vector;

// Forward declarations to avoid bringing in femsolver.h.
namespace FEM {
  class FEMSolverBase;
  template<class T> class FEMSolver;
}
struct HelmholtzFEMProblem;
struct WaveguideModeFEMProblem;

// Solver configuration. These values are extracted from the script's 'config'
// table. This is everything required to generate a solution that is not
// related to the geometry of the computational domain. -1 for any of these
// values means 'not valid'.
struct ScriptConfig {
  // The EZ type is also for Schrodinger constant frequency cavities.
  enum Type {
    UNKNOWN = -1,               // This value is assumed by some code
    EZ,                         // Electrodynamic, E field in Z direction
    EXY,                        // Electrodynamic, E field in XY direction
    TE,                         // Waveguide TE modes
    TM,                         // Waveguide TM modes
    SCHRODINGER,                // Returned by StringToType but changed to EZ
  };

  Type type;                    // Cavity type: EZ, EXY etc.
  bool schrodinger;             // EZ cavities for Schrodinger simulation
  double unit;                  // One script-distance-unit is this many meters
  double mesh_edge_length;      // In units of 'unit'
  int mesh_refines;
  vector<JetNum> port_excitation;  // Magnitudes and phases of port excitations
  double frequency;             // In Hz
  double depth;                 // In units of 'unit'
  double boresight;             // Boresight angle for plotting antenna patterns
  int max_modes;                // TE or TM: Number of modes to compute

  ScriptConfig() {
    type = UNKNOWN;
    schrodinger = false;
    unit = -1;
    mesh_edge_length = -1;
    mesh_refines = -1;
    frequency = -1;
    depth = -1;
    boresight = 0;
    max_modes = 1;
  }

  bool operator==(const ScriptConfig &c) const {
    return type             == c.type
        && schrodinger      == c.schrodinger
        && unit             == c.unit
        && mesh_edge_length == c.mesh_edge_length
        && mesh_refines     == c.mesh_refines
        && port_excitation  == c.port_excitation
        && frequency        == c.frequency
        && depth            == c.depth
        && boresight        == c.boresight
        && max_modes        == c.max_modes;
  }
  bool operator!=(const ScriptConfig &c) const { return !operator==(c); }

  // Convert a cavity type name into a type constant, or UNKNOWN if none.
  static Type StringToType(const char *name);

  // Convenience functions to test the type.
  bool TypeIsElectrodynamic() const { return type == EZ || type == EXY; }
  bool TypeIsWaveguideMode() const { return type == TE || type == TM; }

  // Return the excitation magnitude and phase for a particular port number.
  JetComplex PortExcitation(int port_number) const {
    if (port_number >= 1 && port_number <= port_excitation.size() / 2) {
      return std::polar(port_excitation[port_number * 2 - 2],
                        port_excitation[port_number * 2 - 1] * M_PI / 180.0);
    } else {
      return JetComplex(0.0);
    }
  }
};

// Compute a FEM solution from a mesh and a ScriptConfig. The ports and
// material types will be defined within the mesh. An instance of this class
// computes just one solution to one problem and then caches the results. To
// compute new solutions you will have to instantiate new objects.

class Solver : public Mesh {
 public:
  // The constructor creates the mesh for the shape and computes some auxiliary
  // data but does not yet compute the full solution. That's done on demand by
  // other functions. If 'lua' is provided the dielectric callback functions
  // can be called.
  explicit Solver(const Shape &s, const ScriptConfig &config, Lua *lua);
  ~Solver();

  // Did mesh creation succeed and is the shape and config valid?
  bool IsValid() const;

  // Return true if this solver is compatible with (i.e. will compute the same
  // solution as) another shape and config. This does not consider if any
  // derivatives are the same. This considers if dielectric callback functions
  // are the same and if the values returned by those functions are the same.
  bool SameAs(const Shape &s, const ScriptConfig &config, Lua *lua);

  // Update the derivatives from the new derivatives of points in 's'. This
  // allows e.g. new field and port derivatives to be computed. The shape 's'
  // must have exactly the same structure as the original 's' given to the
  // constructor, only differing in the derivatives. Return true on success or
  // false on failure.
  bool UpdateDerivatives(const Shape &s) MUST_USE_RESULT;

  // Draw the solution to OpenGL. This computes the solution on demand. All
  // drawing modes other than DRAW_AMPLITUDE are phase dependent and can be
  // animated by adjusting the phase_offset.
  enum DrawMode {
    DRAW_AMPLITUDE,                 // Color = |solution|
    DRAW_REAL,                      // Color =  Re{solution}
    DRAW_AMPLITUDE_REAL,            // Color = |Re{solution}|
    DRAW_GRADIENT_AMPLITUDE,        // Color = max |grad(solution)|
    DRAW_GRADIENT_AMPLITUDE_REAL,   // Color = |Re{grad(solution)}|
    DRAW_GRADIENT_VECTORS,          // Draw vectors from gradient of solution
    DRAW_ROTATED_GRADIENT_VECTORS,  // Draw gradient rotated by 90 degrees
    DRAW_POYNTING_VECTORS,          // Draw instantaneous power vectors
    DRAW_POYNTING_VECTORS_TA,       // Draw time averaged power vectors
  };
  void DrawSolution(DrawMode draw_mode, ColorMap::Function colormap,
                    int brightness, double phase_offset);

  // Compute the (complex) amplitudes of the outgoing waves at each port by
  // fitting to a TE10 field. Return true on success. This computes the
  // solution on demand. Note that port 1's field is returned in index 0, etc.
  bool ComputePortOutgoingField1(vector<JetComplex> *result) MUST_USE_RESULT;

  // Similar to ComputePortOutgoingField1() but the numerical technique is
  // different: we integrate the squared field across the port.
  bool ComputePortOutgoingField2(vector<JetComplex> *result) MUST_USE_RESULT;

  // Similar to ComputePortOutgoingField() but compute the outgoing power at
  // each port. Return true on success. The phase of each output corresponds to
  // ComputePortOutgoingField() but the magnitude equals the power.
  bool ComputePortOutgoingPower(vector<JetComplex> *result) MUST_USE_RESULT;

  // Retrieve the field values and other quantities from the solution at the
  // point (x,y) which is in config.unit. This interpolates across mesh
  // elements. It is assumed that the solution and derivative is valid (e.g.
  // because it has already been computed). Some function (e.g.
  // GetFieldPoynting) are not meaningful when solving for waveguide modes, but
  // will still compute a value.
  void GetField(JetNum x, JetNum y, JetComplex *value);
  void GetFieldGradient(JetNum x, JetNum y, JetComplex *dx, JetComplex *dy);
  void GetFieldPoynting(JetNum x, JetNum y, JetPoint *poynting);

  // Compute the radiation pattern at the ABC. Return arrays of azimuth (in
  // radians) and associated field magnitude. The azimuth angles will be
  // monotonically increasing and in the range -pi..pi.
  bool ComputeAntennaPattern(vector<double> *azimuth,
                             vector<JetNum> *magnitude) MUST_USE_RESULT;

  // Look up the radiation pattern for a particular azimuth 'theta' (in
  // radians). This will interpolate the results computed by
  // ComputeAntennaPattern(). Return false on failure.
  bool LookupAntennaPattern(JetNum theta, JetNum *magnitude) MUST_USE_RESULT;

  // Save the mesh and solution (even if it is empty) to the given matlab file.
  // In matlab the solution can be visualized with:
  //   patch('Vertices',p,'Faces',t,'FaceVertexCData',real(u),...
  //         'FaceColor','interp','EdgeColor','none'); axis equal
  void SaveMeshAndSolutionToMatlab(const char *filename);

  // If we are solving for waveguide modes this selects the particular mode to
  // display. This will trigger a solve if one has not been done yet.
  void SelectWaveguideMode(int n);

  // If we are solving for waveguide modes, return the cutoff frequencies of
  // all modes. Return true on success or false on failure. Note that
  // JetComplex is returned even though the result is really 'double' for
  // compatibility with ComputePortOutgoingField().
  bool ComputeModeCutoffFrequencies(vector<JetComplex> *cutoff) MUST_USE_RESULT;

 public:
  // Only one of ed_solver_ or mode_solver_ will be used, depending on the
  // config_.type. If ed_solver_ is used then solver_solution_ will point to a
  // vector within it. If mode_solver_ is used then solver_solution_ will be a
  // separately allocated vector that contains a copy of the eigenvector.
  Shape shape_;
  ScriptConfig config_;
  vector<JetNum> port_lengths_;         // Lengths of all ports. Slot 0 unused
  typedef FEM::FEMSolver<HelmholtzFEMProblem> EDSolverType;
  typedef FEM::FEMSolver<WaveguideModeFEMProblem> ModeSolverType;
  FEM::FEMSolverBase *solver_;          // Pointer to one of the solvers
  EDSolverType *ed_solver_;             // Electrodynamics solver
  ModeSolverType *mode_solver_;         // Eigenmode solver
  Eigen::VectorXcd *solver_solution_;   // Solution vector to display
  vector<double> antenna_azimuth_;      // Cached antenna pattern
  vector<JetNum> antenna_magnitude_;    // Cached antenna pattern

  // **********
  // The following variables are computed on demand, and have value (or size) 0
  // if they are not yet computed. The functions that compute the variables are
  // listed after the variables. Those functions do nothing if the variable was
  // already computed.

  // Solve for the current configuration. Return true on success or false if
  // the solve failed (e.g. the system matrix can not be factored).
  bool Solve() MUST_USE_RESULT;

  // The derivative (with respect to a parameter) of the solver solution. This
  // depends on the solver triplets and rhs.
  Eigen::VectorXcd solution_derivative_;
  // Return false on failure.
  bool ComputeDerivatives() MUST_USE_RESULT;

  // Spatial gradient of solution. This depends on the solver solution.
  Eigen::MatrixXcd Pgradient_;          // rows=0 or points_.size(), cols=2
  // Return false on failure.
  bool ComputeSpatialGradient() MUST_USE_RESULT;

  // Maximum magnitude of the spatial gradient. This depends on Pgradient_.
  Eigen::VectorXd Mgradient_;
  // Return false on failure.
  bool ComputeSpatialGradientMaxAmplitude() MUST_USE_RESULT;

  // The last ComputePortOutgoingPower() result.
  vector<JetComplex> port_outgoing_power_;

  // **********

  // Compute k^2 for the system given the config_.
  double ComputeKSquared();

  // Combine the information in the solver solution and solution_derivative_ to
  // return a JetComplex for point i.
  JetComplex SolutionJet(int i) const;

  friend struct HelmholtzFEMProblem;
  friend struct WaveguideModeFEMProblem;
};

#endif
