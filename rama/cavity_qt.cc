
#include "cavity_qt.h"
#include "mesh.h"
#include "../toolkit/mat_file.h"
#include "../toolkit/gl_font.h"
#include "../toolkit/plot_gui.h"
#include "../toolkit/si_prefix.h"
#include "../toolkit/mystring.h"
#include "../toolkit/shaders.h"

#include <QTimer>
#include <QDial>

using std::vector;
using std::string;
using Eigen::Vector3f;

const int kNumAnimationSteps = 32;          // Must be 2^n and match time_dial_
const double kMaxReasonableTriangles = 1e12;

//***************************************************************************
// Cavity.

Cavity::Cavity(QWidget *parent)
    : LuaModelViewer(parent, gl::DoubleBuffer | gl::MultiSampleBuffer) {  //@@@ gl bits ignored
  antenna_pattern_plot_ = sparam_plot_ = 0;
  time_dial_ = 0;
  solver_draw_mode_static_ = Solver::DRAW_REAL;
  solver_draw_mode_animating_ = Solver::DRAW_REAL;
  show_boundary_lines_and_ports_ = true;
  show_boundary_vertices_ = show_boundary_derivatives_ = false;
  show_grid_ = false;
  mesh_draw_type_ = Mesh::MESH_HIDE;
  show_field_ = show_wideband_pulse_ = false;
  antenna_show_ = antenna_scale_max_ = false;
  waveguide_mode_displayed_ = 0;
  animating_ = false;
  extended_time_dial_ = 0;
  displayed_soln_ = 0;
  optimizer_soln_ = 0;
  sparams_plot_type_ = 0;
  show_sparams_ = false;
}

Cavity::~Cavity() {
}

bool Cavity::IsModelEmpty() {
  return cd_.IsEmpty();
}

void Cavity::ResetModel() {
  debug_shapes_.clear();
  lua_State *L = GetLua()->L();
  Shape::SetLuaGlobals(L);
  LuaUserClassRegister<Shape>(*GetLua(), "Shape");

  GetLua()->SetUserObject(1, this);
  lua_pushcfunction(L, (LuaGlobalStub2<Cavity, &Cavity::LuaDraw, 1>));
  lua_setglobal(L, "Draw");
  lua_pushcfunction(L, (LuaGlobalStub2<Cavity, &Cavity::LuaGetField, 1>));
  lua_setglobal(L, "_GetField");
  lua_pushcfunction(L, (LuaGlobalStub2<Cavity, &Cavity::LuaPattern, 1>));
  lua_setglobal(L, "_Pattern");
  lua_pushcfunction(L, (LuaGlobalStub2<Cavity, &Cavity::LuaDirectivity, 1>));
  lua_setglobal(L, "_Directivity");
  lua_pushcfunction(L,
                    (LuaGlobalStub2<Cavity, &Cavity::LuaGetFieldPoynting, 1>));
  lua_setglobal(L, "_GetFieldPoynting");
  lua_pushcfunction(L, (LuaGlobalStub2<Cavity, &Cavity::LuaSelect, 1>));
  lua_setglobal(L, "_Select");
  lua_pushcfunction(L, (LuaGlobalStub2<Cavity, &Cavity::LuaSolveAll, 1>));
  lua_setglobal(L, "_SolveAll");
  lua_pushcfunction(L, (LuaGlobalStub2<Cavity, &Cavity::LuaPorts, 1>));
  lua_setglobal(L, "_Ports");
}

void Cavity::ScriptJustRan(bool only_compute_derivatives) {
  // Make sure the script left behind a config table containing, among
  // other things, a Shape object called "cd".
  LuaRawGetGlobal(GetLua()->L(), "config");
  if (lua_type(GetLua()->L(), -1) != LUA_TTABLE) {
    GetLua()->Error("The script should leave behind a 'config' table");
  } else {
    lua_getfield(GetLua()->L(), -1, "cd");
    Shape *new_cd = LuaCastTo<Shape>(GetLua()->L(), -1);
    if (new_cd) {
      cd_ = *new_cd;
    } else {
      GetLua()->Error("The script should assign 'config.cd' to a Shape object");
    }
    lua_pop(GetLua()->L(), 1);

    // Get other config_ parameters.
    SetConfigFromTable();
  }

  if (GetLua()->ThereWereErrors()) {
    return;
  }

  // Clean up the shape by removing edges that are too small to matter. The
  // assumption is that if the shape was meshed with triangles about the size
  // of the smallest edge and if that resulted in more than
  // kMaxReasonableTriangles then the simulation would be unreasonably slow.
  {
    JetNum length_max, length_min;
    cd_.ExtremeSideLengths(&length_max, &length_min);
    JetNum h = sqrt(cd_.TotalArea() / kMaxReasonableTriangles);
    cd_.Clean(ToDouble(h));
  }

  // Check for geometry errors.
  {
    const char *err = cd_.GeometryError();
    if (err) {
      GetLua()->Error(err);
      return;
    }
  }

  // If we're only computing updated derivatives then make sure that the solver
  // would compute the same solution (not counting derivatives).
  if (only_compute_derivatives) {
    CHECK(solver_.Valid() && solver_.SameAs(cd_, config_, GetLua()));
    if (!solver_.UpdateDerivatives(cd_)) {
      GetLua()->Error("Could not update derivatives");
    }
  } else {
    // Delete the current solution if it no longer applies to the current
    // configuration, i.e. if the cd or config has changed then any mesh and
    // field solution derived from it are no longer valid.
    if (GetLua()->ThereWereErrors() ||
        (solver_.Valid() && !solver_.SameAs(cd_, config_, GetLua()))) {
      solver_.Clear();
    }
  }

  // Optionally update the antenna pattern.
  if (!GetLua()->ThereWereErrors() && antenna_show_) {
    PlotAntennaPattern();
  }
}

void Cavity::CreatePortPowerAndPhase(int solution_index) {
  vector<JetComplex> port_powers;
  if (CreateSolver() && solver_.At(solution_index)->
      ComputePortOutgoingPower(&port_powers)) {
    // Create port_power and port_phase argument tables.
    lua_newtable(GetLua()->L());
    for (int i = 0; i < port_powers.size(); i++) {
      lua_pushnumber(GetLua()->L(), abs(port_powers[i]));
      lua_rawseti(GetLua()->L(), -2, i + 1);
    }
    lua_newtable(GetLua()->L());
    for (int i = 0; i < port_powers.size(); i++) {
      lua_pushnumber(GetLua()->L(), arg(port_powers[i]));
      lua_rawseti(GetLua()->L(), -2, i + 1);
    }
  } else {
    // If CreateSolver() fails it should emit an error, but we emit one
    // more here just to be safe.
    GetLua()->Error("Can not compute a solution");
  }
}

void Cavity::CreateArgumentsToOptimize(bool optimize_output_requested) {
  // If we know we are going to need the solver later (e.g. because we're
  // drawing fields or the mesh) then we will create it here, so that the
  // call to the lua optimize() function below can look up correct field
  // values through the 3rd argument. If we will not be needing the solver
  // later then we let the field lookups return 0 (e.g. when the field
  // display is turned off, which is a case that we want to render
  // quickly).
  if (WillCreateSolverInDraw()) {
    CreateSolver();
  }

  // We only compute port powers/phases if optimization output is being
  // requested, as ComputePortOutgoingPower() triggers a complete solve
  // which is too expensive if all we are going to do later is draw the cd
  // or the mesh.
  if (optimize_output_requested) {
    CreatePortPowerAndPhase(displayed_soln_);
  } else {
    // Push two dummy arguments for (port_power, port_phase).
    LuaRawGetGlobal(GetLua()->L(), "__ZeroTable__");
    LuaRawGetGlobal(GetLua()->L(), "__ZeroTable__");
  }

  // Push the 3rd argument to config.optimize. If optimizer output is not
  // requested then push a dummy argument that allows execution of the function
  // but that does not actually do any work.
  if (optimize_output_requested) {
    LuaRawGetGlobal(GetLua()->L(), "__Optimize3rdArg__");
  } else {
    LuaRawGetGlobal(GetLua()->L(), "__DummyOptimize3rdArg__");
  }
}

void Cavity::DrawModel() {
  GL(Disable)(GL_BLEND);
  GL(Disable)(GL_CULL_FACE);
  GL(Disable)(GL_DEPTH_TEST);
  GL(CullFace)(GL_BACK);
  GL(FrontFace)(GL_CCW);
  GL(PolygonMode)(GL_FRONT_AND_BACK, GL_FILL);
  GL(PointSize)(1);

  gl::FlatShader().Use();
  ApplyCameraTransformations();

  // Non anti-aliased drawing (the grid).
  if (show_grid_) {
    DrawGrid();
  }

  if (AntiAliasing()) {
    GL(Enable)(GL_MULTISAMPLE);
  }

  // Anti-aliased drawing.
  CreateStandardFonts(devicePixelRatio());
  int window_width = width() * devicePixelRatio();
  int window_height = height() * devicePixelRatio();

  // Draw the polygon interiors.
  // @@@ If we're drawing a field the field will overwrite the interior so this
  //     is wasted work, except when we draw the field as vectors.
  cd_.DrawInterior();

  // Compute and show the mesh and solution if requested. If solver_ has been
  // deleted that was a trigger to compute a new mesh and solution if
  // necessary.
  if (IsModelValid() && WillCreateSolverInDraw()) {
    CreateSolver();
  }
  if (show_field_ && solver_.Valid()) {
    solver_.First()->SelectWaveguideMode(waveguide_mode_displayed_);
    Solver *solver = solver_.At(displayed_soln_);
    solver->DrawSolution(
        animating_ ? solver_draw_mode_animating_ : solver_draw_mode_static_,
        GetColormap(), GetBrightness(),
        2.0 * M_PI * double(extended_time_dial_) / kNumAnimationSteps,
        (show_wideband_pulse_ && solver_.Size() > 1) ? &solver_ : 0);

    // Draw the port powers, or cutoff frequencies, or other such information.
    if (config_.TypeIsElectrodynamic()) {
      vector<JetComplex> power;
      double scale = devicePixelRatio();
      if (solver->ComputePortOutgoingPower(&power)) {
        for (int i = 0; i < power.size(); i++) {
          char s[100];
          snprintf(s, sizeof(s), "S%d = %.2f dB @ %.1f" DEGREE_SYMBOL,
                   i + 1, 10*log10(ToDouble(abs(power[i]))),
                   ToDouble(arg(power[i])) * 180.0 / M_PI);
          DrawString(s, 10*scale, window_height - (10 + 20*i)*scale,
                     &s_parameter_font, 0,0,0, TEXT_ALIGN_LEFT, TEXT_ALIGN_TOP);
        }
      }
      if (config_.frequencies.size() > 1) {
        double mag, F = config_.frequencies[displayed_soln_];
        char prefix = SIPrefix(F, 0.99, &mag);
        char s[100];
        snprintf(s, sizeof(s), "%.2f %cHz", F / mag, prefix);
        DrawString(s, window_width - 10*scale, window_height - 10*scale,
                   &s_parameter_font, 0,0,0, TEXT_ALIGN_RIGHT, TEXT_ALIGN_TOP);
      }
    } else if (config_.TypeIsWaveguideMode()) {
      vector<JetComplex> cutoff;
      if (solver_.First()->ComputeModeCutoffFrequencies(&cutoff)) {
        double scale = devicePixelRatio();
        for (int i = 0; i < cutoff.size(); i++) {
          double mag, F = ToDouble(cutoff[i].real());
          char prefix = SIPrefix(F, 0.99, &mag);
          char s[100];
          snprintf(s, sizeof(s), "F%d = %.2f %cHz", i, F / mag, prefix);
          DrawString(s, 10*scale, window_height - (10 + 20*i)*scale,
                     &s_parameter_font, 0,0,0, TEXT_ALIGN_LEFT, TEXT_ALIGN_TOP);
        }
      }
    }
  }
  const double kBoundaryDerivativesScale = 10;
  if (mesh_draw_type_ != Mesh::MESH_HIDE && solver_.Valid()) {
    Solver *solver = solver_.At(displayed_soln_);
    solver->DrawMesh(mesh_draw_type_, GetColormap(), GetBrightness(),
                     gl::Transform());
    if (show_boundary_derivatives_) {
      solver->DrawPointDerivatives(kBoundaryDerivativesScale);
    }
  }

  // Draw the computational domain.
  gl::SetUniform("color", 0, 0, 0);
  cd_.DrawBoundary(gl::Transform(),
                   show_boundary_lines_and_ports_, show_boundary_vertices_,
                   show_boundary_derivatives_ * kBoundaryDerivativesScale);

  // Draw the debug shapes.
  for (int i = 0; i < debug_shapes_.size(); i++) {
    gl::SetUniform("color", 0.5, 0.5, 0.5);
    debug_shapes_[i].DrawBoundary(gl::Transform(), true,
                          show_boundary_vertices_, show_boundary_derivatives_);
  }

  // Recompute and plot the S parameters for multiple-frequency solutions, if
  // necessary.
  PlotSParams();
}

bool Cavity::PlotSweepResults(int plot_type,
                  const std::string &sweep_parameter_name,
                  const std::vector<double> &sweep_values,
                  const std::vector<std::vector<JetComplex> > &sweep_output) {
  // Plot types are:
  //   0: Magnitude (for power, in dB)
  //   1: Phase
  //   2: Group delay
  GetMainPlot()->plot().Clear();

  // Check the results vector for consistent sizes (e.g. make sure the user
  // didn't change the number of ports or waveguide modes somehow during the
  // sweep).
  const vector<vector<JetComplex> > &results = sweep_output;
  for (int i = 1; i < results.size(); i++) {
    if (results[i].size() != results[i - 1].size()) {
      return false;
    }
  }

  // If no results then nothing to do.
  if (results.empty()) {
    return false;
  }

  // Compute the largest (by magnitude) output value.
  double biggest = 0;
  for (int i = 0; i < results.size(); i++) {
    for (int j = 0; j < results[i].size(); j++) {
      biggest = std::max(biggest, abs(ToComplex(results[i][j])));
    }
  }
  double si_scale;
  char si_prefix = SIPrefix(biggest, 1, &si_scale);

  // Fill in the traces.
  bool sparams = config_.TypeIsElectrodynamic();
  for (int i = 0; i < results[0].size(); i++) {
    vector<double> x, y;
    x.reserve(results.size());
    y.reserve(results.size());
    double phase_offset = 0;
    for (int j = 0; j < results.size(); j++) {
      if (plot_type == 0) {             // Power (in dB)
        x.push_back(sweep_values[j]);
        double y_value = abs(ToComplex(results[j][i]));
        if (sparams) {
          y.push_back(10*log10(y_value));
        } else {
          y.push_back(y_value / si_scale);
        }
      } else if (plot_type == 1) {      // Phase
        x.push_back(sweep_values[j]);
        y.push_back(arg(ToComplex(results[j][i])) * 180.0/M_PI + phase_offset);
        if (j > 0) {
          // Unwrap phase to prevent discontinuities.
          if (y[j] > y[j - 1] + 180) {
            y[j] -= 360;
            phase_offset -= 360;
          } else if (y[j] < y[j - 1] - 180) {
            y[j] += 360;
            phase_offset += 360;
          }
        }
      } else if (plot_type == 2) {      // Group delay
        if (j < results.size() - 1) {
          x.push_back((sweep_values[j] + sweep_values[j+1]) / 2.0);
          // Compute phase rotation over frequency from normalized complex nums.
          Complex p1 = ToComplex(results[j][i] / abs(results[j][i]));
          Complex p2 = ToComplex(results[j+1][i] / abs(results[j+1][i]));
          y.push_back(arg(p2 * conj(p1)) / (2.0 * M_PI *
                      (sweep_values[j+1] - sweep_values[j])));
        }
      }
    }
    GetMainPlot()->plot().AddTrace(x, y, -1, 2);
    if (sparams) {
      char s[100];
      snprintf(s, sizeof(s), "S_{%d}", i + 1);
      GetMainPlot()->plot().AddTraceLabel(s);
    }
  }
  GetMainPlot()->plot().Grid();
  GetMainPlot()->plot().SetXAxisLabel(sweep_parameter_name.c_str());
  if (plot_type == 0) {
    if (sparams) {
      GetMainPlot()->plot().SetYAxisLabel("Power (dB)");
      GetMainPlot()->plot().AxisYTight();
      GetMainPlot()->plot().SetYAxis(-50, 2);
    } else {
      char s[100];
      sprintf(s, "Frequency (%cHz)", si_prefix);
      GetMainPlot()->plot().SetYAxisLabel(s);
    }
  } else if (plot_type == 1) {
    GetMainPlot()->plot().SetYAxisLabel("Phase (degrees)");
  } else if (plot_type == 2) {
    GetMainPlot()->plot().SetYAxisLabel("Group delay (s)");
  }
  if (sparams) {
    GetMainPlot()->plot().SetTitle("Port output over parameter value");
  } else {
    GetMainPlot()->
      plot().SetTitle("Mode cutoff frequencies over parameter value");
  }
  GetMainPlot()->plot().AxisXTight();
  GetMainPlot()->plot().SetXAxis(sweep_values[0], sweep_values.back());
  SelectPane(plot_pane_, true);
  GetMainPlot()->update();
  return true;
}

void Cavity::PlotSParams(bool force_update) {
  //@@@ This has a lot of duplication of PlotSweepResults(), do we want to
  //    make a common helper function?

  // Plot types are:
  //   0: Magnitude (for power, in dB)
  //   1: Phase
  //   2: Group delay

  if (!show_sparams_ || !config_.TypeIsElectrodynamic() || !solver_.Valid()) {
    return;
  }

  // Create a vector of S parameter results. If the results are the same as
  // what is currently being displayed then skip redrawing.
  vector<vector<JetComplex>> results(solver_.Size());
  for (int i = 0; i < solver_.Size(); i++) {
    CHECK(solver_.At(i)->ComputePortOutgoingPower(&results[i]));
  }
  if (!force_update && results == sparam_results_) {
    return;
  }
  sparam_results_ = results;
  sparam_plot_->plot().Clear();

  // If there is only one result at one frequency then plot a short line
  // segment so it is visible.
  vector<double> frequencies = config_.frequencies;
  if (results.size() == 1) {
    results.push_back(results[0]);
    frequencies.push_back(frequencies[0] + 1);
  }

  // Fill in the traces.
  for (int i = 0; i < results[0].size(); i++) {
    vector<double> x, y;
    x.reserve(results.size());
    y.reserve(results.size());
    double phase_offset = 0;
    for (int j = 0; j < results.size(); j++) {
      if (sparams_plot_type_ == 0) {             // Power (in dB)
        x.push_back(frequencies.at(j));
        double y_value = abs(ToComplex(results[j][i]));
        y.push_back(10*log10(y_value));
      } else if (sparams_plot_type_ == 1) {      // Phase
        x.push_back(frequencies.at(j));
        y.push_back(arg(ToComplex(results[j][i])) * 180.0/M_PI + phase_offset);
        if (j > 0) {
          // Unwrap phase to prevent discontinuities.
          if (y[j] > y[j - 1] + 180) {
            y[j] -= 360;
            phase_offset -= 360;
          } else if (y[j] < y[j - 1] - 180) {
            y[j] += 360;
            phase_offset += 360;
          }
        }
      } else if (sparams_plot_type_ == 2) {      // Group delay
        if (j < results.size() - 1) {
          x.push_back((frequencies.at(j) + frequencies.at(j+1)) / 2.0);
          // Compute phase rotation over frequency from normalized complex nums.
          Complex p1 = ToComplex(results[j][i] / abs(results[j][i]));
          Complex p2 = ToComplex(results[j+1][i] / abs(results[j+1][i]));
          y.push_back(arg(p2 * conj(p1)) / (2.0 * M_PI *
                  (frequencies.at(j+1) - frequencies.at(j))));
        }
      }
    }
    sparam_plot_->plot().AddTrace(x, y, -1, 2);
    char s[100];
    snprintf(s, sizeof(s), "S_{%d}", i + 1);
    sparam_plot_->plot().AddTraceLabel(s);
  }
  sparam_plot_->plot().Grid();
  sparam_plot_->plot().SetXAxisLabel("Frequency");
  if (sparams_plot_type_ == 0) {
    sparam_plot_->plot().SetYAxisLabel("Power (dB)");
    sparam_plot_->plot().AxisYTight();
    sparam_plot_->plot().SetYAxis(-50, 2);
  } else if (sparams_plot_type_ == 1) {
    sparam_plot_->plot().SetYAxisLabel("Phase (degrees)");
  } else if (sparams_plot_type_ == 2) {
    sparam_plot_->plot().SetYAxisLabel("Group delay (s)");
  }
  sparam_plot_->plot().SetTitle("Port output over frequency");
  sparam_plot_->plot().AxisXTight();
  sparam_plot_->plot().SetXAxis(frequencies.at(0), frequencies.back());
  sparam_plot_->update();
}

void Cavity::SelectSParamPlot(int sparams_plot_type) {
  sparams_plot_type_ = sparams_plot_type;
  PlotSParams(true);
}

bool Cavity::ComputeSweptOutput(std::vector<JetComplex> *output) {
  // If there's a problem solving or computing sweep outputs then abandon the
  // sweep.
  CreateSolver();
  if (!solver_.Valid()) {
    Error("Sweep interrupted (solver failed)");
    return false;
  }
  if (config_.TypeIsElectrodynamic() &&
      !solver_.At(displayed_soln_)->ComputePortOutgoingPower(output)) {
    Error("Sweep interrupted (failed to compute port powers");
    return false;
  }
  if (config_.TypeIsWaveguideMode() && !solver_.At(displayed_soln_)->
      ComputeModeCutoffFrequencies(output)) {
    Error("Sweep interrupted (failed to compute cutoff frequencies");
    return false;
  }
  return true;
}

void Cavity::PrepareForOptimize() {
  // Clear solver in case we already have a solution for this shape but with
  // wrong derivative.
  solver_.Clear();
}

void Cavity::GetBoundingBox(double bounds[6]) {
  if (IsModelEmpty()) {
    bounds[0] = -1;
    bounds[1] = 1;
    bounds[2] = -1;
    bounds[3] = 1;
  } else {
    JetNum min_x, min_y, max_x, max_y;
    cd_.GetBounds(&min_x, &min_y, &max_x, &max_y);
    bounds[0] = ToDouble(min_x);
    bounds[1] = ToDouble(max_x);
    bounds[2] = ToDouble(min_y);
    bounds[3] = ToDouble(max_y);
  }
  bounds[4] = -1;
  bounds[5] = 1;
}

void Cavity::ToggleShowBoundaryVertices() {
  show_boundary_vertices_ = !show_boundary_vertices_;
  update();
}

void Cavity::ToggleShowBoundaryDerivatives() {
  show_boundary_derivatives_ = !show_boundary_derivatives_;
  update();
}

void Cavity::ToggleShowBoundary() {
  show_boundary_lines_and_ports_ = !show_boundary_lines_and_ports_;
  update();
}

void Cavity::ToggleGrid() {
  show_grid_ = !show_grid_;
  update();
}

void Cavity::ToggleWidebandPulse() {
  show_wideband_pulse_ = !show_wideband_pulse_;
  update();
}

void Cavity::ToggleShowSParams() {
  show_sparams_ = !show_sparams_;
  PlotSParams(true);
}

void Cavity::SetFrequencyIndex(int n) {
  // If there is no config table (because of a script error) or we are
  // simulating waveguide modes then config_.frequencies may not contain
  // anything, so just clamp the value.
  int i = std::max(0, std::min(n, int(config_.frequencies.size()-1)));
  if (displayed_soln_ != i) {
    displayed_soln_ = i;
    update();
    if (antenna_show_) {
      PlotAntennaPattern();
    }
  }
}

int Cavity::GetNumFrequencies() const {
  return config_.frequencies.size();
}

void Cavity::TimeDialChanged(int value) {
  static_assert(kNumAnimationSteps == 32, "kNumAnimationSteps not 32");
  int32_t delta = value - (extended_time_dial_ & 31);
  delta = (delta << (32-5)) >> (32-5);
  extended_time_dial_ += delta;
  if (delta != 0) {
    update();
  }
}

void Cavity::TimeDialToZero() {
  extended_time_dial_ = 0;
  update();
}

void Cavity::ViewMesh(int mesh_choicebox_selection) {
  mesh_draw_type_ = static_cast<Mesh::MeshDrawType>(mesh_choicebox_selection);
  update();
}

void Cavity::ViewField(bool view_field) {
  show_field_ = view_field;
  update();
}

void Cavity::Animate(bool animate) {
  animating_ = animate;
  if (animate) {
    QTimer::singleShot(1000 / kNumAnimationSteps, this,
                       &Cavity::OnAnimationTimeout);
  } else {
    update();
  }
}

void Cavity::SetDisplayStyle(int style) {
  // For Exy cavities:
  switch (style) {
    case 0:                     // 0: Hz amplitude and phase
      solver_draw_mode_static_ = Solver::DRAW_REAL;
      solver_draw_mode_animating_ = Solver::DRAW_REAL;
      break;
    case 1:                     // 1: Hz amplitude only
      solver_draw_mode_static_ = Solver::DRAW_AMPLITUDE;
      solver_draw_mode_animating_ = Solver::DRAW_AMPLITUDE_REAL;
      break;
    case 2:                     // 2: Magnitude [Ex,Ey]
      solver_draw_mode_static_ = Solver::DRAW_GRADIENT_AMPLITUDE;
      solver_draw_mode_animating_ = Solver::DRAW_GRADIENT_AMPLITUDE_REAL;
      break;
    case 3:                     // 3: Rotated E
      solver_draw_mode_static_ = Solver::DRAW_GRADIENT_VECTORS;
      solver_draw_mode_animating_ = Solver::DRAW_GRADIENT_VECTORS;
      break;
    case 4:                     // 4: Vector E, Jsurf
      solver_draw_mode_static_ = Solver::DRAW_ROTATED_GRADIENT_VECTORS;
      solver_draw_mode_animating_ = Solver::DRAW_ROTATED_GRADIENT_VECTORS;
      break;
    case 5:                     // 5: Poynting vector
      solver_draw_mode_static_ = Solver::DRAW_POYNTING_VECTORS_TA;
      solver_draw_mode_animating_ = Solver::DRAW_POYNTING_VECTORS;
      break;
    default:
      Panic("Internal: bad style = %d", style);
  }
  update();
}

void Cavity::ExportFieldMatlab(const char *filename) {
  if (solver_.Valid()) {
    solver_.At(displayed_soln_)->SaveMeshAndSolutionToMatlab(filename);
  } else {
    Error("There is no current solution to save");
  }
}

void Cavity::ExportBoundaryDXF(const char *filename) {
  cd_.SaveBoundaryAsDXF(filename);
}

void Cavity::ExportBoundaryXY(const char *filename) {
  cd_.SaveBoundaryAsXY(filename);
}

void Cavity::ToggleAntennaShow() {
  antenna_show_ = !antenna_show_;
  PlotAntennaPattern();
}

void Cavity::ToggleAntennaScaleMax() {
  antenna_scale_max_ = !antenna_scale_max_;
  PlotAntennaPattern();
}

void Cavity::SetWaveguideModeDisplayed(int n) {
  waveguide_mode_displayed_ = n;
  update();
}

void Cavity::Connect2(qtPlot *antenna_pattern_plot, qtPlot *sparam_plot,
                      QDial *time_dial) {
  antenna_pattern_plot_ = antenna_pattern_plot;
  sparam_plot_ = sparam_plot;
  time_dial_ = time_dial;
}

int Cavity::LuaDraw(lua_State *L) {
  Shape *s = 0;
  if (lua_gettop(L) != 1 || (s = LuaCastTo<Shape>(L, 1)) == 0) {
    LuaError(L, "Draw() expects one Shape argument");
  }
  debug_shapes_.push_back(*s);  // Does a deep copy of the shape
  return 0;
}

int Cavity::LuaGetField(lua_State *L) {
  if (lua_gettop(L) != 2) {
    LuaError(L, "Usage: _GetField(x,y)");
  }
  if (!solver_.Valid()) {
    // Likely config.optimize is being called with dummy arguments without the
    // field having been solved for. Just return zeros.
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    return 2;
  }
  JetComplex value;
  solver_.At(optimizer_soln_)->
        GetField(luaL_checknumber(L, 1), luaL_checknumber(L, 2), &value);
  lua_pushnumber(L, value.real());
  lua_pushnumber(L, value.imag());
  return 2;
}

int Cavity::LuaPattern(lua_State *L) {
  if (lua_gettop(L) != 1) {
    LuaError(L, "Usage: _Pattern(theta)");
  }
  JetNum theta = luaL_checknumber(L, 1) * M_PI / 180.0;         // To radians
  if (!solver_.Valid()) {
    // If there is no solver_ it's likely config.optimize is being called with
    // dummy arguments without the field having been solved for.
    lua_pushnumber(L, 0);
  } else {
    JetNum value;
    if (solver_.At(optimizer_soln_)->LookupAntennaPattern(theta, &value)) {
      lua_pushnumber(L, sqr(abs(value)));
    } else {
      lua_pushnumber(L, 0);
    }
  }
  return 1;
}

int Cavity::LuaDirectivity(lua_State *L) {
  if (lua_gettop(L) != 0) {
    LuaError(L, "Usage: _Directivity()");
  }
  if (!solver_.Valid()) {
    // If there is no solver_ it's likely config.optimize is being called with
    // dummy arguments without the field having been solved for.
    lua_pushnumber(L, 0);
  } else {
    lua_pushnumber(L, ComputeAntennaDirectivity(optimizer_soln_));
  }
  return 1;
}

int Cavity::LuaGetFieldPoynting(lua_State *L) {
  if (lua_gettop(L) != 2) {
    LuaError(L, "Usage: _GetFieldPoynting(x,y)");
  }
  if (!solver_.Valid()) {
    // Likely config.optimize is being called with dummy arguments without the
    // field having been solved for. Just return zeros.
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    return 2;
  }
  JetPoint poynting;
  solver_.At(optimizer_soln_)->
    GetFieldPoynting(luaL_checknumber(L, 1), luaL_checknumber(L, 2), &poynting);
  lua_pushnumber(L, poynting[0]);
  lua_pushnumber(L, poynting[1]);
  return 2;
}

int Cavity::LuaSelect(lua_State *L) {
  if (lua_gettop(L) != 1) {
    LuaError(L, "Usage: _Select(n)");
  }
  int n = ToDouble(lua_tonumber(L, 1));
  if (n < 1 || n > config_.frequencies.size()) {
    LuaError(L, "Usage: _Select(n), invalid n");
  }
  optimizer_soln_ = n - 1;
  return 0;
}

int Cavity::LuaSolveAll(lua_State *L) {
  if (lua_gettop(L) != 0) {
    LuaError(L, "Usage: _SolveAll()");
  }
  if (solver_.Solve()) {
    // Ignore result.
  }
  return 0;
}


int Cavity::LuaPorts(lua_State *) {
  CreatePortPowerAndPhase(optimizer_soln_);
  return 2;
}

bool Cavity::CreateSolver() {
  if (!solver_.Valid()) {
    // Don't do anything for an empty cd as meshing will fail.
    if (cd_.IsEmpty()) {
      return false;
    }
    solver_.PushBack(new Solver(cd_, config_, GetLua(), 0));
    if (config_.TypeIsElectrodynamic()) {
      for (int i = 1; i < config_.frequencies.size(); i++) {
        solver_.PushBack(new Solver(solver_.First(), i));
      }
    }
    // If solver creation fails then it will emit an Error() and clear solver_.
    if (!solver_.IsValid()) {
      solver_.Clear();
    }
  }
  return solver_.Valid();
}

void Cavity::DrawGrid() {
  const double kGridBias = 2;   // Lower numbers fill in the grid more

  int window_width = width() * devicePixelRatio();
  int window_height = height() * devicePixelRatio();

  // Compute model coordinates extents of the viewport.
  Eigen::Vector3d bottom_left, top_right;
  gl::PixelToModelCoordinates(Eigen::Vector3d(0, 0, 0),
                              gl::Transform(), &bottom_left);
  gl::PixelToModelCoordinates(Eigen::Vector3d(window_width, window_height, 0),
                              gl::Transform(), &top_right);

  // Draw two layers of gridlines with different colors and scales.
  double lscale = log10((top_right[0] - bottom_left[0]) / window_width) +
                  kGridBias;
  double iscale[2] = {floor(lscale), ceil(lscale)};     // Two picked scales
  // Compute (greyscale) gridline colors for both scales.
  double col[2];
  for (int i = 0; i < 2; i++) {
    col[i] = 0.6 + 0.4*(fabs(lscale - iscale[i]));
  }

  // Draw lighter gridlines first then darker gridlines.
  int first = (col[0] > col[1]) ? 0 : 1;
  for (int g = 0; g < 2; g++) {
    int index = (first + g) & 1;
    double scale = pow(10, iscale[index]);
    int imin_x = floor(bottom_left[0] / scale);
    int imax_x = ceil(top_right[0] / scale);
    int imin_y = floor(bottom_left[1] / scale);
    int imax_y = ceil(top_right[1] / scale);

    gl::SetUniform("color", col[index], col[index], col[index]);
    vector<Vector3f> points;
    for (int i = imin_x; i <= imax_x; i++) {
      points.push_back(Vector3f(i * scale, bottom_left[1], 0));
      points.push_back(Vector3f(i * scale, top_right[1], 0));
    }
    for (int i = imin_y; i <= imax_y; i++) {
      points.push_back(Vector3f(bottom_left[0], i * scale, 0));
      points.push_back(Vector3f(top_right[0], i * scale, 0));
    }
    gl::Draw(points, GL_LINES);
  }

  // Draw x=0 and y=0 gridlines separately.
  gl::SetUniform("color", 0, 0, 1);
  vector<Vector3f> points(4);
  points[0] << 0, bottom_left[1], 0;
  points[1] << 0, top_right[1], 0;
  points[2] << bottom_left[0], 0, 0;
  points[3] << top_right[0], 0, 0;
  gl::Draw(points, GL_LINES);
}

void Cavity::SetConfigFromTable() {
  #define GET_FIELD(fieldname, required, fn1, fn2, ltype, minval, def) \
    lua_getfield(L, -1, #fieldname); \
    if (lua_type(L, -1) == LUA_TNIL) { \
      /* Field not available, set to the default value (usually -1). */ \
      config_.fieldname = static_cast<typeof(config_.fieldname)>(def); \
      if (required) { \
        GetLua()->Error("config." #fieldname " is required"); \
      } \
    } else { \
      if (lua_tonumber(L, -1).Derivative() != 0) { \
        GetLua()->Error("config." #fieldname " can not depend on " \
                        "parameters as this will confuse the optimizer"); \
      } \
      config_.fieldname = fn1(fn2(L, -1)); \
      if (lua_type(L, -1) != ltype || config_.fieldname < minval || \
          config_.fieldname != fn1(fn2(L, -1))) { \
        config_.fieldname = static_cast<typeof(config_.fieldname)>(-1); \
        GetLua()->Error("config." #fieldname " is not valid"); \
      } \
    } \
    lua_pop(L, 1);
  lua_State *L = GetLua()->L();
  GET_FIELD(type, true, ScriptConfig::StringToType, lua_tostring,
            LUA_TSTRING, 0, -1)
  GET_FIELD(unit, true, DistanceScale, lua_tostring, LUA_TSTRING, 0, -1)
  GET_FIELD(mesh_edge_length, true, ToDouble, lua_tonumber, LUA_TNUMBER, 0, -1)
  GET_FIELD(mesh_refines, true, ToDouble, lua_tonumber, LUA_TNUMBER, 0, -1)
  GET_FIELD(depth, config_.type == ScriptConfig::EXY, ToDouble, lua_tonumber,
            LUA_TNUMBER, 0, -1)
  GET_FIELD(boresight, false, ToDouble, lua_tonumber, LUA_TNUMBER, -1e99, 0)
  GET_FIELD(max_modes, config_.TypeIsWaveguideMode(), ToDouble, lua_tonumber,
            LUA_TNUMBER, 1, 1)
  #undef GET_FIELD
  if (config_.type == ScriptConfig::SCHRODINGER) {
    config_.type = ScriptConfig::EZ;
    config_.schrodinger = true;
  }

  // Handle wideband_window specially because it is an enumeration.
  config_.wideband_window = config_.RECTANGLE;  // Default
  lua_getfield(L, -1, "wideband_window");       // Stack: wideband_window
  if (lua_type(L, -1) != LUA_TNIL) {
    if (strcmp(lua_tostring(L, -1), "rectangle") == 0) {
      config_.wideband_window = config_.RECTANGLE;
    } else if (strcmp(lua_tostring(L, -1), "hamming") == 0) {
      config_.wideband_window = config_.HAMMING;
    } else {
      GetLua()->Error("config.wideband_window is not valid");
    }
  }
  lua_pop(L, 1);

  // Handle excited_port specially because it can be a number or a table.
  config_.port_excitation.clear();
  lua_getfield(L, -1, "excited_port");  // Stack: excited_port
  if (lua_type(L, -1) == LUA_TNIL) {
    if (config_.TypeIsElectrodynamic()) {
      GetLua()->Error("config.excited_port is required");
    }
  } else if (lua_type(L, -1) == LUA_TNUMBER) {
    double excited_port_d = ToDouble(lua_tonumber(L, -1));
    int excited_port = excited_port_d;
    if (excited_port < 1 || excited_port != excited_port_d) {
      GetLua()->Error("config.excited_port is not valid");
    }
    config_.port_excitation.resize(excited_port * 2);
    config_.port_excitation[excited_port * 2 - 2] = 1;
  } else if (lua_type(L, -1) == LUA_TTABLE) {
    lua_len(L, -1);                     // Stack: T len
    int length = lua_tointeger(L, -1);
    lua_pop(L, 1);                      // Stack: T
    for (int i = 1; i <= length; i++) {
      lua_geti(L, -1, i);               // Stack: T T[i]
      config_.port_excitation.push_back(lua_tonumber(L, -1));
      lua_pop(L, 1);                    // Stack: T
    }
  } else {
    GetLua()->Error("config.excited_port is not valid");
  }
  lua_pop(L, 1);

  // Handle config.frequency specially because it can be a number or a table.
  config_.frequencies.clear();
  lua_getfield(L, -1, "frequency");     // Stack: frequencies
  if (lua_type(L, -1) == LUA_TNIL) {
    if (config_.TypeIsElectrodynamic()) {
      GetLua()->Error("config.frequency is required");
    }
  } else {
    if (config_.TypeIsWaveguideMode()) {
      GetLua()->Error("config.frequency is not used for mode solutions");
    }
    if (lua_type(L, -1) == LUA_TNUMBER) {
      int ok = 0;
      config_.frequencies.push_back(ToDouble(lua_tonumberx(L, -1, &ok)));
      if (!ok || config_.frequencies.back() < 0) {
        GetLua()->Error("config.frequency has invalid frequency "
                        "(not a number or < 0)");
      }
    } else if (lua_type(L, -1) == LUA_TTABLE) {
      lua_len(L, -1);                     // Stack: T len
      int length = lua_tointeger(L, -1);
      if (length <= 0) {
        GetLua()->Error("config.frequency must contain at least one entry");
      }
      lua_pop(L, 1);                      // Stack: T
      for (int i = 1; i <= length; i++) {
        lua_geti(L, -1, i);               // Stack: T T[i]
        int ok = 0;
        config_.frequencies.push_back(ToDouble(lua_tonumberx(L, -1, &ok)));
        lua_pop(L, 1);                    // Stack: T
        if (!ok || config_.frequencies.back() < 0) {
          GetLua()->Error("config.frequency has invalid frequency "
                          "(not a number or < 0)");
        }
      }
    } else {
      GetLua()->Error("config.frequency is not valid");
    }
  }
  lua_pop(L, 1);

  // Make sure solution indexes are in the correct range.
  displayed_soln_ = std::max(0,
      std::min(displayed_soln_, int(config_.frequencies.size()-1)));
  optimizer_soln_ = std::max(0,
      std::min(optimizer_soln_, int(config_.frequencies.size()-1)));
}

void Cavity::OnAnimationTimeout() {
  if (animating_) {
    extended_time_dial_++;
    time_dial_->setValue(extended_time_dial_ & (kNumAnimationSteps - 1));
    update();
    QTimer::singleShot(1000 / kNumAnimationSteps, this,
                       &Cavity::OnAnimationTimeout);
  }
}

JetNum Cavity::ComputeAntennaDirectivity(int solution_index) {
  vector<double> azimuth;
  vector<JetNum> magnitude;
  CreateSolver();
  if (solver_.Valid() && solver_.At(solution_index)->
                            ComputeAntennaPattern(&azimuth, &magnitude)) {
    CHECK(azimuth.size() == magnitude.size());
    JetNum power_avg = 0, power_max = 0;
    for (int i = 0; i < magnitude.size(); i++) {
      int inext = (i + 1) % magnitude.size();
      int iprev = (i + magnitude.size() - 1) % magnitude.size();
      double delta_angle = azimuth[inext] - azimuth[iprev];
      if (delta_angle < 0) {
        delta_angle += 2 * M_PI;
      }
      delta_angle /= 2.0;
      power_avg += sqr(magnitude[i]) * delta_angle / (2 * M_PI);
      power_max = std::max(power_max, sqr(magnitude[i]));
    }
    return power_max / power_avg;
  }
  return 0;
}

void Cavity::PlotAntennaPattern() {
  vector<double> azimuth;
  vector<JetNum> magnitude;
  vector<Plot::Axis> axis_stack;
  axis_stack = antenna_pattern_plot_->plot().GetAxisStack();
  antenna_pattern_plot_->plot().Clear();
  CreateSolver();
  if (!antenna_show_ || !solver_.Valid() ||
      !solver_.At(displayed_soln_)->ComputeAntennaPattern(&azimuth, &magnitude)) {
    // The antenna pattern can't be (or should not be) computed, e.g. because
    // there is no valid solution, no ABC or the effective k doesn't support
    // propagating waves.
  } else {
    vector<double> x(azimuth.size()), y(azimuth.size());
    double maxy = -1e99;
    for (int i = 0; i < azimuth.size(); i++) {
      x[i] = azimuth[i] * 180.0 / M_PI;
      y[i] = 20 * log10(ToDouble(magnitude[i]));
      maxy = std::max(maxy, y[i]);
    }
    if (antenna_scale_max_) {
      for (int i = 0; i < azimuth.size(); i++) {
        y[i] -= maxy;
      }
    }
    antenna_pattern_plot_->plot().AddTrace(x, y, 0xc08000, 2);
    antenna_pattern_plot_->plot().SetXAxis(-180, 180);
    antenna_pattern_plot_->plot().AxisXTight();
    if (antenna_scale_max_) {
      antenna_pattern_plot_->plot().SetYAxis(-50, 0);
      antenna_pattern_plot_->plot().AxisYTight();
    }
    antenna_pattern_plot_->plot().SetXAxisLabel("Angle (degrees)");
    antenna_pattern_plot_->plot().SetYAxisLabel("dB");
    antenna_pattern_plot_->plot().Grid();
    if (axis_stack.size() > 1) {
      antenna_pattern_plot_->plot().SetAxisStack(axis_stack);
    }
    JetNum directivity_dB = 10.0 *
                            log10(ComputeAntennaDirectivity(displayed_soln_));
    string title;
    StringPrintf(&title, "Directivity = %.2f dB", ToDouble(directivity_dB));
    antenna_pattern_plot_->plot().SetTitle(title.c_str());
  }
  // Draw now so we get updates e.g. during slider movement:
  antenna_pattern_plot_->repaint();
}
