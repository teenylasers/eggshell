
#include "stdwx.h"
#include "cavity.h"
#include "mesh.h"
#include "mat_file.h"
#include "wxgl_font.h"
#include "plot_wx.h"
#include "si_prefix.h"
#include "mystring.h"
#include "shaders.h"

using std::vector;
using std::string;
using Eigen::Vector3f;

const int kNumAnimationSteps = 32;
const double kMaxReasonableTriangles = 1e12;

//***************************************************************************
// Cavity.

enum {
  ANIMATION_TIMER_ID = 10,
};

BEGIN_EVENT_TABLE(Cavity, LuaModelViewer)
  EVT_TIMER(ANIMATION_TIMER_ID, Cavity::OnAnimationTimeout)
END_EVENT_TABLE()

Cavity::Cavity(wxWindow* parent, wxWindowID id, const wxPoint &pos,
               const wxSize &size, long style)
    : LuaModelViewer(parent, id, pos, size, style,
                     gl::DoubleBuffer | gl::MultiSampleBuffer),
      animation_timer_(this, ANIMATION_TIMER_ID) {
  antenna_pattern_plot_ = 0;
  valid_ = false;
  solver_ = 0;
  solver_draw_mode_static_ = Solver::DRAW_REAL;
  solver_draw_mode_animating_ = Solver::DRAW_REAL;
  show_boundary_lines_and_ports_ = true;
  show_boundary_vertices_ = show_boundary_derivatives_ = false;
  show_grid_ = false;
  mesh_draw_type_ = Mesh::MESH_HIDE;
  show_field_ = false;
  antenna_show_ = antenna_scale_max_ = false;
  waveguide_mode_displayed_ = 0;
  animation_phase_ = -1;
}

Cavity::~Cavity() {
}

bool Cavity::IsModelEmpty() {
  return cd_.IsEmpty();
}

bool Cavity::IsModelValid() {
  return valid_;
}

void Cavity::SetModelValid(bool v) {
  valid_ = v;
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
    CHECK(solver_ && solver_->SameAs(cd_, config_, GetLua()));
    if (!solver_->UpdateDerivatives(cd_)) {
      GetLua()->Error("Could not update derivatives");
    }
  } else {
    // Delete the current solution if it no longer applies to the current
    // configuration, i.e. if the cd or config has changed then any mesh and
    // field solution derived from it are no longer valid.
    if (GetLua()->ThereWereErrors() ||
        (solver_ && !solver_->SameAs(cd_, config_, GetLua()))) {
      delete solver_;
      solver_ = 0;
    }
  }

  // Optionally update the antenna pattern.
  if (!GetLua()->ThereWereErrors() && antenna_show_) {
    PlotAntennaPattern();
  }
}

void Cavity::CreateArgumentsToOptimize(bool optimize_output_requested) {
  // Create the (port_power, port_phase, field) arguments to
  // config.optimize.
  vector<JetComplex> port_powers;

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
    if (CreateSolver() && solver_->ComputePortOutgoingPower(&port_powers)) {
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
  } else {
    // Push two dummy arguments for (port_power, port_phase).
    LuaRawGetGlobal(GetLua()->L(), "__ZeroTable__");
    LuaRawGetGlobal(GetLua()->L(), "__ZeroTable__");
  }
  // Push the 3rd argument to config.optimize.
  LuaRawGetGlobal(GetLua()->L(), "__Optimize3rdArg__");
}

void Cavity::DrawModel() {
  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glPointSize(1);

  gl::FlatShader().Use();
  ApplyCameraTransformations();

  // Non anti-aliased drawing (the grid).
  if (show_grid_) {
    DrawGrid();
  }

  if (AntiAliasing()) {
    glEnable(GL_MULTISAMPLE);
  }

  // Anti-aliased drawing.
  CreateStandardFonts(GetContentScaleFactor());
  int window_width, window_height;
  GetScaledClientSize(&window_width, &window_height);

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
  if (show_field_ && solver_) {
    solver_->SelectWaveguideMode(waveguide_mode_displayed_);
    solver_->DrawSolution(
        (animation_phase_ >= 0) ? solver_draw_mode_animating_ :
                                  solver_draw_mode_static_,
        GetColormap(), GetBrightness(),
        2.0 * M_PI * double(animation_phase_) / kNumAnimationSteps);

    // Draw the port powers, or cutoff frequencies, or other such information.
    if (config_.TypeIsElectrodynamic()) {
      vector<JetComplex> power;
      if (solver_->ComputePortOutgoingPower(&power)) {
        double scale = GetContentScaleFactor();
        for (int i = 0; i < power.size(); i++) {
          char s[100];
          snprintf(s, sizeof(s), "S%d = %.2f dB @ %.1f" DEGREE_SYMBOL,
                   i + 1, 10*log10(ToDouble(abs(power[i]))),
                   ToDouble(arg(power[i])) * 180.0 / M_PI);
          DrawString(s, 10*scale, window_height - (10 + 20*i)*scale,
                     &s_parameter_font, 0,0,0, TEXT_ALIGN_LEFT, TEXT_ALIGN_TOP);
        }
      }
    } else if (config_.TypeIsWaveguideMode()) {
      vector<JetComplex> cutoff;
      if (solver_->ComputeModeCutoffFrequencies(&cutoff)) {
        double scale = GetContentScaleFactor();
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
  if (mesh_draw_type_ != Mesh::MESH_HIDE && solver_) {
    solver_->DrawMesh(mesh_draw_type_, GetColormap(), GetBrightness(),
                      gl::Transform());
    if (show_boundary_derivatives_) {
      solver_->DrawPointDerivatives(kBoundaryDerivativesScale);
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
  SelectPane("Plots", true);
  GetMainPlot()->Refresh();
  return true;
}

bool Cavity::ComputeSweptOutput(std::vector<JetComplex> *output) {
  // If there's a problem solving or computing sweep outputs then abandon the
  // sweep.
  CreateSolver();
  if (!solver_) {
    Error("Sweep interrupted (solver failed)");
    return false;
  }
  if (config_.TypeIsElectrodynamic() &&
      !solver_->ComputePortOutgoingPower(output)) {
    Error("Sweep interrupted (failed to compute port powers");
    return false;
  }
  if (config_.TypeIsWaveguideMode() && !solver_->
      ComputeModeCutoffFrequencies(output)) {
    Error("Sweep interrupted (failed to compute cutoff frequencies");
    return false;
  }
  return true;
}

void Cavity::PrepareForOptimize() {
  // Delete solver in case we already have a solution for this shape but with
  // wrong derivative.
  delete solver_;
  solver_ = 0;
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
  Refresh();
}

void Cavity::ToggleShowBoundaryDerivatives() {
  show_boundary_derivatives_ = !show_boundary_derivatives_;
  Refresh();
}

void Cavity::ToggleShowBoundary() {
  show_boundary_lines_and_ports_ = !show_boundary_lines_and_ports_;
  Refresh();
}

void Cavity::ToggleGrid() {
  show_grid_ = !show_grid_;
  Refresh();
}

void Cavity::ViewMesh(int mesh_choicebox_selection) {
  mesh_draw_type_ = static_cast<Mesh::MeshDrawType>(mesh_choicebox_selection);
  Refresh();
}

void Cavity::ViewField(bool view_field) {
  show_field_ = view_field;
  Refresh();
}

void Cavity::Animate(bool animate) {
  if (animate) {
    animation_phase_ = 0;
    animation_timer_.Start(1000 / kNumAnimationSteps, wxTIMER_CONTINUOUS);
  } else {
    animation_phase_ = -1;
    animation_timer_.Stop();
    Refresh();
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
  Refresh();
}

void Cavity::ExportFieldMatlab(const char *filename) {
  if (solver_) {
    solver_->SaveMeshAndSolutionToMatlab(filename);
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
  Refresh();
}

void Cavity::Connect2(wxPlot *antenna_pattern_plot) {
  antenna_pattern_plot_ = antenna_pattern_plot;
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
  if (!solver_) {
    // Likely config.optimize is being called with dummy arguments without the
    // field having been solved for. Just return zeros.
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    return 2;
  }
  JetComplex value;
  solver_->GetField(luaL_checknumber(L, 1), luaL_checknumber(L, 2), &value);
  lua_pushnumber(L, value.real());
  lua_pushnumber(L, value.imag());
  return 2;
}

int Cavity::LuaPattern(lua_State *L) {
  if (lua_gettop(L) != 1) {
    LuaError(L, "Usage: _Pattern(theta)");
  }
  JetNum theta = luaL_checknumber(L, 1) * M_PI / 180.0;         // To radians
  if (!solver_) {
    // If there is no solver_ it's likely config.optimize is being called with
    // dummy arguments without the field having been solved for.
    lua_pushnumber(L, 0);
  } else {
    JetNum value;
    if (solver_->LookupAntennaPattern(theta, &value)) {
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
  if (!solver_) {
    // If there is no solver_ it's likely config.optimize is being called with
    // dummy arguments without the field having been solved for.
    lua_pushnumber(L, 0);
  } else {
    lua_pushnumber(L, ComputeAntennaDirectivity());
  }
  return 1;
}

int Cavity::LuaGetFieldPoynting(lua_State *L) {
  if (lua_gettop(L) != 2) {
    LuaError(L, "Usage: _GetFieldPoynting(x,y)");
  }
  if (!solver_) {
    // Likely config.optimize is being called with dummy arguments without the
    // field having been solved for. Just return zeros.
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    return 2;
  }
  JetPoint poynting;
  solver_->GetFieldPoynting(luaL_checknumber(L, 1), luaL_checknumber(L, 2),
                            &poynting);
  lua_pushnumber(L, poynting[0]);
  lua_pushnumber(L, poynting[1]);
  return 2;
}

Solver *Cavity::CreateSolver() {
  if (!solver_) {
    // Don't do anything for an empty cd as meshing will fail.
    if (cd_.IsEmpty()) {
      return 0;
    }
    solver_ = new Solver(cd_, config_, GetLua());
    // If solver creation fails then it will emit an Error() and we set
    // solver_=0.
    if (!solver_->IsValid()) {
      delete solver_;
      solver_ = 0;
    }
  }
  return solver_;
}

void Cavity::DrawGrid() {
  const double kGridBias = 2;   // Lower numbers fill in the grid more

  int window_width, window_height;
  GetScaledClientSize(&window_width, &window_height);

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
  GET_FIELD(frequency, config_.TypeIsElectrodynamic(), ToDouble, lua_tonumber,
            LUA_TNUMBER, 0, -1)
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

  // Handle excited_port specially because it can be a number or a table.
  config_.port_excitation.clear();
  lua_getfield(L, -1, "excited_port");
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
    lua_pop(L, 1);                      // Stack T
    for (int i = 1; i <= length; i++) {
      lua_geti(L, -1, i);               // Stack T T[i]
      config_.port_excitation.push_back(lua_tonumber(L, -1));
      lua_pop(L, 1);                    // Stack T
    }
  } else {
    GetLua()->Error("config.excited_port is not valid");
  }
  lua_pop(L, 1);
}

void Cavity::OnAnimationTimeout(wxTimerEvent& event) {
  animation_phase_ = (animation_phase_ + 1) % kNumAnimationSteps;
  Refresh();
}

JetNum Cavity::ComputeAntennaDirectivity() {
  vector<double> azimuth;
  vector<JetNum> magnitude;
  CreateSolver();
  if (solver_ && solver_->ComputeAntennaPattern(&azimuth, &magnitude)) {
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
  if (!antenna_show_ || !solver_ ||
      !solver_->ComputeAntennaPattern(&azimuth, &magnitude)) {
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
    JetNum directivity_dB = 10.0 * log10(ComputeAntennaDirectivity());
    string title;
    StringPrintf(&title, "Directivity = %.2f dB", ToDouble(directivity_dB));
    antenna_pattern_plot_->plot().SetTitle(title.c_str());
  }
  antenna_pattern_plot_->Refresh();
  // Draw now so we get updates e.g. during slider movement:
  antenna_pattern_plot_->Update();
}
