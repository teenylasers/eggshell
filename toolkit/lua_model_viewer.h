
// A shell for applications that have Lua script driven creation of models,
// that are controllable by parameters and can be swept and optimized.

#ifndef __LUA_MODEL_VIEWER_H__
#define __LUA_MODEL_VIEWER_H__

#include <string>
#include <map>

#include "viewer.h"
#include "gl_utils.h"
#include "colormaps.h"
#include "optimizer.h"
#include "text_alignment.h"
#include "my_jet.h"
#include "lua_util.h"

class MyLua;
class MyCheckBox;
class MySlider;
class MyTextCtrl;
class wxPlot;

struct Parameter {
  std::string label;            // Name of this parameter.
  double the_min, the_max;      // Valid range of parameter
  double the_default;           // Default value of parameter
  double value;                 // Current value of parameter
  bool integer;                 // Takes on integer values only?
  MyCheckBox *checkbox;         // Associated checkbox control
  MySlider *slider;             // Associated slider control
  MyTextCtrl *text_ctrl;        // Associated text control

  Parameter() {
    the_min = the_max = the_default = value = 0;
    integer = false;
    checkbox = 0;
    slider = 0;
    text_ctrl = 0;
  }
};

struct DebugText {
  DebugText(double x_, double y_, std::string text_,
            TextAlignment halign_, TextAlignment valign_)
    : x(x_), y(y_), text(text_), halign(halign_), valign(valign_) {}
  double x, y;
  std::string text;
  TextAlignment halign, valign;
};

class LuaModelViewer : public GLViewer {
 public:
  LuaModelViewer(wxWindow* parent, int gl_type);
  ~LuaModelViewer();

  // wx event handling.
  void OnIdle(wxIdleEvent &event);
  void OnScriptMessage(wxCommandEvent &event);

  // Handling for menu and control commands.
  void ZoomToExtents();
  void ZoomIn(double scale_factor = 0);
  void ZoomOut();
  void ToggleAntialiasing();
  void ToggleShowMarkers();
  void SetDisplayColorScheme(int color_scheme);
  void SetBrightness(int brightness);
  void SetIfRunScriptResetsParameters(bool runscript_resets_param_map) {
    runscript_resets_param_map_ = runscript_resets_param_map;
  }
  void Sweep(const std::string &parameter_name,
             double start_value, double end_value, int num_steps);
  void Optimize();
  void StopSweepOrOptimize();
  void ToggleEmitTraceReport();
  void SetOptimizer(OptimizerType type) { ih_.optimizer_type = type; }
  void CopyParametersToClipboard();

  // Connect to external controls.
  void Connect(wxListCtrl *script_messages, wxAuiNotebook *aui_notebook,
               wxFlexGridSizer *parameter_flexgrid,
               wxScrolledWindow* parameter_scrolledwindow, wxPlot *plot);

  // Execute a new script, but only if it is different from the last script
  // than we ran or if rerun_even_if_same is true. Rebuild all parameter
  // controls along the way. Return true if the script actually ran, or false
  // if the script did not change. See RerunScript() for the treatment of
  // optimize_output.
  bool RunScript(const char *script, bool rerun_even_if_same,
                 std::vector<JetNum> *optimize_output = 0);

  // Rerun the last script given to RunScript(). This does nothing if
  // RunScript() has not yet been called. Parameter controls are not rebuilt by
  // default, so this is faster than RunScript() in the case where only
  // parameter values have changed. If optimize_output is nonzero return the
  // output of the config.optimize function, emitting an error and setting
  // optimize_output to the empty vector if no optimizer output could be
  // obtained (e.g. because of script error). The return value is false if
  // there were script errors (this value is also stored in valid_). If
  // only_compute_derivatives is true then the script is being redundantly
  // rerun with the same parameters but different derivatives.
  bool RerunScript(bool refresh_window = true,
                   std::vector<JetNum> *optimize_output = 0,
                   bool only_compute_derivatives = false);

  // Get information about a parameter. It's a fatal error if the parameter
  // name doesn't exist.
  const Parameter &GetParameter(const std::string &parameter_name);

  // Return all current parameter control names, in the order they are
  // displayed.
  void GetParamControlNames(std::vector<std::string> *names) const;

  // Implement lua functions:
  int LuaCreateParameter(lua_State *L);   // _CreateParameter()
  int LuaCreateMarker(lua_State *L);      // _CreateMarker()
  int LuaParameterDivider(lua_State *L);  // ParameterDivider()
  int LuaDrawText(lua_State *L);          // DrawText()

  // Accessors.
  ColorMap::Function GetColormap() const { return colormap_; }
  int GetBrightness() const { return brightness_; }
  wxPlot *GetMainPlot() { return plot_; }
  Lua *GetLua();
  bool AntiAliasing() const { return antialiasing_; }

  // Add a line to the "script messages" pane. The arguments map directly to
  // wxListCtrl::InsertItem. This is callable from multiple threads so that
  // is can be called as part of an Error() callback.
  void AddScriptMessage(int index, const std::string &msg, int icon_number);

  // Return the client size scaled by GetContentScaleFactor(). This is only
  // different from GetClientSize() on OS X, where it returns the window size
  // in raw pixels not points (e.g. on retina displays each point can be 2x2
  // pixels).
  void GetScaledClientSize(int *window_width, int *window_height);

  // Select the given pane of the notebook. If sticky is true then ignore
  // further SelectPane calls for 100ms.
  void SelectPane(const char *pane_name, bool sticky = false);

  // Select which kind of plot we want to display (the plot types are subclass
  // dependent).
  void SelectPlot(int plot_kind);

  // Convert a units name ('meter', 'mil', etc) to a distance scale in meters
  // or return -1 if the unit name is not known.
  double DistanceScale(const char *unit_name);

  // Export plot data to a matlab file.
  void ExportPlotMatlab(const char *filename);

  // ********** Override these to implement the model functionality.

  // Is there something to display? This is false if DrawModel() will draw
  // something.
  virtual bool IsModelEmpty()=0;

  // Is the model good enough to simulate?
  virtual bool IsModelValid()=0;

  // Set the model-valid flag
  virtual void SetModelValid(bool v)=0;

  // We are running a new script. Reset everything in preparation for a new
  // model being built. A fresh lua state has been constructed so populate it
  // with the required functions etc.
  virtual void ResetModel()=0;

  // We have just finished running a script without errors. Handle whatever
  // post-script computations need to be done. If only_compute_derivatives is
  // true then the script is being redundantly rerun with the same parameters
  // but different derivatives.
  virtual void ScriptJustRan(bool only_compute_derivatives)=0;

  // Push the Lua arguments for the optimize() function. If
  // optimize_output_requested is true then this is a real call which should
  // generate actual results, otherwise this is a dummy call to make sure the
  // function runs.
  virtual void CreateArgumentsToOptimize(bool optimize_output_requested)=0;

  // Draw the model.
  virtual void DrawModel()=0;

  // Plot the sweep results and show the plot pane. Return false if there was
  // an error in the data.
  virtual bool PlotSweepResults(int plot_type,
      const std::string &sweep_parameter_name,
      const std::vector<double> &sweep_values,
      const std::vector<std::vector<JetComplex> > &sweep_output)=0;

  // This is called for every sweep parameter value after the script has been
  // successfully run. It puts the results into output, which will be later
  // plotted. Return false if there is some error.
  virtual bool ComputeSweptOutput(std::vector<JetComplex> *output)=0;

  // For each iteration of the optimization this is called before RerunScript()
  // to do any extra preparation that is necessary.
  virtual void PrepareForOptimize()=0;

 private:
  int dragging_marker_;                 // >= 0 if now dragging a marker
  std::string script_;                  // The last script we ran
  MyLua *lua_;                          // Lua context for last script, 0=none
  int num_optimize_outputs_;            // Num values returned by optimize()
  bool in_rerun_script_;                // If RerunScript() is running
  bool emit_trace_report_;              // If printing trace report when idle
  int plot_type_;                       // Given to SelectPlot()

  // Connections to external controls.
  wxListCtrl *script_messages_;
  wxAuiNotebook *aui_notebook_;
  wxFlexGridSizer *parameter_flexgrid_;
  wxScrolledWindow *parameter_scrolledwindow_;
  wxPlot *plot_;

  // The current parameters, and state used during script execution. The
  // param_map_ maps param labels to values (and other parameter state) so that
  // the values can persist across runs and reloads of the script.
  typedef std::map<std::string, Parameter> ParamMap;
  ParamMap param_map_;
  bool runscript_resets_param_map_;  // If param_map_ reset by RunScript()
  std::vector<std::string> current_param_controls_;  // Current control names
  std::vector<std::string> markers_; // Pairs of controls names shown as markers
  int num_ticked_count_;             // Ticked params seen by CreateParameter()
  int derivative_index_;             // n'th checked parameter gets derivative
  // If true then rebuild parameter controls in RerunScript(). If script
  // execution is triggered by parameter change then this must be false as the
  // user will be currently interacting with the parameter controls.
  bool rebuild_parameters_;

  // The objects that are currently being drawn, and drawing state.
  ColorMap::Function colormap_;
  int brightness_;                      // In the range 0..1000
  std::vector<DebugText> debug_text_;   // Text emitted from s:DrawText()
  bool antialiasing_;
  bool show_markers_;

  // State used for sweeping and optimizing, i.e. when an "invisible hand" is
  // moving the parameter sliders.
  struct InvisibleHand {
    enum State { OFF, SWEEPING, OPTIMIZING } state;
    std::string sweep_parameter_name;         // Parameter name we're sweeping
    int sweep_index;                          // Current parameter value index
    std::vector<double> sweep_values;           // All values of parameter to use
    std::vector<std::vector<JetComplex> > sweep_output;
    OptimizerType optimizer_type;             // Algorithm to use
    CeresInteractiveOptimizer *optimizer;     // Nonzero if currently optimizing
    std::vector<std::string> opt_parameter_names;  // Parameter names optimizing
    bool optimizer_done_;                     // true if found optimal solution

    InvisibleHand() {
      optimizer_type = LEVENBERG_MARQUARDT;
      optimizer = 0;
      Start();
    }

    void Start() {
      Stop();
      // The sweep values, results vectors and parameter name are not cleared
      // by Stop(), to keep them around for plotting.
      sweep_values.clear();
      sweep_output.clear();
      sweep_parameter_name.clear();
    }
    void Stop() {
      state = OFF;
      sweep_index = 0;
      delete optimizer;
      optimizer = 0;
      opt_parameter_names.clear();
      optimizer_done_ = false;
    }
  };
  InvisibleHand ih_;

  // GLViewer overridden functions.
  void Draw();
  void HandleClick(int x, int y, bool button, const Eigen::Vector3d &model_pt);
  void HandleDrag(int x, int y, const Eigen::Vector3d &model_pt);

  // For marker n (offset into the markers_ array) return the x and y
  // coordinates of the marker.
  void GetMarkerPosition(int n, double *x, double *y);

  // Set the named parameter to the given value (updating all associated
  // controls). The value is clamped to the parameter min and max values.
  // Return true if the parameter name exists or false otherwise.
  bool SetParameter(const std::string &parameter_name, double value);

  // Called by OnIdle() to deal with the current invisible hand state. Return
  // true if there is more work left to do and we need more idle events. If
  // false is returned then the invisible hand state will be reset in OnIdle().
  bool OnInvisibleHandSweep();
  bool OnInvisibleHandOptimize();

  DECLARE_EVENT_TABLE()
};

#endif
