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

// A shell for applications that have Lua script driven creation of models,
// that are controllable by parameters and can be swept and optimized.

#ifndef __LUA_MODEL_VIEWER_QT_H__
#define __LUA_MODEL_VIEWER_QT_H__

#include <string>
#include <map>

#include "viewer.h"
#include "gl_utils.h"
#include "colormaps.h"
#include "optimizer.h"
#include "text_alignment.h"
#include "my_jet.h"
#include "../../toolkit/lua_util.h"

class MyLua;
class MyCheckBox;
class MySlider;
class MyTextCtrl;
class qtPlot;
class QWidget;
class QListWidget;
class QGridLayout;
class QIcon;

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
  Q_OBJECT

 public:
  explicit LuaModelViewer(QWidget *parent);
  ~LuaModelViewer();

  // Do background sweeps and optimization when idle.
 public slots:
  void IdleProcessing();
public:
  // Is the model good enough to simulate?
  bool IsModelValid() { return valid_; }

  // Handling for menu and control commands.
  void ZoomToExtents();
  void ZoomIn(double scale_factor = 0);
  void ZoomOut();
  void ToggleAntialiasing();
  void ToggleShowMarkers();
  void ToggleSwitchToModelAfterEachSolve();
  void SetDisplayColorScheme(int color_scheme);
  void SetBrightness(int brightness);
  void SetIfRunScriptResetsParameters(bool runscript_resets_param_map) {
    runscript_resets_param_map_ = runscript_resets_param_map;
  }
  void Sweep(const std::string &parameter_name,
             double start_value, double end_value, int num_steps,
             bool sweep_over_test_output, const std::string &image_filename);
  void Optimize();
  void StopSweepOrOptimize();
  void ToggleEmitTraceReport();
  void SetOptimizer(OptimizerType type) { ih_.optimizer_type = type; }
  void CopyParametersToClipboard();
  void ToggleRunTestAfterSolve();
  void ScriptTestMode();

  // Connect to external controls.
  void Connect(QListWidget *script_messages, QWidget *parameter_window,
               qtPlot *plot, QWidget *model_pane, QWidget *plot_pane,
               QWidget *script_messages_pane);

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
  // parameter values have changed. A new Lua state is created, and this state
  // will persist until the next time [Re]runScript() is called, so that e.g.
  // any callback functions created can be called.
  //
  // If optimize_output is nonzero return the output of the config.optimize
  // function, emitting an error and setting optimize_output to the empty
  // vector if no optimizer output could be obtained (e.g. because of script
  // error). If run_test_after_solve_ is true then run the config.test
  // function, which may emit an error, and set test_output to its return
  // values.
  //
  // The return value is false if there were script errors (this value is also
  // stored in valid_). If only_compute_derivatives is true then the script is
  // being redundantly rerun with the same parameters but different
  // derivatives.
  bool RerunScript(bool refresh_window = true,
                   std::vector<JetNum> *optimize_output = 0,
                   std::vector<JetNum> *test_output = 0,
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
  int LuaDistanceScale(lua_State *L);     // _DistanceScale

  // Accessors.
  ColorMap::Function GetColormap() const { return colormap_; }
  int GetBrightness() const { return brightness_; }
  qtPlot *GetMainPlot() { return plot_; }
  Lua *GetLua();
  bool AntiAliasing() const { return antialiasing_; }

  // Add a line to the "script messages" pane. This is callable from multiple
  // threads so that it can be called as part of an Error() callback.
  enum { ICON_BLANK = 0, ICON_INFO, ICON_WARNING, ICON_ERROR };
 signals:
  void AddScriptMessage(QString msg, int icon_number);
 public slots:
  void AddScriptMessageNotThreadSafe(QString msg, int icon_number);
 public:

  // Show and raise the given window. If sticky is true then ignore
  // further SelectPane calls for 100ms.
  void SelectPane(QWidget *win, bool sticky = false);

  // Return true if there are Lua errors.
  bool ThereAreLuaErrors();

  // If there were Lua errors, select the script messages pane and return true.
  bool SelectScriptMessagesIfErrors();

  // Select which kind of plot we want to display (the plot types are subclass
  // dependent).
  void SelectPlot(int plot_kind);

  // Convert a units name ('meter', 'mil', etc) to a distance scale in meters
  // or return -1 if the unit name is not known.
  double DistanceScale(const char *unit_name);

  // Export plot data to a matlab file.
  void ExportPlotMatlab(const char *filename);

  // Set command line arguments.
  void SetCommandLineArguments(int argc, char **argv) {
    copy_of_argc_ = argc;
    copy_of_argv_ = argv;
  }

  // ********** Override these to implement the model functionality.

  // Is there something to display? This is false if DrawModel() will draw
  // something.
  virtual bool IsModelEmpty()=0;

  // We are running a new script. Reset everything in preparation for a new
  // model being built. A fresh lua state has been constructed so populate it
  // with the required functions etc.
  virtual void ResetModel()=0;

  // We have just finished running a script without errors. Handle whatever
  // post-script computations need to be done. If only_compute_derivatives is
  // true then the script is being redundantly rerun with the same parameters
  // but different derivatives.
  virtual void ScriptJustRan(bool only_compute_derivatives)=0;

  // Push the Lua arguments for the optimize() or test() functions. If
  // real_invocation is true then this is a real call which should generate
  // actual results, otherwise this is a dummy call to make sure the function
  // runs.
  virtual void CreateArgumentsToOptimize(bool real_invocation)=0;

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
  bool valid_;                          // Is the model valid?
  int dragging_marker_;                 // >= 0 if now dragging a marker
  std::string script_;                  // The last script we ran
  MyLua *lua_;                          // Lua context for last script, 0=none
  int num_optimize_outputs_;            // Num values returned by optimize()
  bool in_rerun_script_;                // If RerunScript() is running
  bool emit_trace_report_;              // If printing trace report when idle
  int plot_type_;                       // Given to SelectPlot()
  QIcon *error_icon_, *warning_icon_, *info_icon_;
  bool scroll_to_bottom_of_script_messages_;
  bool run_test_after_solve_;
  bool script_test_mode_;
  int copy_of_argc_ = 0;                // Copy of argument given to main()
  char **copy_of_argv_ = 0;             // Copy of argument given to main()
  bool switch_to_model_after_each_solve_;
  bool disable_idle_processing_ = false;

  // Connections to external controls.
  QListWidget *script_messages_;
  QWidget *parameter_window_;
  QGridLayout *parameter_layout_;
  qtPlot *plot_;
 protected:
  // Used by subclasess in SelectPane():
  QWidget *model_pane_, *plot_pane_, *script_messages_pane_;
 private:

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
    std::vector<double> sweep_values;         // All values of parameter to use
    bool sweep_over_test_output;              // Plot test() output?
    std::vector<std::vector<JetComplex> > sweep_output;
    OptimizerType optimizer_type;             // Algorithm to use
    CeresInteractiveOptimizer *optimizer;     // Nonzero if currently optimizing
    std::vector<std::string> opt_parameter_names;  // Parameter names optimizing
    bool optimizer_done_;                     // true if found optimal solution
    std::string image_filename;               // nonempty to save model images

    InvisibleHand() {
      optimizer_type = LEVENBERG_MARQUARDT;
      optimizer = 0;
      sweep_over_test_output = false;
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
  void Draw() override;
  void HandleClick(int x, int y, bool button, const Eigen::Vector3d &model_pt) override;
  void HandleDrag(int x, int y, const Eigen::Vector3d &model_pt) override;

  // For marker n (offset into the markers_ array) return the x and y
  // coordinates of the marker.
  void GetMarkerPosition(int n, double *x, double *y);

  // Set the named parameter to the given value (updating all associated
  // controls). The value is clamped to the parameter min and max values.
  // Return true if the parameter name exists or false otherwise.
  bool SetParameter(const std::string &parameter_name, double value);

  // Called by IdleProcessing() to deal with the current invisible hand state.
  // Return true if there is more work left to do and we need more idle events.
  // If false is returned then the invisible hand state will be reset in
  // IdleProcessing().
  bool OnInvisibleHandSweep();
  bool OnInvisibleHandOptimize();
};

#endif
