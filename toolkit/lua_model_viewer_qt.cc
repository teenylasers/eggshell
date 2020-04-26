
#include <algorithm>
#include <string>
#include <stdint.h>
#include "lua_model_viewer_qt.h"
#include "gl_font.h"
#include "plot_gui.h"
#include "lua_vector.h"
#include "mystring.h"
#include "mat_file.h"
#include "trace.h"
#include "platform.h"
#include "shaders.h"

#include <QCheckBox>
#include <QLineEdit>
#include <QSlider>
#include <QListWidget>
#include <QGuiApplication>
#include <QClipboard>
#include <QGridLayout>
#include <QSpacerItem>
#include <QApplication>
#include <QAbstractEventDispatcher>
#include <QTimer>

using std::vector;
using std::string;
using Eigen::Vector2f;

const int kSliderResolution = 1000;
const int kMarkerSize = 5;

// Source code for the lua utility functions that are available to user
// scripts.
extern "C" char user_script_util_dot_lua;
extern "C" int user_script_util_dot_lua_length;

// Externally defined font.
extern Font debug_string_font;

//***************************************************************************
// Lua support.

class MyLua;

void LuaPanic(const char *message) {
  Panic("%s", message);
}

struct LuaErrorHandler : public ErrorHandler {
  MyLua *mylua_instance;
  void HandleError(Type type, const char *msg, va_list ap);
};

class MyLua : public Lua {
 public:
  MyLua(LuaModelViewer *model) : model_(model) {
    // Capture global errors to this object's Error() function.
    error_handler_.mylua_instance = this;
    previous_error_handler_ = SetErrorHandler(&error_handler_);
  }

  ~MyLua() {
    ErrorHandler *e = SetErrorHandler(previous_error_handler_);
    CHECK(e == &error_handler_);
  }

  void HandleStackBacktrace(const char *message) {
    Output(message, LuaModelViewer::ICON_INFO);
  }

  void HandleError(const char *message) {
    Output(message, LuaModelViewer::ICON_ERROR);
  }

  int Print() {
    std::string message;
    int n = lua_gettop(L());
    for (int i = 1; i <= n; i++) {
      message += LuaToString(L(), i);
      if (i < n) {
        message += ' ';
      }
    }
    Output(message.c_str(), LuaModelViewer::ICON_BLANK);
    return 0;
  }

  // Implement lua _CreateParameter(label, min, max, default, integer).
  int LuaCreateParameter() {
    return model_->LuaCreateParameter(L());
  }

  // Implement lua _CreateMarker().
  int LuaCreateMarker() {
    return model_->LuaCreateMarker(L());
  }

  // Implement lua ParameterDivider().
  int LuaParameterDivider() {
    return model_->LuaParameterDivider(L());
  }

  // Implement lua DrawText().
  int LuaDrawText() {
    return model_->LuaDrawText(L());
  }

  // Implement lua _DistanceScale().
  int LuaDistanceScale() {
    return model_->LuaDistanceScale(L());
  }

  // Create jets with derivatives (for debugging).
  int LuaJet() {
    if (lua_gettop(L()) != 2) {
      LuaError(L(), "Usage: Jet(value, derivative)");
    }
    JetNum n;
    n.a = ToDouble(luaL_checknumber(L(), 1));
    n.v()[0] = ToDouble(luaL_checknumber(L(), 2));
    lua_pushnumber(L(), n);
    return 1;
  }

  // Extract jet derivatives (for debugging).
  int LuaJetDerivative() {
    if (lua_gettop(L()) != 2) {
      LuaError(L(), "Usage: JetDerivative(number, derivative_index)");
    }
    JetNum number = luaL_checknumber(L(), 1);
    double index = ToDouble(luaL_checknumber(L(), 2));
    if (int(index) != index || index < 1 || index > JetNum::DIMENSION) {
      LuaError(L(), "Invalid derivative index");
    }
    lua_pushnumber(L(), number.v()[index - 1]);
    return 1;
  }

 private:
  LuaModelViewer *model_;
  LuaErrorHandler error_handler_;
  ErrorHandler *previous_error_handler_;

  // Note: this must be thread safe as it's called by LuaErrorHandler.
  void Output(const char *message, int icon_number) {
    // If the message contains multiple lines separated by \n's then insert
    // each one separately.
    if (strchr(message, '\n')) {
      char *copy = strdup(message), *next = 0;
      for (char *part = copy; part && *part; part = next) {
        next = strchr(part, '\n');
        if (next) {
          *next = 0;
          next++;
        }
        model_->AddScriptMessage(part, icon_number);
      }
      free(copy);
    } else {
      model_->AddScriptMessage(message, icon_number);
    }
  }

  friend struct LuaErrorHandler;
};

void LuaErrorHandler::HandleError(Type type, const char *msg, va_list ap) {
  // We need separate copies of 'ap' for all users, as users will mutate it.
  va_list ap2;
  va_copy(ap2, ap);
  if (type == Error) {
    string message;
    StringVPrintf(&message, msg, ap2);
    mylua_instance->Error(message.c_str());
  }
  mylua_instance->previous_error_handler_->HandleError(type, msg, ap);
}

//***************************************************************************
// Custom controls for parameter changing.

class MyCheckBox : public QCheckBox {
 public:
  MyCheckBox(QWidget *parent, Parameter *param)
          : QCheckBox(param->label.c_str(), parent) {
    param_ = param;
    my_index_ = my_checkbox_count_;
    shift_click_ = false;
    checkbox_map_[my_index_] = this;
    my_checkbox_count_++;
    QObject::connect(this, &MyCheckBox::stateChanged,
                     this, &MyCheckBox::OnStateChange);
  }
  ~MyCheckBox() {
    checkbox_map_.erase(my_index_);
  }
  void mousePressEvent(QMouseEvent *event) override {
    shift_click_ = ((event->modifiers() & Qt::ShiftModifier) != 0);
    QCheckBox::mousePressEvent(event);
  }

  void OnStateChange(int state) {
    // When the user shift-clicks on this checkbox, change the state of all
    // checkboxes between this one and the last one clicked. This allows ranges
    // of checkboxes to be toggled quickly.
    if (shift_click_ && last_clicked_ != my_index_) {
      for (int i = last_clicked_; i != my_index_;
           i += (last_clicked_ < my_index_) ? 1 : -1) {
        auto it = checkbox_map_.find(i);
        if (it != checkbox_map_.end()) {
          it->second->setChecked(state);
        }
      }
    }
    last_clicked_ = my_index_;
  }

 private:
  Parameter *param_;
  int64_t my_index_;                    // All checkboxes get a unique index
  bool shift_click_;                    // If the last click was a shift+click
  static int64_t my_checkbox_count_;    // How many MyCheckBoxes have been made
  static int64_t last_clicked_;         // The last OnClick()ed checkbox
  static std::map<int64_t, MyCheckBox*> checkbox_map_;
};

int64_t MyCheckBox::my_checkbox_count_;
int64_t MyCheckBox::last_clicked_;
std::map<int64_t, MyCheckBox*> MyCheckBox::checkbox_map_;

class MyTextCtrl : public QLineEdit {
 public:
  MyTextCtrl(QWidget *parent, LuaModelViewer *model, Parameter *param)
        : QLineEdit(parent) {
    model_ = model;
    param_ = param;
    slider_ = 0;
    QWidget::connect(this, &MyTextCtrl::returnPressed,
                     this, &MyTextCtrl::OnReturnPressed);
    QWidget::connect(this, &MyTextCtrl::textChanged,
                     this, &MyTextCtrl::OnTextChanged);
  }

  void LinkToSlider(MySlider *slider) {
    slider_ = slider;
  }

  void ReflectParameterValue() {
    setText(QString::asprintf("%g", param_->value));
  }

  void OnTextChanged(const QString &text);
  void OnReturnPressed();
  void focusOutEvent(QFocusEvent *event) override;

 private:
  LuaModelViewer *model_;
  Parameter *param_;
  MySlider *slider_;
};

class MySlider : public QSlider {
 public:
  MySlider(QWidget *parent, LuaModelViewer *model, MyTextCtrl *text_ctrl,
           Parameter *param)
    : QSlider(Qt::Horizontal, parent) {
    setMinimum(param->integer ? param->the_min : 0);
    setMaximum(param->integer ? param->the_max : kSliderResolution);
    setTracking(true);
    model_ = model;
    param_ = param;
    text_ctrl_ = text_ctrl;
    QWidget::connect(this, &MySlider::valueChanged,
                     this, &MySlider::OnValueChanged);
  }

  // Set the slider to reflect the parameter value but don't emit any signals.
  void ReflectParameterValue() {
    blockSignals(true);
    if (param_->integer) {
      setValue(round(param_->value));
    } else {
      setValue(round((param_->value - param_->the_min) /
          (param_->the_max - param_->the_min) * kSliderResolution));
    }
    blockSignals(false);
  }

  void OnValueChanged(int value);

 private:
  LuaModelViewer *model_;
  Parameter *param_;
  MyTextCtrl *text_ctrl_;
};

void MyTextCtrl::OnTextChanged(const QString &text) {
  // Set the control background based on whether this is a valid number or not.
  bool ok;
  text.toDouble(&ok);
  if (ok) {
    setStyleSheet("QLineEdit {}");
  } else {
    setStyleSheet("QLineEdit { background-color : red; }");
  }
}

void MyTextCtrl::OnReturnPressed() {
  bool ok;
  double value = text().toDouble(&ok);
  if (ok) {
    if (param_->integer) {
      value = round(value);
    }
    param_->value = std::max(param_->the_min, std::min(param_->the_max, value));
  } else {
    param_->value = param_->the_default;
  }
  ReflectParameterValue();
  slider_->ReflectParameterValue();
  model_->RerunScript();
}

void MyTextCtrl::focusOutEvent(QFocusEvent *event) {
  OnReturnPressed();
  QLineEdit::focusOutEvent(event);
}

void MySlider::OnValueChanged(int value) {
  if (param_->integer) {
    param_->value = value;
  } else {
    param_->value = param_->the_min +
        (double(value) / kSliderResolution *
        (param_->the_max - param_->the_min));
  }
  if (text_ctrl_) {
    text_ctrl_->ReflectParameterValue();
  }
  model_->RerunScript();
}

//***************************************************************************
// LuaModelViewer.

//@@@ gl_type ignored here is the GL state is set globally, hmm.

LuaModelViewer::LuaModelViewer(QWidget *parent, int gl_type)
    : GLViewer(parent) {
  valid_ = false;
  dragging_marker_ = -1;
  lua_ = 0;
  num_optimize_outputs_ = 0;
  in_rerun_script_ = false;
  emit_trace_report_ = false;
  plot_type_ = 0;
  scroll_to_bottom_of_script_messages_ = false;
  script_messages_ = 0;
  parameter_window_ = 0;
  parameter_layout_ = 0;
  plot_ = 0;
  model_pane_ = plot_pane_ = script_messages_pane_ = 0;
  runscript_resets_param_map_ = true;
  num_ticked_count_ = 0;
  derivative_index_ = 0;
  rebuild_parameters_ = false;
  colormap_ = ColorMap::Wave;
  brightness_ = 500;
  antialiasing_ = true;
  show_markers_ = true;
  run_test_after_solve_ = false;
  script_test_mode_ = false;
  switch_to_model_after_each_solve_ = true;

  // Setup the camera for 2D operation. This can be changed in a subclass.
  SetPerspective(false);
  AllowCameraRotation(false);

  // Set up a signal/slot connection that for AddScriptMessage that works
  // across threads.
  QObject::connect(this, SIGNAL(AddScriptMessage(QString, int)),
                   this, SLOT(AddScriptMessageNotThreadSafe(QString, int)));

  // Create icons for the script messages window.
  error_icon_ = new QIcon;
  warning_icon_ = new QIcon;
  info_icon_ = new QIcon;
  *error_icon_ = qApp->style()->standardIcon(QStyle::SP_MessageBoxCritical);
  *warning_icon_ = qApp->style()->standardIcon(QStyle::SP_MessageBoxWarning);
  *info_icon_ = qApp->style()->standardIcon(QStyle::SP_MessageBoxInformation);

  // Setup the idle event processing mechanism.
  auto *dispatcher = QAbstractEventDispatcher::instance();
  connect(dispatcher, SIGNAL(aboutToBlock()),
          this, SLOT(IdleProcessing()));
}

LuaModelViewer::~LuaModelViewer() {
  delete lua_;
  //@@@ Deleting this causes a crash, why?: delete parameter_layout_;
  delete error_icon_;
  delete warning_icon_;
  delete info_icon_;
}

void LuaModelViewer::IdleProcessing() {
  // This is called from a single shot QTimer or when the event loop is about
  // to block (i.e. when there is no other work to do in the system. It's a
  // good place to start traces and to collect trace reports.
  string report;
  TraceReport(&report);
  if (!report.empty() && emit_trace_report_) {
    printf("%s\n", report.c_str());
  }
  TraceStart();

  // If necessary scroll to the bottom of the script messages list so the most
  // recently added item is visible. This is quite slow so we only do it when
  // idle.
  if (scroll_to_bottom_of_script_messages_) {
    script_messages_->scrollToItem(
          script_messages_->item(script_messages_->count() - 1));
    scroll_to_bottom_of_script_messages_ = false;
  }

  // Deal with the current "invisible hand" state.
  bool want_more_idles = false;
  if (ih_.state == InvisibleHand::SWEEPING) {
    want_more_idles = OnInvisibleHandSweep();
  } else if (ih_.state == InvisibleHand::OPTIMIZING) {
    want_more_idles = OnInvisibleHandOptimize();
  }
  if (want_more_idles) {
    QTimer::singleShot(0, this, &LuaModelViewer::IdleProcessing);
  } else {
    // ih_.Stop() makes sure that future idle processing does nothing as the
    // state will be OFF.
    ih_.Stop();
  }
}

void LuaModelViewer::ZoomToExtents() {
  if (IsModelEmpty()) {
    Warning("Nothing is visible yet so you can't zoom to extents");
  } else {
    ZoomExtents();
  }
}

void LuaModelViewer::ZoomIn(double scale_factor) {
  if (scale_factor == 0) {
    scale_factor = 1.0 / sqrt(2);
  }
  Zoom(scale_factor);
}

void LuaModelViewer::ZoomOut() {
  ZoomIn(sqrt(2));
}

void LuaModelViewer::ToggleAntialiasing() {
  antialiasing_ = !antialiasing_;
  update();
}

void LuaModelViewer::ToggleShowMarkers() {
  show_markers_ = !show_markers_;
  update();
}

void LuaModelViewer::ToggleSwitchToModelAfterEachSolve() {
  switch_to_model_after_each_solve_ = !switch_to_model_after_each_solve_;
}

void LuaModelViewer::SetDisplayColorScheme(int color_scheme) {
  if (color_scheme == 0) {
    colormap_ = ColorMap::Wave;
  } else if (color_scheme == 1) {
    colormap_ = ColorMap::Jet;
  } else if (color_scheme == 2) {
    colormap_ = ColorMap::Hot;
  } else if (color_scheme == 3) {
    colormap_ = ColorMap::Gray;
  } else if (color_scheme == 4) {
    colormap_ = ColorMap::HSV;
  } else if (color_scheme == 5) {
    colormap_ = ColorMap::Bone;
  } else if (color_scheme == 6) {
    colormap_ = ColorMap::Copper;
  } else if (color_scheme == 7) {
    colormap_ = ColorMap::Wheel;
  } else {
    Panic("Internal: bad color_scheme = %d", color_scheme);
  }
  update();
}

void LuaModelViewer::SetBrightness(int brightness) {
  brightness_ = brightness;
  update();
}

void LuaModelViewer::Sweep(const string &parameter_name,
                           double start_value, double end_value,
                           int num_steps, bool sweep_over_test_output) {
  if (!IsModelValid()) {
    Error("The model is not yet valid");
    return;
  }

  // Sweeping parameters is the same as an invisible hand moving the slider and
  // recording the results after each solve.
  ih_.Start();
  ih_.sweep_over_test_output = sweep_over_test_output;
  const Parameter &p = GetParameter(parameter_name);
  if (p.integer) {
    // Integer parameter.
    CHECK(start_value == int(start_value));
    CHECK(end_value == int(end_value));
    CHECK(start_value >= p.the_min && end_value <= p.the_max);
    num_steps = end_value - start_value + 1;
  }
  CHECK(num_steps > 0);
  ih_.sweep_parameter_name = parameter_name;
  ih_.sweep_values.resize(num_steps);
  ih_.sweep_output.resize(num_steps);
  if (num_steps == 1) {
    ih_.sweep_values[0] = start_value;
  } else {
    for (int i = 0; i < num_steps; i++) {
      ih_.sweep_values[i] = double(i) / double(num_steps - 1) *
                            (end_value - start_value) + start_value;
    }
  }
  ih_.state = InvisibleHand::SWEEPING;
  // Run IdleProcessing() to kick off the actual sweep.
  QTimer::singleShot(0, this, &LuaModelViewer::IdleProcessing);
}

void LuaModelViewer::Optimize() {
  // Optimizing parameters is the same as an invisible hand moving the sliders
  // and recording the results after each solve.
  ih_.Start();

  if (!IsModelValid()) {
    Error("The model is not yet valid");
    return;
  }

  // Make sure the config.optimize() function is defined and useful.
  if (num_optimize_outputs_ <= 0) {
    Error("The config.optimize() function must return one or more error "
            "values.");
    return;
  }

  // Find all the parameters we're going to optimize.
  ih_.opt_parameter_names.clear();
  for (int i = 0; i < current_param_controls_.size(); i++) {
    const Parameter &p = GetParameter(current_param_controls_[i]);
    CHECK(p.checkbox);
    if (p.checkbox->isChecked()) {
      ih_.opt_parameter_names.push_back(p.label);
      if (p.integer) {
        Error("Can not optimize the integer parameter '%s'.", p.label.c_str());
        return;
      }
    }
  }
  if (ih_.opt_parameter_names.size() == 0) {
    Error("No parameters selected to optimize.");
    return;
  }

  // Create the optimizer if necessary.
  ih_.optimizer = new CeresInteractiveOptimizer(num_optimize_outputs_,
                                                ih_.optimizer_type);
  ih_.optimizer->SetFunctionTolerance(1e-6);    //@@@ revisit
  ih_.optimizer->SetParameterTolerance(1e-4);   //@@@ revisit
  // Gradients computed from mesh derivatives are not perfectly accurate, so:
  ih_.optimizer->SetGradientTolerance(0);

  // Start optimization.
  ih_.state = InvisibleHand::OPTIMIZING;
  // Run IdleProcessing() to kick off the actual optimization.
  QTimer::singleShot(0, this, &LuaModelViewer::IdleProcessing);
}

void LuaModelViewer::StopSweepOrOptimize() {
  ih_.Stop();
}

void LuaModelViewer::ToggleEmitTraceReport() {
  emit_trace_report_ = !emit_trace_report_;
}

void LuaModelViewer::CopyParametersToClipboard() {
  QString s = "default_parameters = {\n";
  for (int i = 0; i < current_param_controls_.size(); i++) {
    const Parameter &p = GetParameter(current_param_controls_[i]);
    s.append(QString::asprintf("  ['%s'] = %.10g,\n",
                               current_param_controls_[i].c_str(), p.value));
  }
  s.append("}\n");
  QGuiApplication::clipboard()->setText(s);
}

void LuaModelViewer::ToggleRunTestAfterSolve() {
  run_test_after_solve_ ^= 1;
}

void LuaModelViewer::ScriptTestMode() {
  script_test_mode_ = true;
}

void LuaModelViewer::Connect(QListWidget *script_messages,
                             QWidget *parameter_window, qtPlot *plot,
                             QWidget *model_pane, QWidget *plot_pane,
                             QWidget *script_messages_pane) {
  script_messages_ = script_messages;
  parameter_window_ = parameter_window;
  plot_ = plot;
  model_pane_ = model_pane;
  plot_pane_ = plot_pane;
  script_messages_pane_ = script_messages_pane;
}

bool LuaModelViewer::RunScript(const char *script, bool rerun_even_if_same,
                               vector<JetNum> *optimize_output) {
  if (script_ == script && !rerun_even_if_same) {
    return false;
  }
  script_ = script;

  // Issue fresh complaints for each new load of this model.
  ResetComplaints();

  // Remove any existing parameter widgets and prepare to rebuild the layout.
  {
    delete parameter_layout_;
    parameter_layout_ = new QGridLayout(parameter_window_);
    parameter_layout_->setSpacing(5);
    parameter_layout_->setContentsMargins(0, 0, 0, 0);
    qDeleteAll(parameter_window_->findChildren<QWidget*>
                 (QString(), Qt::FindDirectChildrenOnly));
  }
  current_param_controls_.clear();

  // To catch bugs, zero all control references in the param_map_ since we just
  // deleted all those controls.
  for (ParamMap::iterator it = param_map_.begin();
       it != param_map_.end(); ++it) {
    it->second.text_ctrl = 0;
    it->second.slider = 0;
    it->second.text_ctrl = 0;
  }

  // Optionally clear the parameters map. Note that we do this *after* the
  // existing parameter controls have been deleted, as they contain references
  // to these Parameters.
  if (runscript_resets_param_map_) {
    param_map_.clear();
  }

  // Reset the invisible hand state in case we're in the middle of a sweep or
  // optimizing.
  ih_.Stop();

  // Run script.
  rebuild_parameters_ = true;
  RerunScript(true, optimize_output);
  rebuild_parameters_ = false;

  // Add spacing to the bottom of the parameter pane to push all the existing
  // rows to the top.
  parameter_layout_->addItem(new QSpacerItem(0, 0),
                             parameter_layout_->rowCount(), 0);
  parameter_layout_->setRowStretch(parameter_layout_->rowCount()-1, 1);

  // Make sure the whole model is visible in all cameras.
  if (!IsModelEmpty()) {
    int c = GetCameraIndex();
    for (int i = 0; i < MAX_CAMERAS; i++) {
      SwitchCamera(i);
      ZoomToExtents();
    }
    SwitchCamera(c);
  }

  return true;
}

bool LuaModelViewer::RerunScript(bool refresh_window,
                                 vector<JetNum> *optimize_output,
                                 vector<JetNum> *test_output,
                                 bool only_compute_derivatives) {
  if (in_rerun_script_) {
    // The custom controls for parameter changing can sometimes cause this
    // function to indirectly call itself, e.g. through SelectPane(). Prevent
    // such recursion.
    return true;
  }
  Trace trace(__func__);
  in_rerun_script_ = true;
  num_ticked_count_ = 0;
  valid_ = false;               // Assumption, updated below

  // optimize_output will be returned empty on any error, e.g. on script error
  // or if we can't find the optimize function or solve.
  if (optimize_output) {
    optimize_output->clear();
  }

  // If script_ is the empty string then RunScript() might not have been called
  // but it's possible also that the user tried to run an empty file.
  if (script_.empty()) {
    in_rerun_script_ = false;
    return false;
  }

  // Reset state, setup lua context. NOTE that MyLua will capture both calls to
  // lua.Error() and the global Error() function, so lua.ThereWereErrors() can
  // be used to check for either kind of error.
  script_messages_->clear();
  debug_text_.clear();
  markers_.clear();
  delete lua_;
  lua_ = new MyLua(this);
  lua_->UseStandardLibraries(true);
  LuaVector::SetLuaGlobals(lua_->L());
  ResetModel();                 // Subclass resets model and adds to lua state

  // Register our functions and classes.
  LuaUserClassRegister<LuaVector>(*lua_, "Vector");
  lua_pushcfunction(lua_->L(),
                    (LuaGlobalStub<MyLua, &MyLua::LuaCreateParameter>));
  lua_setglobal(lua_->L(), "_CreateParameter");
  lua_pushcfunction(lua_->L(), (LuaGlobalStub<MyLua, &MyLua::LuaCreateMarker>));
  lua_setglobal(lua_->L(), "_CreateMarker");
  lua_pushcfunction(lua_->L(),
                    (LuaGlobalStub<MyLua, &MyLua::LuaParameterDivider>));
  lua_setglobal(lua_->L(), "ParameterDivider");
  lua_pushcfunction(lua_->L(), (LuaGlobalStub<MyLua, &MyLua::LuaDrawText>));
  lua_setglobal(lua_->L(), "DrawText");
  lua_pushcfunction(lua_->L(), (LuaGlobalStub<MyLua, &MyLua::LuaJet>));
  lua_setglobal(lua_->L(), "_Jet");
  lua_pushcfunction(lua_->L(),
                    (LuaGlobalStub<MyLua, &MyLua::LuaJetDerivative>));
  lua_setglobal(lua_->L(), "_JetDerivative");
  lua_pushcfunction(lua_->L(), (LuaGlobalStub<MyLua,
                    &MyLua::LuaDistanceScale>));
  lua_setglobal(lua_->L(), "_DistanceScale");

  // Populate the global 'FLAGS' table with the -key=value command line
  // arguments.
  lua_newtable(lua_->L());                                      // stack: T
  for (int i = 1; i < copy_of_argc_; i++) {
    char *arg = copy_of_argv_[i];
    if (arg[0] == '-') {
      char *equals = strchr(arg, '=');
      if (equals && equals >= arg + 2) {
        lua_pushlstring(lua_->L(), arg + 1, equals - arg - 1);  // stack: T k
        lua_pushstring(lua_->L(), equals + 1);                  // stack: T k v
        lua_rawset(lua_->L(), -3);                              // stark: T
      }
    }
  }
  lua_setglobal(lua_->L(), "FLAGS");

  // Run the script. First load the lua utility functions that are available to
  // user scripts.
  string user_script_util(&user_script_util_dot_lua,
                          user_script_util_dot_lua_length);
  if (!lua_->RunString(user_script_util.c_str())) {
    // On script failure an error message will have been shown. But this should
    // never happen.
    lua_->Error("Internal error in script utility code");
  } else {
    if (!lua_->RunString(script_.c_str())) {
      // On script failure an error message should have been shown.
      CHECK(lua_->ThereWereErrors());
    }
  }

  // Check that a config table was left behind.
  LuaRawGetGlobal(lua_->L(), "config");
  if (lua_type(lua_->L(), -1) != LUA_TTABLE) {
    lua_->Error("The script should leave behind a 'config' table");
  }
  lua_pop(lua_->L(), 1);

  // Run subclass code.
  if (!lua_->ThereWereErrors()) {
    ScriptJustRan(only_compute_derivatives);
  }

  // Call the script's optimize() function. If the optimize_output is requested
  // then first compute the solution and call it with real inputs. Otherwise
  // call it with dummy input just to count how many error values are returned.
  num_optimize_outputs_ = 0;            // Initial assumption
  if (!lua_->ThereWereErrors()) {
    // Find the function. Since there are no errors so far, the config table is
    // guaranteed to exist.
    LuaRawGetGlobal(lua_->L(), "config");
    lua_getfield(lua_->L(), -1, "optimize");
    int top = lua_gettop(lua_->L());
    if (lua_type(lua_->L(), -1) != LUA_TFUNCTION) {
      // Since the function is optional, only complain about a missing function
      // if we're trying to compute optimize_output.
      if (optimize_output) {
        lua_->Error("Script does not define the config.optimize() function");
      }
    } else {
      CreateArgumentsToOptimize(optimize_output != 0);

      // Call the optimize function.
      if (!lua_->ThereWereErrors()) {
        int err = lua_->PCall(3, LUA_MULTRET);
        if (err != LUA_OK) {
          lua_->Error("The config.optimize() function failed");
        } else {
          num_optimize_outputs_ = lua_gettop(lua_->L()) - top + 1;
          for (int i = 0; i < num_optimize_outputs_; i++) {
            if (optimize_output) {
              JetNum out = lua_tonumber(lua_->L(), top + i);
              // If any infinities or NaNs are returned from config.optimize()
              // (either values or derivatives) we pass them directly to the
              // optimizer to deal with. A too-conservative alternative is to
              // bail, below:
              //   if (!IsFinite(out)) {
              //     Error("Return value %d of config.optimize() (or its "
              //           "derivative) is not finite.", i + 1);
              //   }
              optimize_output->push_back(out);
            }
          }
        }
      }
    }
  }

  // Call the script's test() function, if requested.
  if (!lua_->ThereWereErrors() && run_test_after_solve_) {
    // Find the function. Since there are no errors so far, the config table is
    // guaranteed to exist.
    LuaRawGetGlobal(lua_->L(), "config");
    lua_getfield(lua_->L(), -1, "test");
    int top = lua_gettop(lua_->L());
    if (lua_type(lua_->L(), -1) != LUA_TFUNCTION) {
      lua_->Error("Script does not define the config.test() function");
    } else {
      CreateArgumentsToOptimize(true);
      if (!lua_->ThereWereErrors()) {
        int err = lua_->PCall(3, LUA_MULTRET);
        if (err != LUA_OK) {
          lua_->Error("The config.test() function failed");
        } else if (test_output) {
          int n = lua_gettop(lua_->L()) - top + 1;
          for (int i = 0; i < n; i++) {
            test_output->push_back(lua_tonumber(lua_->L(), top + i));
          }
        }
      }
    }
    // In script test mode, exit after running the test function.
    if (script_test_mode_) {
      exit(lua_->ThereWereErrors());
    }
  }

  // Select the script messages pane if there were errors, otherwise select the
  // pane that shows the computational domain.
  if (!SelectScriptMessagesIfErrors()) {
    if (switch_to_model_after_each_solve_) {
      SelectPane(model_pane_);
    }
  }

  // If there were any script errors we assume that the model and config_ are
  // not valid.
  valid_ = !lua_->ThereWereErrors();

  // Redraw window right now, if requested.
  if (refresh_window) {
    repaint();
  }

  in_rerun_script_ = false;
  return valid_;
}

const Parameter &LuaModelViewer::GetParameter(const std::string &parameter_name)
{
  ParamMap::const_iterator it = param_map_.find(parameter_name);
  CHECK(it != param_map_.end());
  return it->second;
}

void LuaModelViewer::GetParamControlNames(vector<string> *names) const {
  *names = current_param_controls_;
}

int LuaModelViewer::LuaCreateParameter(lua_State *L) {
  // This is called during script execution as _CreateParameter(label, min,
  // max, default, integer).
  if (lua_gettop(L) != 5) {
    LuaError(L, "Internal error: Expecting 5 arguments");
  }

  // Extract arguments and return the current parameter value. If the parameter
  // has an entry in param_map_ then use the cached value, otherwise use the
  // default.
  const char *label = lua_tostring(L, 1);
  Parameter &p = param_map_[label];
  JetNum the_min = lua_tonumber(L, 2);
  JetNum the_max = lua_tonumber(L, 3);
  if (p.label.empty()) {
    // Previously unseen parameters get default values.
    p.the_min = ToDouble(the_min);
    p.the_max = ToDouble(the_max);
    p.the_default = ToDouble(lua_tonumber(L, 4));
    p.value = p.the_default;
  } else {
    // For previously seen parameters we make sure that the min/max have not
    // changed so that the existing parameter values will still be valid.
    if (p.the_min != ToDouble(the_min) || p.the_max != ToDouble(the_max)) {
      LuaError(L, "The min or max for this parameter has unexpectedly "
                  "changed.");
    }
    p.the_default = ToDouble(lua_tonumber(L, 4));
  }
  if (the_min.Derivative() != 0 || the_max.Derivative() != 0) {
    LuaError(L, "Parameter min and max values can not depend on other "
                "parameters, as this will confuse the optimizer.");
  }
  p.label = label;              // Lua Parameter() fn checks label not empty
  p.integer = lua_toboolean(L, 5);
  JetNum value = p.value;

  // Keep count of the number of checkbox-ticked parameters seen so far. The
  // n'th checked parameters gets to have a derivative:
  if (p.checkbox && p.checkbox->isChecked()) {
    if (!rebuild_parameters_ && derivative_index_ == num_ticked_count_) {
      value.Derivative() = 1;
    }
    num_ticked_count_++;
  }

  // Return value of lua function.
  lua_pushnumber(L, value);

  // Create user interface elements if requested. It's a problem if we already
  // have user interface elements with this name.
  if (rebuild_parameters_) {
    for (int i = 0; i < current_param_controls_.size(); i++) {
      if (label == current_param_controls_[i]) {
        LuaError(L, "Can not add '%s' twice", label);
      }
    }
    int row = parameter_layout_->rowCount();
    CHECK(!p.checkbox && !p.text_ctrl && !p.slider);
    p.checkbox = new MyCheckBox(parameter_window_, &p);
    p.checkbox->setChecked(false);
    parameter_layout_->addWidget(p.checkbox, row, 0);
    p.text_ctrl = new MyTextCtrl(parameter_window_, this, &p);
    p.text_ctrl->ReflectParameterValue();
    parameter_layout_->addWidget(p.text_ctrl, row, 1);
    p.slider = new MySlider(parameter_window_, this, p.text_ctrl, &p);
    p.slider->ReflectParameterValue();
    parameter_layout_->addWidget(p.slider, row, 2);
    p.text_ctrl->LinkToSlider(p.slider);
    current_param_controls_.push_back(label);
  }

  return 1;
}

int LuaModelViewer::LuaCreateMarker(lua_State *L) {
  if (lua_gettop(L) != 2) {
    LuaError(L, "Internal error: Expecting 2 arguments");
  }
  markers_.push_back(lua_tostring(L, 1));
  markers_.push_back(lua_tostring(L, 2));
  return 0;
}

int LuaModelViewer::LuaParameterDivider(lua_State *L) {
  if (lua_gettop(L) != 0) {
    LuaError(L, "ParameterDivider() does not take any arguments");
  }
  if (rebuild_parameters_) {
    QFrame *line = new QFrame(parameter_window_);
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    parameter_layout_->addWidget(line, parameter_layout_->rowCount(), 0, 1, -1);
  }
  return 0;
}

int LuaModelViewer::LuaDrawText(lua_State *L) {
  int n = lua_gettop(L);
  if (n < 3 || n > 5) {
    LuaError(L, "Usage: _DrawText(x,y,s,[ha],[va])");
  }
  TextAlignment halign = TEXT_ALIGN_LEFT;
  TextAlignment valign = TEXT_ALIGN_BASELINE;
  if (n >= 4 && lua_type(L, 4) != LUA_TNIL) {
    const char *ha = LuaToString(L, 4);
    if (strcmp(ha, "left") == 0) {
      halign = TEXT_ALIGN_LEFT;
    } else if (strcmp(ha, "right") == 0) {
      halign = TEXT_ALIGN_RIGHT;
    } else if (strcmp(ha, "center") == 0) {
      halign = TEXT_ALIGN_CENTER;
    } else {
      LuaError(L, "Bad 'ha' argument to DrawText()");
    }
  }
  if (n >= 5 && lua_type(L, 5) != LUA_TNIL) {
    const char *va = LuaToString(L, 5);
    if (strcmp(va, "top") == 0) {
      valign = TEXT_ALIGN_TOP;
    } else if (strcmp(va, "bottom") == 0) {
      valign = TEXT_ALIGN_BOTTOM;
    } else if (strcmp(va, "center") == 0) {
      valign = TEXT_ALIGN_CENTER;
    } else if (strcmp(va, "baseline") == 0) {
      valign = TEXT_ALIGN_BASELINE;
    } else {
      LuaError(L, "Bad 'va' argument to DrawText()");
    }
  }

  debug_text_.emplace_back(ToDouble(luaL_checknumber(L, 1)),
                           ToDouble(luaL_checknumber(L, 2)), LuaToString(L, 3),
                           halign, valign);
  return 0;
}

int LuaModelViewer::LuaDistanceScale(lua_State *L) {
  lua_pushnumber(L, DistanceScale(lua_tostring(L, 1)));
  return 1;
}

Lua *LuaModelViewer::GetLua() {
  return lua_;
}

void LuaModelViewer::AddScriptMessageNotThreadSafe(QString msg,
                                                   int icon_number) {
  auto item = new QListWidgetItem(msg, 0);
  if (icon_number == ICON_ERROR) {
    item->setTextColor(QColor(255, 0, 0));
    item->setIcon(*error_icon_);
  } else if (icon_number == ICON_WARNING) {
    item->setTextColor(QColor(255, 128, 0));
    item->setIcon(*warning_icon_);
  } else if (icon_number == ICON_INFO) {
    item->setTextColor(QColor(0, 0, 255));
    item->setIcon(*info_icon_);
  }
  script_messages_->insertItem(script_messages_->count(), item);
  // Scroll to the bottom of the script messages list so the most recently
  // added item is visible. script_messages_->scrollToItem() is quite slow so
  // we make sure that it's only called once, when idle.
  scroll_to_bottom_of_script_messages_ = true;

  // Send script messages to standard output in script test mode.
  if (script_test_mode_) {
    printf("%s\n", msg.toUtf8().data());
  }
}

void LuaModelViewer::SelectPane(QWidget *win, bool sticky) {
  // If sticky is true then ignore further SelectPane calls() for 100ms. This
  // is necessary e.g. when switching to the plots pane at the end of a sweep,
  // in case there is also a paint event pending which would run Draw() and
  // switch us back.
  static QTimer *timer = 0;
  if (!timer) {
    timer = new QTimer;
    timer->setSingleShot(true);
  }
  if (timer->isActive()) {
    return;
  }
  if (sticky) {
    timer->start(100);
  }

  win->show();
  win->raise();
}

bool LuaModelViewer::ThereAreLuaErrors() {
  return lua_->ThereWereErrors();
}

bool LuaModelViewer::SelectScriptMessagesIfErrors() {
  if (lua_->ThereWereErrors()) {
    SelectPane(script_messages_pane_);
    return true;
  }
  return false;
}

void LuaModelViewer::SelectPlot(int plot_type) {
  plot_type_ = plot_type;
  PlotSweepResults(plot_type, ih_.sweep_parameter_name, ih_.sweep_values,
                   ih_.sweep_output);
}

double LuaModelViewer::DistanceScale(const char *unit_name) {
  if (unit_name == 0)
    return -1;
  if (strcmp(unit_name, "nm") == 0)
    return 1e-9;
  if (strcmp(unit_name, "micron") == 0)
    return 1e-6;
  if (strcmp(unit_name, "mm") == 0)
    return 1e-3;
  if (strcmp(unit_name, "cm") == 0)
    return 1e-2;
  if (strcmp(unit_name, "m") == 0)
    return 1;
  if (strcmp(unit_name, "km") == 0)
    return 1000;
  if (strcmp(unit_name, "mil") == 0 || strcmp(unit_name, "thou") == 0)
    return 2.54e-5;
  if (strcmp(unit_name, "inch") == 0)
    return 2.54e-2;
  if (strcmp(unit_name, "foot") == 0)
    return 0.3048;
  if (strcmp(unit_name, "yard") == 0)
    return 0.9144;
  if (strcmp(unit_name, "mile") == 0)
    return 1609.344;
  return -1;
}

void LuaModelViewer::ExportPlotMatlab(const char *filename) {
  if (ih_.state != InvisibleHand::OFF) {
    Error("Can't save plot data until sweep or optimization is finished");
    return;
  }
  if (ih_.sweep_output.empty() || ih_.sweep_output[0].empty()) {
    Error("No plot data to save");
    return;
  }
  for (int i = 1; i < ih_.sweep_output.size(); i++) {
    if (ih_.sweep_output[i].size() != ih_.sweep_output[i-1].size()) {
      Error("Sweep output data is inconsistent, can't save it");
      return;
    }
  }
  MatFile mat(filename);
  int dims[2] = {0, 1};
  dims[0] = ih_.sweep_output.size();            // Rows = points per trace
  CHECK(dims[0] == ih_.sweep_values.size());
  mat.WriteMatrix("x", 2, dims, MatFile::mxDOUBLE_CLASS,
                  ih_.sweep_values.data(), 0);
  dims[1] = ih_.sweep_output[0].size();         // One column per trace
  vector<double> rdata(dims[0] * dims[1]), idata(dims[0] * dims[1]);
  for (int i = 0; i < dims[0]; i++) {
    for (int j = 0; j < dims[1]; j++) {
      rdata[i + j * dims[0]] = ToDouble(ih_.sweep_output[i][j].real());
      idata[i + j * dims[0]] = ToDouble(ih_.sweep_output[i][j].imag());
    }
  }
  mat.WriteMatrix("y", 2, dims, MatFile::mxDOUBLE_CLASS,
                  rdata.data(), idata.data());
}

void LuaModelViewer::Draw() {
  Trace trace(__func__);

  // Clear the buffer.
  if (valid_) {
    GL(ClearColor)(1, 1, 1, 0);
  } else {
    GL(ClearColor)(0.75, 0.75, 0.75, 0);
  }
  GL(Clear)(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  DrawModel();

  GL(Disable)(GL_MULTISAMPLE);

  // Draw the markers.
  if (show_markers_) {
    gl::PushShader push_shader(gl::FlatShader());
    vector<Vector2f> p;
    for (int i = 1; i < markers_.size(); i += 2) {
      double x, y;
      GetMarkerPosition(i - 1, &x, &y);
      p.push_back(Vector2f(x, y));
    }
    gl::VertexBuffer<Vector2f> buffer(p);
    buffer.Specify1("vertex", 0, 2, GL_FLOAT);

    double scale = devicePixelRatio();
    for (int j = 0; j <= 1; j++) {
      GL(PointSize)((kMarkerSize - j) * 2 * scale);
      gl::SetUniform("color", j, j, j);
      buffer.Draw(GL_POINTS);
    }
    if (dragging_marker_ >= 0) {
      gl::SetUniform("color", 1, 1, 0);
      buffer.Draw(GL_POINTS, dragging_marker_ / 2, 1);
    }
  }

  GL(UseProgram)(0);

  // Draw all strings.
  Eigen::Matrix4d T = gl::Transform();
  for (const auto &dt : debug_text_) {
    DrawStringM(dt.text.c_str(), dt.x, dt.y, 0, T,
                &debug_string_font, 0,0,0, dt.halign, dt.valign);
  }
  RenderDrawStrings(this);
}

void LuaModelViewer::HandleClick(int x, int y, bool button,
                                 const Eigen::Vector3d &model_pt) {
  if (button) {
    // See if we clicked on a marker.
    if (show_markers_) {
      double sz = kMarkerSize * devicePixelRatio();
      for (int i = 1; i < markers_.size(); i += 2) {
        double mx, my, px, py;
        GetMarkerPosition(i - 1, &mx, &my);
        Eigen::Vector3d model_pt(mx, my, 0);
        if (ModelToPixelCoords(model_pt, &px, &py)) {
          if (x >= px - sz && x <= px + sz &&
              y >= py - sz && y <= py + sz) {
            dragging_marker_ = i - 1;
            update();
          }
        }
      }
    }
  } else {
    dragging_marker_ = -1;
    update();
  }
}

void LuaModelViewer::HandleDrag(int x, int y, const Eigen::Vector3d &model_pt) {
  if (dragging_marker_ >= 0 && dragging_marker_ + 1 < markers_.size()) {
    Eigen::Vector3d model_pt;
    PixelToModelCoords(x, y, &model_pt);
    SetParameter(markers_[dragging_marker_], model_pt[0]);
    SetParameter(markers_[dragging_marker_ + 1], model_pt[1]);
    RerunScript();
  }
}

void LuaModelViewer::GetMarkerPosition(int n, double *x, double *y) {
  CHECK(n >= 0 && n + 1 < markers_.size());
  const Parameter &px = GetParameter(markers_[n]);
  const Parameter &py = GetParameter(markers_[n + 1]);
  *x = px.value;
  *y = py.value;
}

bool LuaModelViewer::SetParameter(const string &parameter_name, double value) {
  ParamMap::iterator it = param_map_.find(parameter_name);
  if (it != param_map_.end()) {
    Parameter &p = it->second;
    p.value = std::max(p.the_min, std::min(p.the_max, value));
    CHECK(p.text_ctrl);
    CHECK(p.slider);
    p.text_ctrl->ReflectParameterValue();
    p.slider->ReflectParameterValue();
    return true;
  } else {
    return false;
  }
}

bool LuaModelViewer::OnInvisibleHandSweep() {
  // Sanity checks, and update the sweep parameter value.
  if (ih_.sweep_index < 0 || ih_.sweep_index >= ih_.sweep_values.size() ||
      ih_.sweep_values.size() != ih_.sweep_output.size() ||
      !SetParameter(ih_.sweep_parameter_name,
                    ih_.sweep_values[ih_.sweep_index])) {
    Error("Sweep interrupted (internal error)");
    return false;
  }
  if (ih_.sweep_over_test_output && !run_test_after_solve_) {
    Error("If sweeping over test output, enable 'Run test() after each solve'");
    return false;
  }

  // Rerun the script, collect the test function output if requested.
  std::vector<JetNum> test_output;
  if (!RerunScript(false, 0, ih_.sweep_over_test_output ? &test_output : 0)) {
    Error("Sweep interrupted");
    return false;
  }

  // If the test function was run and generated output, use it as the swept
  // output. Otherwise compute the sweep output by ComputeSweptOutput().
  if (!test_output.empty()) {
    for (int i = 0; i < test_output.size(); i++) {
      ih_.sweep_output[ih_.sweep_index].push_back(JetComplex(test_output[i]));
    }
  } else {
    if (!ComputeSweptOutput(&ih_.sweep_output[ih_.sweep_index])) {
      Error("Sweep interrupted");
      return false;
    }
  }

  // Display the result.
  repaint();

  // Advance to the next value in a sweep.
  ih_.sweep_index++;
  if (ih_.sweep_index < ih_.sweep_values.size()) {
    return true;
  } else {
    // We're all done so plot the results.
    if (!PlotSweepResults(plot_type_, ih_.sweep_parameter_name,
                          ih_.sweep_values, ih_.sweep_output)) {
      Error("Error during sweep (e.g. nonconstant number of ports or modes)");
    }
    return false;
  }
}

bool LuaModelViewer::OnInvisibleHandOptimize() {
  Trace trace(__func__);
  CHECK(ih_.optimizer);

  // Initialize optimizer if necessary.
  if (!ih_.optimizer->IsInitialized()) {
    vector<InteractiveOptimizer::Parameter>
        start(ih_.opt_parameter_names.size());
    for (int i = 0; i < ih_.opt_parameter_names.size(); i++) {
      const Parameter &p = GetParameter(ih_.opt_parameter_names[i]);
      start[i].starting_value = p.value;
      start[i].min_value = p.the_min;
      start[i].max_value = p.the_max;
      start[i].gradient_step = 0;       // Numerical jacobians disallowed
    }
    ih_.optimizer->Initialize(start);
    if (ih_.optimizer->Parameters().empty()) {
      Error("Optimizer interrupted (could not initialize)");
      return false;
    }
    CHECK(ih_.optimizer->Parameters().size() == ih_.opt_parameter_names.size());
  }

  // Copy optimizer parameters so RerunScript() can pick them up, also also
  // update all parameter controls so the user can see evidence of progress.
  for (int i = 0; i < ih_.opt_parameter_names.size(); i++) {
    if (!SetParameter(ih_.opt_parameter_names[i],
                      ih_.optimizer->Parameters()[i])) {
      Error("Optimizer interrupted (could not set parameter value)");
      return false;
    }
  }

  // Rerun the script with the new parameter values, get the output of the lua
  // optimize() function.
  // @@@ The first time we are likely redundantly evaluating the system as
  // new_parameters probably == the existing parameters. Optimize by detecting
  // this case. Or we could ignore it as the fractional impact is low.
  vector<JetNum> optimize_errors;
  derivative_index_ = 0;
  PrepareForOptimize();
  if (!RerunScript(false, &optimize_errors) || optimize_errors.empty()) {
    Error("Optimizer interrupted (script failed)");
    return false;
  }

  // See if we're all done.
  if (ih_.optimizer_done_) {
    update();
    return false;
  }

  // Compute the Jacobian if necessary.
  //    jacobians[j*num_parameters + i] = d error[j] / d parameter[i]
  vector<double> jacobians;
  if (ih_.optimizer->JacobianRequested()) {
    jacobians.resize(ih_.opt_parameter_names.size() * num_optimize_outputs_);
    for (int i = 0; i < ih_.opt_parameter_names.size(); i++) {
      // Compute the derivatives for this derivative index. The results for
      // i==0 were already computed above.
      if (i > 0) {
        derivative_index_ = i;
        if (!RerunScript(false, &optimize_errors, 0, true) ||
            optimize_errors.empty()) {
          Error("Optimizer interrupted (script failed)");
          return false;
        }
      }
      CHECK(num_optimize_outputs_ == optimize_errors.size());
      for (int j = 0; j < num_optimize_outputs_; j++) {
        jacobians[j*ih_.opt_parameter_names.size() + i] =
          optimize_errors[j].Derivative();
      }
    }
  }
  derivative_index_ = 0;

  // Convert optimize_errors to double.
  vector<double> optimize_errors_d(optimize_errors.size());
  for (int i = 0; i < optimize_errors.size(); i++) {
    optimize_errors_d[i] = ToDouble(optimize_errors[i]);
  }

  // Do one iteration of the optimizer.
  ih_.optimizer_done_ =
        ih_.optimizer->DoOneIteration(optimize_errors_d, jacobians);
  if (ih_.optimizer->Parameters().empty()) {
    //@@@ should we Refresh() on all error exits?
    Error("Optimizer interrupted (iteration failed)");
    return false;
  }
  if (ih_.optimizer_done_) {
    // Optimization is complete. The optimizer does not guarantee that the last
    // model evaluated is the optimal one, so we need to do a final iteration
    // with the optimal parameters.
    return true;
  }

  // Display the result.
  repaint();

  // Schedule the next iteration of the optimization.
  return true;
}
