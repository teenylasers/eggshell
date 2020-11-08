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

#include <QDesktopServices>
#include <QUrl>
#include <QFileDialog>
#include <QFile>
#include <QByteArray>
#include <QTimer>
#include <QClipboard>
#include <QNetworkReply>
#include <QMessageBox>
#include <QWindow>
#include <QScreen>

#include "main_window.h"
#include "ui_main_window.h"
#include "about.h"
#include "sweep.h"
#include "nelder_mead.h"
#include "../version.h"
#include "../../toolkit/error.h"

#if defined(__APPLE__)
#define __APP_LATEST_VERSION_PATH__ __APP_LATEST_VERSION_PATH_MAC__
#elif defined(__linux__)
#define __APP_LATEST_VERSION_PATH__ __APP_LATEST_VERSION_PATH_LINUX__
#else
#define __APP_LATEST_VERSION_PATH__ __APP_LATEST_VERSION_PATH_WIN__
#endif

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent), ui(new Ui::MainWindow), watcher_(this)
{
  ui->setupUi(this);

  // Set some menu and control defaults.
  ui->actionAntialiasing->setChecked(true);

  // Configure docked windows.
  setTabPosition(Qt::AllDockWidgetAreas, QTabWidget::North);
  tabifyDockWidget(ui->dock_model, ui->dock_plot);
  tabifyDockWidget(ui->dock_model, ui->dock_sparams);
  tabifyDockWidget(ui->dock_model, ui->dock_antenna);
  tabifyDockWidget(ui->dock_model, ui->dock_script_messages);
  ui->dock_model->raise();

  // Default shows and hidden controls.
  ui->display_style_Exy->hide();
  ui->display_style_ES->hide();
  ui->display_style_TE->hide();
  ui->display_style_TM->hide();
  ui->mode_label->hide();
  ui->mode_number->hide();

  // Position the window so it takes up the entire screen minus a margin.
  QScreen *s = QGuiApplication::screenAt(mapToGlobal({width()/2, height()/2}));
  if (s) {
    QRect rect = s->availableGeometry();
    rect = rect.marginsRemoved(QMargins(rect.width()/8, rect.height()/8,
                                        rect.width()/8, rect.height()/8));
    setGeometry(rect);
    resizeDocks({ui->dock_model}, {rect.width() /3*2}, Qt::Horizontal);
  }

  // Connect the model viewer to other controls.
  ui->model->Connect(ui->script_messages, ui->parameter_pane, ui->plot,
                     ui->dock_model, ui->dock_plot, ui->dock_script_messages);
  ui->model->Connect2(ui->antenna_plot, ui->sparam_plot, ui->time_dial);

  // Set up the file watcher.
  QObject::connect(&watcher_, &QFileSystemWatcher::fileChanged,
                   this, &MainWindow::OnFileChanged);

  // Asynchronously retrieve the latest application version number and
  // display a message if there is a newer version available. Don't do
  // this is script test mode though, because we are going to be exiting
  // very soon and processing the reply might cause problems.
  if (!ui->model->IsScriptTestMode()) {
    QNetworkAccessManager *manager = new QNetworkAccessManager(this);
    connect(manager, SIGNAL(finished(QNetworkReply*)),
            this, SLOT(VersionReplyAvailable(QNetworkReply*)));
    QNetworkRequest request(QUrl(__APP_URL__ __APP_LATEST_VERSION_PATH__));
    // A user agent is necessary otherwise the security module on an apache
    // server denies the request.
    request.setRawHeader("User-Agent", "Rama");
    manager->get(request);
  }
}

MainWindow::~MainWindow() {
  delete ui;
}

void MainWindow::ToggleRunTestAfterSolve() {
  ui->model->ToggleRunTestAfterSolve();
}

void MainWindow::ScriptTestMode() {
  ui->model->ScriptTestMode();
}

void MainWindow::LoadFile(const QString &full_path) {
  // Remove any current file watchers.
  {
    auto list = watcher_.files();
    if (!list.empty()) {
      watcher_.removePaths(list);   // Generates a warning if list empty
    }
  }

  // Change the directory to where the file is, so that if it does 'dofile' or
  // similar it can refer to relative paths. This also makes the default save
  // location that directory.
  QFileInfo info(full_path);
  QDir::setCurrent(info.path());
  script_filename_ = info.fileName();

  // Reload the file and watch for further changes to it.
  if (ReloadScript(true)) {
    // Watch this file for changes.
    if (!watcher_.addPath(script_filename_)) {
      Error("Can not watch %s for changes", script_filename_.toUtf8().data());
    }
  }
}

bool MainWindow::ReloadScript(bool rerun_even_if_same) {
  if (script_filename_.isEmpty()) {
    Error("No script has been opened yet");
    return false;
  }

  // Open and read the file.
  QFile f(script_filename_);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
    Error("Can not open %s", script_filename_.toUtf8().data());
    return false;
  }
  QByteArray buffer = f.readAll();
  if (buffer.isEmpty()) {
    // Either the file was empty or an error occurred.
    // @@@ Distinguish between these cases.
    // @@@ If the file is empty, it's possible we loaded it just as the
    //     editor was saving it.
    Error("Can not read %s", script_filename_.toUtf8().data());
    return false;
  }

  // Run the script.
  QString script = "util.script_filename = '" + script_filename_ + "'\n" +
                   buffer;
  bool script_ran = ui->model->RunScript(script.toUtf8().data(),
                                         rerun_even_if_same);

  // Highlight the "current model" label, revert it to normal color after 1s.
  if (script_ran) {
    QFileInfo fi(script_filename_);
    ui->current_model->setText(fi.baseName());
    ui->current_model->setStyleSheet("QLabel { background-color : yellow; }");
    QTimer::singleShot(1000, this, &MainWindow::ResetCurrentModelBackground);
  }

  // Show the correct controls based on the cavity type.
  ui->display_style_Ez->setVisible(ui->model->IsEzCavity());
  ui->display_style_Exy->setVisible(ui->model->IsExyCavity());
  ui->display_style_ES->setVisible(ui->model->IsESCavity());
  ui->display_style_TE->setVisible(ui->model->IsTEMode());
  ui->display_style_TM->setVisible(ui->model->IsTMMode());
  ui->animate->setVisible(ui->model->NumWaveguideModes() == 0);
  ui->mode_number->setVisible(ui->model->NumWaveguideModes());
  ui->mode_label->setVisible(ui->model->NumWaveguideModes());
  ui->mode_number->setMinimum(0);
  ui->mode_number->setMaximum(std::max(0, ui->model->NumWaveguideModes() - 1));
  ui->model->SetWaveguideModeDisplayed(ui->mode_number->value());
  SetNumFrequencies(ui->model->GetNumFrequencies());
  SetWidebandControlsEnabledState();
  ui->model->SetFrequencyIndex(ui->frequency_index_slider->value() - 1);

  return true;
}

void MainWindow::OnFileChanged(const QString &path) {
  // We might get this event more than once for a file modification (it depends
  // on how the editor saves the file). To try and avoid reading the file in
  // some in-between state, we trigger the actual file read 100ms from now.
  QTimer::singleShot(100, this, &MainWindow::ReloadTimeout);
}

void MainWindow::ReloadTimeout() {
  if (autorun_) {
    ReloadScript(false);
  }
}

void MainWindow::SetCommandLineArguments(int argc, char **argv) {
  ui->model->SetCommandLineArguments(argc, argv);
}

void MainWindow::ResetCurrentModelBackground() {
  ui->current_model->setStyleSheet("QLabel { }");
}

void MainWindow::on_actionQuit_triggered() {
  QApplication::quit();
}

void MainWindow::on_actionAbout_triggered() {
  auto about = new About(this);
  about->show();
}

void MainWindow::on_actionSweep_triggered() {
  Sweep sweep(this);
  sweep.SetupForModel(ui->model);
  if (sweep.exec() == QDialog::Accepted) {
    ui->model->Sweep(sweep.GetParameterName().toUtf8().data(),
                     sweep.GetStartValue(),
                     sweep.GetEndValue(),
                     sweep.GetNumSteps(),
                     sweep.GetTestOutput(),
                     sweep.GetImageFileName().toUtf8().data());
  }
}

void MainWindow::on_actionOpen_triggered() {
  QString filename = QFileDialog::getOpenFileName(this, "Open model",
      QString(), "Lua files (*.lua)", Q_NULLPTR, QFileDialog::ReadOnly);
  if (!filename.isEmpty()) {
    LoadFile(filename);
  }
}

void MainWindow::on_actionReload_triggered() {
  ReloadScript(true);
}

void MainWindow::on_actionAutoRun_triggered() {
  autorun_ = !autorun_;
  if (autorun_) {
    ReloadScript(false);
  }
}

void MainWindow::on_actionExportBoundaryDXF_triggered() {
  QString filename = QFileDialog::getSaveFileName(this,
      "Select a DXF file to save", QString(), "DXF files (*.dxf)");
  if (!filename.isEmpty()) {
    ui->model->ExportBoundaryDXF(filename.toUtf8().data());
  }
}

void MainWindow::on_actionExportBoundaryXY_triggered() {
  QString filename = QFileDialog::getSaveFileName(this,
      "Select a file to save", QString(), "*.*");
  if (!filename.isEmpty()) {
    ui->model->ExportBoundaryXY(filename.toUtf8().data());
  }
}

void MainWindow::on_actionExportFieldAsMatlab_triggered() {
  QString filename = QFileDialog::getSaveFileName(this,
      "Select a matlab file to save", QString(), "*.mat");
  if (!filename.isEmpty()) {
    ui->model->ExportFieldMatlab(filename.toUtf8().data());
  }
}

void MainWindow::on_actionExportPlotAsMatlab_triggered() {
  QString filename = QFileDialog::getSaveFileName(this,
      "Select a matlab file to save", QString(), "*.mat");
  if (!filename.isEmpty()) {
    ui->model->ExportPlotMatlab(filename.toUtf8().data());
  }
}

void MainWindow::on_actionCopyParameters_triggered() {
  ui->model->CopyParametersToClipboard();
}

void MainWindow::on_actionZoomExtents_triggered() {
  ui->model->ZoomExtents();
}

void MainWindow::on_actionZoomIn_triggered() {
  ui->model->Zoom(1.0/sqrt(2));
}

void MainWindow::on_actionZoomOut_triggered() {
  ui->model->Zoom(sqrt(2));
}

void MainWindow::on_actionViewLines_triggered() {
  ui->model->ToggleShowBoundaryLines();
}

void MainWindow::on_actionViewPorts_triggered() {
  ui->model->ToggleShowBoundaryPorts();
}

void MainWindow::on_actionViewVertices_triggered() {
  ui->model->ToggleShowBoundaryVertices();
}

void MainWindow::on_actionViewVertexDerivatives_triggered() {
  ui->model->ToggleShowBoundaryDerivatives();
}

void MainWindow::on_actionShowGrid_triggered() {
  ui->model->ToggleGrid();
}

void MainWindow::on_actionShowMarkers_triggered() {
  ui->model->ToggleShowMarkers();
}

void MainWindow::on_actionAntialiasing_triggered() {
  ui->model->ToggleAntialiasing();
}

void MainWindow::on_actionEmitTrace_triggered() {
  ui->model->ToggleEmitTraceReport();
}

void MainWindow::on_actionSelectLevenbergMarquardt_triggered() {
  ui->model->SetOptimizer(OptimizerType::LEVENBERG_MARQUARDT);
  UncheckSimulationMethods();
  ui->actionSelectLevenbergMarquardt->setChecked(true);
}

void MainWindow::on_actionSelectSubspaceDogleg_triggered() {
  ui->model->SetOptimizer(OptimizerType::SUBSPACE_DOGLEG);
  UncheckSimulationMethods();
  ui->actionSelectSubspaceDogleg->setChecked(true);
}

void MainWindow::on_actionSelectNelderMead_triggered() {
  NelderMead neldermead(this);
  if (neldermead.exec() == QDialog::Accepted) {
    UncheckSimulationMethods();
    ui->actionSelectNelderMead->setChecked(true);
    ui->model->SetOptimizer(OptimizerType::NELDER_MEAD);
    ui->model->SetNelderMeadParameters(neldermead.GetParameters());
  } else {
    ui->actionSelectNelderMead->setChecked(false);
  }
}

void MainWindow::on_actionSelectRandomSearch_triggered() {
  ui->model->SetOptimizer(OptimizerType::RANDOM_SEARCH);
  UncheckSimulationMethods();
  ui->actionSelectRandomSearch->setChecked(true);
}

void MainWindow::UncheckSimulationMethods() {
  ui->actionSelectLevenbergMarquardt->setChecked(false);
  ui->actionSelectSubspaceDogleg->setChecked(false);
  ui->actionSelectNelderMead->setChecked(false);
  ui->actionSelectRandomSearch->setChecked(false);
}

void MainWindow::on_actionStartOptimization_triggered() {
  ui->model->Optimize();
}

void MainWindow::on_actionStopSweepOrOptimization_triggered() {
  ui->model->StopSweepOrOptimize();
}

void MainWindow::on_actionRamaManual_triggered() {
  // Open the manual from the installed location.
  QDir dir = QDir(QCoreApplication::applicationDirPath());
  #ifdef __APPLE__
    // Navigate within the bundle.
    dir.cd("../../Contents/Resources");
  #endif
  QString path = dir.absoluteFilePath("rama.html");
  #ifdef __linux__
    QDesktopServices::openUrl(QUrl("file://" + path));
  #else
    QDesktopServices::openUrl(QUrl("file:///" + path));
  #endif
}

void MainWindow::on_actionRamaWebsite_triggered() {
  QString link = __APP_URL__;
  QDesktopServices::openUrl(QUrl(link));
}

void MainWindow::on_actionLuaManual_triggered() {
  QString link = "http://www.lua.org/manual/5.4/";
  QDesktopServices::openUrl(QUrl(link));
}

void MainWindow::on_actionCheckForLatestVersion_triggered() {
  QMessageBox box;
  box.setText("Do you want to install the latest software?");
  box.setInformativeText("Clicking 'Yes' will quit the application and start "
      "downloading the installer for the latest version of " __APP_NAME__);
  box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  box.setDefaultButton(QMessageBox::No);
  if (box.exec() == QMessageBox::Yes) {
    #if __APPLE__
      QDesktopServices::openUrl(
            QUrl(__APP_URL__ __APP_LATEST_MAC_DOWNLOAD_PATH__));
    #else
      QDesktopServices::openUrl(
            QUrl(__APP_URL__ __APP_LATEST_WINDOWS_DOWNLOAD_PATH__));
    #endif
    exit(0);
  }
}

void MainWindow::on_reload_resets_parameters_stateChanged(int arg1) {
  ui->model->SetIfRunScriptResetsParameters(arg1);
}

void MainWindow::on_show_field_stateChanged(int arg1) {
  // Turning off the field view automatically turns off the animation.
  if (!arg1) {
    if (ui->animate->isChecked()) {
      ui->animate->setChecked(false);
      ui->model->Animate(0);
    }
  }
  ui->model->ViewField(arg1);
}

void MainWindow::on_animate_stateChanged(int arg1) {
  // Turning on the animation automatically turns on the field view.
  if (arg1) {
    if (!ui->show_field->isChecked()) {
      ui->show_field->setChecked(true);
      ui->model->ViewField(1);
    }
  }
  ui->model->Animate(arg1);
}

void MainWindow::on_color_map_currentIndexChanged(int index) {
  ui->model->SetDisplayColorScheme(index);
}

void MainWindow::on_brightness_valueChanged(int value) {
  ui->model->SetBrightness(value);
}

void MainWindow::on_plot_type_currentIndexChanged(int index) {
  ui->model->SelectPlot(index);
}

void MainWindow::on_mesh_type_currentIndexChanged(int index) {
  ui->model->ViewMesh(index);
}

void MainWindow::on_antenna_show_stateChanged(int arg1) {
  ui->model->ToggleAntennaShow();
}

void MainWindow::on_antenna_scale_max_stateChanged(int arg1) {
  ui->model->ToggleAntennaScaleMax();
}

void MainWindow::on_mode_number_valueChanged(int arg1) {
  ui->model->SetWaveguideModeDisplayed(arg1);
}

void MainWindow::on_display_style_Ez_currentIndexChanged(int index) {
  UpdateDisplayStyle(index);
}

void MainWindow::on_display_style_Exy_currentIndexChanged(int index) {
  UpdateDisplayStyle(index);
}

void MainWindow::on_display_style_ES_currentIndexChanged(int index) {
  UpdateDisplayStyle(index);
}

void MainWindow::on_display_style_TM_currentIndexChanged(int index) {
  UpdateDisplayStyle(index);
}

void MainWindow::on_display_style_TE_currentIndexChanged(int index) {
  UpdateDisplayStyle(index);
}

void MainWindow::UpdateDisplayStyle(int index) {
  // All display style choice boxes are synchronized with each other and with
  // the display style state in the model.
  // @@@ Do we need to sync the TM and TE display styles too? they only have 5 entries.
  // @@@ Does TM/TE display style sync when a mode model is loaded?
  ui->display_style_Exy->setCurrentIndex(index);
  ui->display_style_ES->setCurrentIndex(index);
  ui->display_style_Ez->setCurrentIndex(index);
  ui->model->SetDisplayStyle(index);
}

void MainWindow::on_copy_to_clipboard_clicked() {
  // Copy script messages to clipboard.
  auto *list = ui->script_messages;
  int n = list->count();
  QString s;
  for (int i = 0; i < n; i++) {
    s += list->item(i)->text() + "\n";
  }
  QGuiApplication::clipboard()->setText(s);
}

void MainWindow::VersionReplyAvailable(QNetworkReply *reply) {
  QByteArray b = reply->readAll();
  int major, minor;
  if (sscanf(b.data(), "%d.%d", &major, &minor) == 2) {
    int latest = (major << 16) | minor;
    int current = (__APP_VERSION_MAJOR__ << 16) | __APP_VERSION_MINOR__;
    if (latest > current) {
      Message("A newer version of " __APP_NAME__ " is available (current "
              "version is " __APP_VERSION__ ", latest version is %d.%d). "
              "Use 'Help/Download latest' to install the latest version.",
              major, minor);
    }
  }
  reply->deleteLater();
}

void MainWindow::on_frequency_index_spinner_valueChanged(int arg1) {
  ui->frequency_index_slider->setValue(arg1);
  ui->model->SetFrequencyIndex(arg1 - 1);
}

void MainWindow::on_frequency_index_slider_valueChanged(int value) {
  ui->frequency_index_spinner->setValue(value);
  ui->model->SetFrequencyIndex(value - 1);
}

void MainWindow::on_wideband_pulse_stateChanged(int arg1) {
  ui->model->ToggleWidebandPulse();
  SetWidebandControlsEnabledState();
}

void MainWindow::SetNumFrequencies(int n) {
  // If n==1 then the slider will be disabled, but setting min=max=1 will cause
  // the slider to disappear for some reason. Therefore if n==1 we set max to 2
  // and value to 1 ensuring that the slider is visible and looks innocuous.
  ui->frequency_index_slider->setMinimum(1);
  ui->frequency_index_slider->setMaximum(std::max(n, 2));
  ui->frequency_index_spinner->setMinimum(1);
  ui->frequency_index_spinner->setMaximum(n);
}

void MainWindow::SetWidebandControlsEnabledState() {
  int n = ui->model->GetNumFrequencies();
  bool wb = ui->model->GetWidebandPulse();
  ui->wideband_pulse->setEnabled(n > 1);
  ui->frequency_index_slider->setEnabled(!wb && n > 1);
  ui->frequency_index_spinner->setEnabled(!wb && n > 1);
  ui->frequency_label->setEnabled(!wb && n > 1);
}

void MainWindow::on_sparam_plot_type_currentIndexChanged(int index) {
  ui->model->SelectSParamPlot(index);
}

void MainWindow::on_show_sparams_stateChanged(int arg1) {
  ui->model->ToggleShowSParamsGraph();
}

void MainWindow::on_time_dial_valueChanged(int value) {
  ui->model->TimeDialChanged(value);
}

void MainWindow::on_actionSet_animation_time_to_0_triggered() {
  ui->time_dial->setValue(0);
  ui->model->TimeDialToZero();
}

void MainWindow::on_actionRunTest_triggered() {
  ui->model->ToggleRunTestAfterSolve();
}

void MainWindow::on_actionShowSParameters_triggered() {
  ui->model->ToggleShowSParameters();
}

void MainWindow::on_actionSwitchToModelAfterEachSolve_triggered() {
  ui->model->ToggleSwitchToModelAfterEachSolve();
}

void MainWindow::on_actionExportAntennaPatternAsMatlabData_triggered() {
  QString filename = QFileDialog::getSaveFileName(this,
      "Select a matlab file to save", QString(), "Matlab files (*.mat)");
  if (!filename.isEmpty()) {
    ui->model->ExportAntennaPatternMatlab(filename.toUtf8().data());
  }
}

void MainWindow::on_actionIncrease_animation_time_triggered() {
  ui->time_dial->setValue(ui->time_dial->value() + 1);
  ui->model->TimeDialChanged(ui->time_dial->value());
}

void MainWindow::on_actionDecrease_animation_time_triggered() {
  ui->time_dial->setValue(ui->time_dial->value() - 1);
  ui->model->TimeDialChanged(ui->time_dial->value());
}

void MainWindow::on_action3D_triggered() {
  ui->model->Toggle3D();
  // Selecting a color map doesn't make sense in 3D mode, at least for now.
  ui->color_map->setEnabled(!ui->color_map->isEnabled());
}

void MainWindow::on_actionInterior_triggered() {
  ui->model->ToggleShowBoundaryInterior();
}
