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

#ifndef __MAIN_WINDOW_H__
#define __MAIN_WINDOW_H__

#include <QMainWindow>
#include <QFileSystemWatcher>
#include <QNetworkAccessManager>

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

  void AsynchronouslySeeIfNewerVersionAvailable();
  void ToggleRunTestAfterSolve();
  void ScriptTestMode();
  void LoadFile(const QString &full_path);
  bool ReloadScript(bool rerun_even_if_same);
  void OnFileChanged(const QString &path);
  void ReloadTimeout();
  void SetCommandLineArguments(int argc, char **argv);

private slots:
  void ResetCurrentModelBackground();
  void on_actionQuit_triggered();
  void on_actionAbout_triggered();
  void on_actionSweep_triggered();
  void on_actionOpen_triggered();
  void on_actionReload_triggered();
  void on_actionAutoRun_triggered();
  void on_actionExportBoundaryDXF_triggered();
  void on_actionExportBoundaryXY_triggered();
  void on_actionExportFieldAsMatlab_triggered();
  void on_actionExportPlotAsMatlab_triggered();
  void on_actionCopyParameters_triggered();
  void on_actionZoomExtents_triggered();
  void on_actionZoomIn_triggered();
  void on_actionZoomOut_triggered();
  void on_actionViewLines_triggered();
  void on_actionViewPorts_triggered();
  void on_actionViewVertices_triggered();
  void on_actionViewVertexDerivatives_triggered();
  void on_actionShowGrid_triggered();
  void on_actionShowMarkers_triggered();
  void on_actionAntialiasing_triggered();
  void on_actionEmitTrace_triggered();
  void on_actionSelectLevenbergMarquardt_triggered();
  void on_actionSelectSubspaceDogleg_triggered();
  void on_actionStartOptimization_triggered();
  void on_actionStopSweepOrOptimization_triggered();
  void on_actionRamaManual_triggered();
  void on_actionRamaWebsite_triggered();
  void on_actionLuaManual_triggered();
  void on_actionCheckForLatestVersion_triggered();
  void on_reload_resets_parameters_stateChanged(int arg1);
  void on_show_field_stateChanged(int arg1);
  void on_animate_stateChanged(int arg1);
  void on_color_map_currentIndexChanged(int index);
  void on_brightness_valueChanged(int value);
  void on_plot_type_currentIndexChanged(int index);
  void on_mesh_type_currentIndexChanged(int index);
  void on_antenna_show_stateChanged(int arg1);
  void on_antenna_scale_max_stateChanged(int arg1);
  void on_mode_number_valueChanged(int arg1);
  void on_display_style_Ez_currentIndexChanged(int index);
  void on_display_style_Exy_currentIndexChanged(int index);
  void on_display_style_ES_currentIndexChanged(int index);
  void on_display_style_TM_currentIndexChanged(int index);
  void on_display_style_TE_currentIndexChanged(int index);
  void on_copy_to_clipboard_clicked();
  void VersionReplyAvailable(QNetworkReply *reply);
  void on_frequency_index_spinner_valueChanged(int arg1);
  void on_frequency_index_slider_valueChanged(int value);
  void on_wideband_pulse_stateChanged(int arg1);
  void on_sparam_plot_type_currentIndexChanged(int index);
  void on_show_sparams_stateChanged(int arg1);
  void on_time_dial_valueChanged(int value);
  void on_actionSet_animation_time_to_0_triggered();
  void on_actionRunTest_triggered();
  void on_actionShowSParameters_triggered();
  void on_actionSwitchToModelAfterEachSolve_triggered();
  void on_actionExportAntennaPatternAsMatlabData_triggered();
  void on_actionIncrease_animation_time_triggered();
  void on_actionDecrease_animation_time_triggered();
  void on_actionSelectRandomSearch_triggered();
  void on_actionSelectNelderMead_triggered();
  void on_action3D_triggered();
  void on_actionInterior_triggered();
  void on_reload_resets_view_stateChanged(int arg1);
  void on_actionSelectRepeatedLevenbergMarquardt_triggered();

private:
  Ui::MainWindow *ui = 0;
  QString script_filename_;
  bool autorun_ = true;
  QFileSystemWatcher watcher_;

  void UpdateDisplayStyle(int index);
  void SetNumFrequencies(int n);
  void SetWidebandControlsEnabledState();
  void UncheckSimulationMethods();
};

#endif
