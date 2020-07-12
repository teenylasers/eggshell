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

#ifndef SWEEP_H
#define SWEEP_H

#include <QDialog>

class LuaModelViewer;
namespace Ui {
  class Sweep;
}

class Sweep : public QDialog {
  Q_OBJECT

public:
  explicit Sweep(QWidget *parent = 0);
  ~Sweep();

  void SetupForModel(LuaModelViewer *viewer);
  void SetParameter(const std::string &name);
  QString GetParameterName();
  double GetStartValue();
  double GetEndValue();
  int GetNumSteps();
  bool GetTestOutput();
  QString GetImageFileName();

private slots:
  void on_cancel_button_clicked();
  void on_ok_button_clicked();
  void on_parameters_currentIndexChanged(const QString &arg1);
  void on_browse_clicked();

private:
  Ui::Sweep *ui;
  LuaModelViewer *viewer_;
  std::string parameter_;
  bool integer_;
  double start_value_, end_value_, min_value_, max_value_;
  int num_steps_;
  QString image_file_name_;

  bool Check();
};

#endif // SWEEP_H
