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

#include "sweep.h"
#include "ui_sweep.h"
#include "../../toolkit/lua_model_viewer_qt.h"

#include <QMessageBox>
#include <QFileDialog>

Sweep::Sweep(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::Sweep)
{
  ui->setupUi(this);
  viewer_ = 0;
  integer_ = 0;
  start_value_ = end_value_ = min_value_ = max_value_ = 0;
  num_steps_ = 0;
}

void Sweep::SetupForModel(LuaModelViewer *viewer) {
  viewer_ = viewer;
  std::vector<std::string> names;
  viewer->GetParamControlNames(&names);
  for (int i = 0; i < names.size(); i++) {
    ui->parameters->addItem(names[i].c_str());
  }
  ui->parameters->setCurrentIndex(0);
  if (!names.empty()) {
    SetParameter(names[0]);
  }
}

void Sweep::SetParameter(const std::string &name) {
  parameter_ = name;
  const Parameter &param = viewer_->GetParameter(name);
  ui->start_value->setText(QString::asprintf("%.10g", param.the_min));
  ui->end_value->setText(QString::asprintf("%.10g", param.the_max));
  ui->num_steps->setEnabled(!param.integer);
  ui->label_num_steps->setEnabled(!param.integer);
  integer_ = param.integer;
  min_value_ = param.the_min;
  max_value_ = param.the_max;
}

Sweep::~Sweep() {
  delete ui;
}

QString Sweep::GetParameterName() {
  return parameter_.c_str();
}

double Sweep::GetStartValue() {
  return start_value_;
}

double Sweep::GetEndValue() {
  return end_value_;
}

int Sweep::GetNumSteps() {
  return num_steps_;
}

bool Sweep::GetTestOutput() {
  return ui->test_output->isChecked();
}

QString Sweep::GetImageFileName() {
  return image_file_name_;
}

bool Sweep::Check() {
  bool ok;
  start_value_ = ui->start_value->text().toDouble(&ok);
  if (!ok) {
    QMessageBox::warning(this, "Error", "Bad start value");
    return false;
  }
  end_value_ = ui->end_value->text().toDouble(&ok);
  if (!ok) {
    QMessageBox::warning(this, "Error", "Bad end value");
    return false;
  }
  num_steps_ = ui->num_steps->text().toInt(&ok);
  if (!ok || num_steps_ <= 0) {
    QMessageBox::warning(this, "Error", "Bad number of steps");
    return false;
  }
  if (start_value_ >= end_value_) {
    QMessageBox::warning(this, "Error",
                         "Start value must be less than end value");
    return false;
  }
  // If the start and end value are just slightly beyond the min and max it
  // might be because of truncation errors converting text to double. Handle
  // this gracefully.
  if (start_value_ < min_value_ && start_value_ > min_value_ * (1 - 1e-9)) {
    start_value_ = min_value_;
  }
  if (end_value_ > max_value_ && end_value_ < max_value_ * (1 + 1e-9)) {
    end_value_ = max_value_;
  }
  if (start_value_ < min_value_ || end_value_ > max_value_) {
    QMessageBox::warning(this, "Error",
        QString::asprintf("Start and end must be in the range %g to %g",
                          min_value_, max_value_));
    return false;
  }
  if (integer_) {
    if (start_value_ != int(start_value_) || end_value_ != int(end_value_)) {
      QMessageBox::warning(this, "Error",
                           "Start and end values must be integers");
      return false;
    }
  }
  image_file_name_ = ui->image_filename->text();
  if (!image_file_name_.isEmpty()) {
    int index = image_file_name_.indexOf('#');
    if (index == -1 || image_file_name_.indexOf('#', index + 1) >= 0) {
      QMessageBox::warning(this, "Error",
          "Image filename must contain one '#' to represent the image number");
      return false;
    }
    if (image_file_name_.indexOf('%') >= 0) {
      QMessageBox::warning(this, "Error",
          "Image filename can not contain the character '%'");
      return false;
    }
    if (!image_file_name_.endsWith(".png",Qt::CaseInsensitive)) {
      QMessageBox::warning(this, "Error",
          "Image filename must end with .png");
      return false;
    }
    image_file_name_ = image_file_name_.left(index) + "%06d" +
                       image_file_name_.mid(index + 1);
  }
  return true;
}

void Sweep::on_cancel_button_clicked() {
  reject();
}

void Sweep::on_ok_button_clicked() {
  if (Check()) {
    accept();
  }
}

void Sweep::on_parameters_currentIndexChanged(const QString &arg1) {
  SetParameter(arg1.toUtf8().data());
}

void Sweep::on_browse_clicked() {
  QString filename = QFileDialog::getSaveFileName(this,
      "Select an image file to save", QString(), "Image files (*.png)");
  if (!filename.isEmpty()) {
    if (filename.endsWith(".png")) {
      filename = filename.mid(0, filename.length()-4) + "#" + ".png";
    }
    ui->image_filename->setText(filename);
  }
}
