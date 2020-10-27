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

#include <QMessageBox>

#include "nelder_mead.h"
#include "ui_nelder_mead.h"

NelderMeadOptimizer::Settings NelderMead::s_;

NelderMead::NelderMead(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::NelderMead)
{
  ui->setupUi(this);
  resize(0, 0);

  ui->edit_time
    ->setText(QString::asprintf("%.10g", s_.annealing_time));
  ui->edit_initial_fraction
    ->setText(QString::asprintf("%.10g", s_.initial_fraction));
  ui->edit_initial_temperature
    ->setText(QString::asprintf("%.10g", s_.initial_temperature));
  ui->edit_final_temperature
    ->setText(QString::asprintf("%.10g", s_.final_temperature));
}

NelderMead::~NelderMead() {
  delete ui;
}

void NelderMead::on_ok_button_clicked() {
  if (Check()) {
    accept();
  }
}

void NelderMead::on_cancel_button_clicked() {
  reject();
}

bool NelderMead::Check() {
  bool ok;

  s_.annealing_time = ui->edit_time->text().toDouble(&ok);
  if (!ok || s_.annealing_time < 0) {
    QMessageBox::warning(this, "Error",
                         "Bad annealing time value (must be a number >= 0)");
    return false;
  }

  s_.initial_fraction = ui->edit_initial_fraction->text().toDouble(&ok);
  if (!ok || s_.initial_fraction < 0 || s_.initial_fraction > 1) {
    QMessageBox::warning(this, "Error", "Bad initial fraction value "
                         "(must be a number in the range 0..1)");
    return false;
  }

  s_.initial_temperature = ui->edit_initial_temperature->text().toDouble(&ok);
  if (!ok || s_.initial_temperature < 0) {
    QMessageBox::warning(this, "Error",
                       "Bad initial temperature value (must be a number >= 0)");
    return false;
  }

  s_.final_temperature = ui->edit_final_temperature->text().toDouble(&ok);
  if (!ok || s_.final_temperature <= 0 || s_.final_temperature >= 1) {
    QMessageBox::warning(this, "Error", "Bad final temperature value "
                         "(must be a number >0 and <1)");
    return false;
  }

  return true;
}
