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

#ifndef __NELDER_MEAD_H__
#define __NELDER_MEAD_H__

#include <QDialog>
#include "../../toolkit/optimizer.h"

namespace Ui {
  class NelderMead;
}

class NelderMead : public QDialog {
  Q_OBJECT

public:
  explicit NelderMead(QWidget *parent = nullptr);
  ~NelderMead();

  NelderMeadOptimizer::Settings GetParameters() const {
    return s_;
  }

private slots:
  void on_ok_button_clicked();
  void on_cancel_button_clicked();

private:
  Ui::NelderMead *ui = 0;
  static NelderMeadOptimizer::Settings s_;

  bool Check();
};

#endif
