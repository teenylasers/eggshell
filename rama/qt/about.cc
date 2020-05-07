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

#include "about.h"
#include "ui_about.h"
#include "../version.h"

extern char license_text[];

About::About(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::About)
{
  ui->setupUi(this);
  ui->label_version->setText("Version: " __APP_VERSION__);
  ui->label_build_date->setText("Build date: " __DATE__ ", " __TIME__);
  ui->license->setText(license_text);
}

About::~About() {
  delete ui;
}
