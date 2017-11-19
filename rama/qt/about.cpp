#include "about.h"
#include "ui_about.h"
#include "../version.h"

About::About(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::About)
{
  ui->setupUi(this);
  ui->label_version->setText("Version: " __APP_VERSION__);
  ui->label_build_date->setText("Build date: " __DATE__ ", " __TIME__);
}

About::~About() {
  delete ui;
}
