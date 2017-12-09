#include "sweep.h"
#include "ui_sweep.h"
#include "../../toolkit/lua_model_viewer_qt.h"

#include <QMessageBox>

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
