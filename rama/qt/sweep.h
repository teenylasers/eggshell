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
