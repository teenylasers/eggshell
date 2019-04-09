#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

private slots:
  void on_actionRun_triggered();
  void on_actionShow_bounding_box_triggered();

  void on_actionSingle_step_triggered();

private:
  Ui::MainWindow *ui;
};

#endif // MAIN_WINDOW_H
