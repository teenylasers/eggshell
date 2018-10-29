
#include "main_window.h"
#include "ui_main_window.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  ui->eggshell_view->Link(ui->status_bar);
}

MainWindow::~MainWindow() {
  delete ui;
}

void MainWindow::on_actionRun_triggered() {
  ui->eggshell_view->ToggleRunning();
}

void MainWindow::on_actionShow_bounding_box_triggered() {
  ui->eggshell_view->ToggleShowBoundingBox();
}
