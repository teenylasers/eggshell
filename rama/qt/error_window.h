#ifndef ERROR_WINDOW_H
#define ERROR_WINDOW_H

#include <QDialog>
#include <QIcon>

namespace Ui {
  class ErrorWindow;
}

class ErrorWindow : public QDialog {
  Q_OBJECT

public:
  explicit ErrorWindow(QWidget *parent = 0);
  ~ErrorWindow();

  enum Type { ERROR, WARNING, MESSAGE };

  // This can be called from any thread to add a message line.
  void VAddLine(const char *msg, va_list ap, int type);

signals:
  // This can be called from any thread to add a message line.
  void AddLineSignal(QString s, int type);

private slots:
  // This must be called from the main GUI thread to add a message line.
  void AddLine(QString s, int type);

  void on_ok_button_clicked();

  void on_copy_to_clipboard_button_clicked();

private:
  Ui::ErrorWindow *ui;
  QIcon error_icon_, warning_icon_;
};

#endif // ERROR_WINDOW_H
