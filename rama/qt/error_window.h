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

  void ShowAndRaise();
};

#endif // ERROR_WINDOW_H
