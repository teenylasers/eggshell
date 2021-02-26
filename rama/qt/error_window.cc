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

#include "error_window.h"
#include "ui_error_window.h"
#include "../../toolkit/error.h"
#include <QClipboard>
#include <QTimer>

ErrorWindow::ErrorWindow(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ErrorWindow)
{
  // Can only have one instance of this window.
  static bool created = false;
  CHECK(!created);
  created = true;

  // Setup UI and icons.
  ui->setupUi(this);
  error_icon_ = qApp->style()->standardIcon(QStyle::SP_MessageBoxCritical);
  warning_icon_ = qApp->style()->standardIcon(QStyle::SP_MessageBoxWarning);

  // Set up a signal/slot connection that works across threads.
  QObject::connect(this, SIGNAL(AddLineSignal(QString, int)),
                   this, SLOT(AddLine(QString, int)));
}

ErrorWindow::~ErrorWindow() {
  delete ui;
}

void ErrorWindow::on_ok_button_clicked() {
  // We hide the window but don't destroy it, as we will need it later
  // for other errors.
  hide();
  ui->error_list->clear();
}

void ErrorWindow::on_copy_to_clipboard_button_clicked() {
  auto *list = ui->error_list;
  int n = list->count();
  QString s;
  for (int i = 0; i < n; i++) {
    s += list->item(i)->text() + "\n";
  }
  QGuiApplication::clipboard()->setText(s);
}

void ErrorWindow::VAddLine(const char *msg, va_list ap, int type) {
  // This function is potentially called from a thread that is not the main
  // GUI thread, so we use the signal/slot mechanism to pass a message to
  // the error window in the GUI thread.
  QString s = QString::vasprintf(msg, ap);
  AddLineSignal(s, type);
}

void ErrorWindow::AddLine(QString s, int type) {
  auto item = new QListWidgetItem(s, 0);
  if (type == Type::ERROR) {
    item->setForeground(QColor(255, 0, 0));
    item->setIcon(error_icon_);
  } else if (type == Type::WARNING) {
    item->setForeground(QColor(255, 128, 0));
    item->setIcon(warning_icon_);
  }
  auto *list = ui->error_list;
  list->insertItem(list->count(), item);
  list->scrollToItem(item);

  // Show and raise the error window. If errors are generated in the
  // constructors of other windows before they are shown then the error window
  // could get obscured, so to prevent this we show/raise the error window
  // after a short delay.
  QTimer::singleShot(0, this, &ErrorWindow::ShowAndRaise);
}

void ErrorWindow::ShowAndRaise() {
  show();
  raise();
}
