#ifndef __ABOUT_H__
#define __ABOUT_H__

#include <QDialog>

namespace Ui {
  class About;
}

class About : public QDialog
{
  Q_OBJECT

public:
  explicit About(QWidget *parent = 0);
  ~About();

private:
  Ui::About *ui;
};

#endif
