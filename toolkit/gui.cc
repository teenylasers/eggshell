// Copyright (C) 2014-2020 Russell Smith.
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

#include "gui.h"
#include "error.h"

#include <QPainter>
#include <QMouseEvent>
#include <QRubberBand>
#include <QImage>

namespace GUI {

//***************************************************************************
// Window.

Window::~Window() {
}

void Window::MouseEvent(double x, double y, int button_state) {
}

void Window::HandleKeyPress(int keycode) {
}

//***************************************************************************
// QtWindow.

QtWindow::QtWindow(QWidget *parent) : QWidget(parent) {
  setMouseTracking(true);       // So MouseEvent() gets events when buttons up
  rubber_band_ = new QRubberBand(QRubberBand::Rectangle, this);
}

QtWindow::~QtWindow() {
  // rubber_band_ is deleted automatically, it is a child of this window.
  for (auto it : fonts_) {
    delete it.second;
  }
  for (auto it : metrics_) {
    delete it.second;
  }
  delete painter_;
  delete rubber_band_;
}

double QtWindow::Width() const {
  return QWidget::width();
}

double QtWindow::Height() const {
  return QWidget::height();
}

void QtWindow::Refresh() {
  QWidget::update();
}

void QtWindow::SetColor(uint32_t rrggbb) {
  color_.setRgb((rrggbb >> 16) & 0xff, (rrggbb >> 8) & 0xff, rrggbb & 0xff);
  SetPen();
}

void QtWindow::SetLineWidth(double w) {
  line_width_ = w;
  SetPen();
}

void QtWindow::SetLineStyle(LineStyle style) {
  style_ = style;
  SetPen();
}

void QtWindow::DrawLine(double x1, double y1, double x2, double y2) {
  CHECK(painter_);
  painter_->setPen(pen_);
  // Adjust coordinates so that horizontal or vertical lines with integer
  // coordinates are one screen pixel in width when anti aliasing is turned on.
  x1 += units_per_pixel_*0.5;
  x2 += units_per_pixel_*0.5;
  double h = Height();
  y1 = h - y1 - units_per_pixel_*0.5;
  y2 = h - y2 - units_per_pixel_*0.5;
  painter_->drawLine(QPointF(x1, y1), QPointF(x2, y2));
}

void QtWindow::DrawArc(double x, double y, double r, double a1 , double a2) {
  CHECK(painter_);
  painter_->setPen(pen_);
  x += units_per_pixel_*0.5;
  y += units_per_pixel_*0.5;
  int startAngle = a1 * 16*180 / M_PI;
  while (a2 < a1) a2 += 2 * M_PI;
  int spanAngle = std::min(round((a2 - a1) * 16*180 / M_PI), 360.0 * 16);
  painter_->drawArc(QRectF(x-r, Height() - y - r, 2*r, 2*r),
                    startAngle, spanAngle);
}

void QtWindow::FillRectangle(double x, double y, double w, double h) {
  CHECK(painter_);
  painter_->setPen(QPen(color_, Qt::NoPen));
  painter_->setBrush(QBrush(color_));
  // Nothing is drawn if width or height is zero.
  if (w == 0 || h == 0) {
    return;
  }
  // Adjust if width or height is negative.
  if (w < 0) {
    x += w + units_per_pixel_;
    w = -w;
  }
  if (h < 0) {
    y += h + units_per_pixel_;
    h = -h;
  }
  // Adjust coordinates so that rectangles with integer coordinates don't have
  // fuzzy edges.
  painter_->drawRect(QRectF(x + units_per_pixel_*0.5,
                            Height() + units_per_pixel_*0.5 - (y + h),
                            w - units_per_pixel_, h - units_per_pixel_));
}

void QtWindow::FillCircle(double x, double y, double r) {
  CHECK(painter_);
  painter_->setPen(QPen(color_, Qt::NoPen));
  painter_->setBrush(QBrush(color_));
  // Adjust coordinates to line up with DrawArc().
  painter_->drawEllipse(QRectF(x - r + units_per_pixel_, Height() - y - r,
                               2*r - units_per_pixel_, 2*r - units_per_pixel_));
}

void QtWindow::DrawText(const char *s, double x, double y, TextAlignment halign,
                        TextAlignment valign, double angle) {
  CHECK(painter_);
  CHECK(current_font_size_);
  int height = metrics_[current_font_size_]->height();
  int width = metrics_[current_font_size_]->size(0, s).width();
  int ascent = metrics_[current_font_size_]->ascent();
  int descent = metrics_[current_font_size_]->descent();
  ::AlignText(width, height, descent, halign, valign, angle == 90, &x, &y);
  painter_->setPen(QPen(color_));
  painter_->setFont(*fonts_[current_font_size_]);
  y = Height() - 1 - y;
  x -= units_per_pixel_;
  if (angle == 90) {
    painter_->rotate(-90);
    painter_->drawText(QPointF(-y, x + ascent), s);
    painter_->rotate(90);
  } else {
    painter_->drawText(QPointF(x, y + ascent), s);
  }
}

void QtWindow::ShowSelectionRectangle(double x1, double y1,
                                      double x2, double y2) {
  y1 = Height() - 1 - y1;
  y2 = Height() - 1 - y2;
  rubber_band_->setGeometry(std::min(x1, x2), std::min(y1, y2),
                            fabs(x2 - x1), fabs(y2 - y1));
  rubber_band_->show();
}

void QtWindow::HideSelectionRectangle() {
  rubber_band_->hide();
}

QtWindow::Handle QtWindow::CreateImage(int width, int height,
                                       uint32_t *rgb_data) {
  CHECK(width > 0 && height > 0)

  // Copy the RGB data, invert Y, make sure the top 8 bits are 1's. Use
  // malloc() here instead of 'new' because QImage will call free() for us.
  uint32_t *data = (uint32_t*) malloc(width * height * sizeof(uint32_t));
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      data[y*width + x] = rgb_data[(height-1-y)*width + x] | 0xff000000;
    }
  }

  // Create the image, set it to automatically free 'data' when done.
  QImage *image = new QImage((uint8_t*) data, width, height,
                             QImage::Format_RGB32, free, data);

  // Return the handle.
  Handle h;
  h.width = width;
  h.height = height;
  h.pointer = image;
  return h;
}

void QtWindow::DestroyImage(Handle h) {
  delete (QImage*) h.pointer;
}

void QtWindow::DrawImage(Handle h, double x, double y,
                         double width, double height) {
  y = Height() - y - height;
  QRectF target(x, y, width, height);
  QRectF source(0, 0, h.width, h.height);
  painter_->drawImage(target, *((QImage*) h.pointer), source);
}

void QtWindow::SetFontSize(double size) {
  if (fonts_.count(size) == 0) {
    fonts_[size] = new QFont("Helvetica", size);
    metrics_[size] = new QFontMetrics(*fonts_[size]);
  }
  current_font_size_ = size;
}

double QtWindow::TextWidth(const char *s) {
  CHECK(current_font_size_ > 0);
  return metrics_[current_font_size_]->size(0, s).width();
}

double QtWindow::TextHeight(double *descent) {
  CHECK(current_font_size_ > 0);
  if (descent) {
    *descent = metrics_[current_font_size_]->descent();
  }
  return metrics_[current_font_size_]->height();
}

void QtWindow::mousePressEvent(QMouseEvent *event) {
  HandleMouse(event);
}

void QtWindow::mouseReleaseEvent(QMouseEvent *event) {
  HandleMouse(event);
}

void QtWindow::mouseDoubleClickEvent(QMouseEvent *event) {
  HandleMouse(event);
}

void QtWindow::mouseMoveEvent(QMouseEvent *event) {
  HandleMouse(event);
}

void QtWindow::keyPressEvent(QKeyEvent *event) {
  HandleKeyPress(MapKeyCode(event->key()));
  QWidget::keyPressEvent(event);
}

void QtWindow::paintEvent(QPaintEvent *event) {
  // Reset drawing state.
  style_ = LineStyle::SOLID;
  color_.setRgb(0, 0, 0);
  line_width_ = 0;
  SetPen();

  // Grab window state.
  units_per_pixel_ = 1.0 / devicePixelRatio();

  // Create painter.
  CHECK(painter_ == 0);
  painter_ = new QPainter(this);
  painter_->setRenderHint(QPainter::Antialiasing, true);

  // Actually do the painting.
  PaintEvent();

  // Done with the painter.
  delete painter_;
  painter_ = 0;
}

void QtWindow::HandleMouse(QMouseEvent *event) {
  int buttons = 0;
  if (event->buttons() & Qt::LeftButton) buttons |= 1;
  if (event->buttons() & Qt::MidButton) buttons |= 2;
  if (event->buttons() & Qt::RightButton) buttons |= 4;
  MouseEvent(event->x(), height() - 1 - event->y(), buttons);
}

// Set the pen according to the other drawing state.
void QtWindow::SetPen() {
  if (style_ == LineStyle::DOTTED) {
    pen_ = QPen(color_, line_width_, Qt::DotLine);
  } else {
    pen_ = QPen(color_, line_width_);
  }
}

int QtWindow::MapKeyCode(int keycode) {
  #define KEYMAP(a,b) case Qt::Key_##a : return b;
  switch (keycode) {
    KEYMAP(Escape   , ESCAPE)
    KEYMAP(Tab      , TAB)
    KEYMAP(Backtab  , BACKTAB)
    KEYMAP(Backspace, BACKSPACE)
    KEYMAP(Return   , RETURN)
    KEYMAP(Insert   , INSERT)
    KEYMAP(Delete   , DELETE)
    KEYMAP(Home     , HOME)
    KEYMAP(End      , END)
    KEYMAP(Left     , LEFT)
    KEYMAP(Up       , UP)
    KEYMAP(Right    , RIGHT)
    KEYMAP(Down     , DOWN)
    KEYMAP(PageUp   , PAGEUP)
    KEYMAP(PageDown , PAGEDOWN)
    KEYMAP(F1       , F1)
    KEYMAP(F2       , F2)
    KEYMAP(F3       , F3)
    KEYMAP(F4       , F4)
    KEYMAP(F5       , F5)
    KEYMAP(F6       , F6)
    KEYMAP(F7       , F7)
    KEYMAP(F8       , F8)
    KEYMAP(F9       , F9)
    KEYMAP(F10      , F10)
    KEYMAP(F11      , F11)
    KEYMAP(F12      , F12)
    default: return keycode;
  }
  #undef KEYMAP
}

}  // namespace GUI
