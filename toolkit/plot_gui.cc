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

#include "plot_gui.h"
#include "error.h"
#include <math.h>

//***************************************************************************
// Plot::Window interface for Qt.

#ifdef QT_CORE_LIB

#include <QPainter>
#include <QMouseEvent>
#include <QRubberBand>
#include <QImage>
#include "platform.h"

namespace Plot {

const int kLabelFontSize = 9 * FONT_SCALE;
const int kLabelSmallFontSize = 6 * FONT_SCALE;
const int kNumFonts = 2;

class PlotWindow : public Plot::Window {
 public:
  QWidget *win;
  QPainter *painter;
  int win_height;
  Style style;
  QColor color;
  double line_width;
  QPen pen;
  QFont *fonts[kNumFonts];            // Regular and small font
  QFontMetrics *metrics[kNumFonts];   // Metrics for the fonts
  int current_font;                   // index into fonts[]
  QRubberBand *rubber_band;

  PlotWindow(QWidget *_win) {
    win = _win;
    painter = 0;
    win_height = win->height();
    style = BORDER;
    color.black();
    line_width = 0;
    fonts[0] = new QFont("Helvetica", kLabelFontSize);
    fonts[1] = new QFont("Helvetica", kLabelSmallFontSize);
    for (int i = 0; i < kNumFonts; i++) {
      metrics[i] = new QFontMetrics(*fonts[i]);
    }
    current_font = 0;
    rubber_band = new QRubberBand(QRubberBand::Rectangle, win);
  }

  // This is called before drawing, to reset state.
  void Reset() {
    CHECK(painter == 0);
    painter = new QPainter(win);
    painter->setRenderHint(QPainter::Antialiasing, true);
    win_height = win->height();
    style = BORDER;
    color.black();
    line_width = 0;
    SetPen();
    UseLabelFont();
  }

  // This is called immediately after drawing.
  void DoneDrawing() {
    CHECK(painter);
    delete painter;
    painter = 0;
  }

  ~PlotWindow() {
    CHECK(painter == 0);
    for (int i = 0; i < kNumFonts; i++) {
      delete metrics[i];
      delete fonts[i];
    }
    delete rubber_band;
  }

  void SetPen() {
    if (style == GRID) {
      pen = QPen(color, 0, Qt::DotLine);
    } else if (style == BORDER) {
      pen = QPen(color, 0);
    } else {
      pen = QPen(color, line_width);
    }
  }

  double Width() const override {
    return win->width();
  }

  double Height() const override {
    return win_height;
  }

  void Refresh() const override {
    win->update();
  }

  void Color(uint32_t rrggbb) override {
    color.setRgb((rrggbb >> 16) & 0xff,
                 (rrggbb >> 8) & 0xff,
                  rrggbb & 0xff);
    SetPen();
  }

  void LineWidth(double w) override {
    line_width = w;
    SetPen();
  }

  void StartLine(Style s) override {
    style = s;
    SetPen();
  }

  void EndLine() override {
  }

  void Line(double x1, double y1, double x2, double y2) override {
    y1 = win_height - y1;
    y2 = win_height - y2;
    if (style == BORDER || style == GRID) {
      // Necessary to achieve single-pixel-width (non antialiased) lines.
      x1 = round(x1);
      y1 = round(y1);
      x2 = round(x2);
      y2 = round(y2);
    }
    painter->setPen(pen);
    painter->drawLine(QPointF(x1, y1), QPointF(x2, y2));
  }

  void Rectangle(double x, double y, double w, double h) override {
    painter->setPen(QPen(color, Qt::NoPen));
    painter->setBrush(QBrush(color));
    painter->drawRect(QRectF(x, win_height - (y + h), w, h));
  }

  void DrawText(const char *s, double x, double y,
                TextAlignment halign, TextAlignment valign,
                bool rotate_90) override {
    int height = metrics[current_font]->height();
    int width = metrics[current_font]->size(0, s).width();
    int ascent = metrics[current_font]->ascent();
    int descent = metrics[current_font]->descent();
    ::AlignText(width, height, descent, halign, valign, rotate_90, &x, &y);
    painter->setPen(QPen(color));
    y = win_height - y;
    if (rotate_90) {
      painter->rotate(-90);
      painter->drawText(QPointF(-y, x + ascent), s);
      painter->rotate(90);
    } else {
      painter->drawText(QPointF(x, y + ascent), s);
    }
  }

  void SelectionRectangle(double x1, double y1, double x2, double y2,
                          bool state) override {
    y1 = win_height - y1;
    y2 = win_height - y2;
    rubber_band->setGeometry(std::min(x1, x2), std::min(y1, y2),
                             fabs(x2 - x1), fabs(y2 - y1));
    if (state) {
      rubber_band->show();
    } else {
      rubber_band->hide();
    }
  }

  Handle CreateImage(int width, int height, uint8_t *rgb_data) override {
    CHECK(width > 0 && height > 0)

    // Copy the RGB data.
    uint32_t *data = (uint32_t*) malloc(width * height * sizeof(uint32_t));
    for (int y = 0; y < height; y++) {
      uint32_t *dptr = data + y * width;
      uint8_t *sptr = rgb_data + y * width * 3;
      for (int x = 0; x < width; x++) {
        *dptr++ = 0xff000000 | (sptr[0] << 16) | (sptr[1] << 8) | sptr[2];
        sptr += 3;
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

  void DestroyImage(Handle h) override {
    if (h.pointer) {
      delete (QImage*) h.pointer;
    }
  }

  void DrawImage(Handle h, double x, double y,
                 double width, double height) override {
    y = win_height - y;
    QRectF target(x, y, width, height);
    QRectF source(0, 0, h.width, h.height);
    painter->drawImage(target, *((QImage*) h.pointer), source);
  }

  void UseLabelFont() override {
    current_font = 0;
    painter->setFont(*fonts[current_font]);
  }

  void UseLabelSmallFont() override {
    current_font = 1;
    painter->setFont(*fonts[current_font]);
  }

  double TextWidth(const char *s) override {
    return metrics[current_font]->size(0, s).width();
  }

  double TextHeight(double *descent) override {
    if (descent) {
      *descent = metrics[current_font]->descent();
    }
    return metrics[current_font]->height();
  }
};

}  // Plot namespace

#endif  // QT_CORE_LIB

//***************************************************************************
// qtPlot.

#ifdef QT_CORE_LIB

qtPlot::qtPlot(QWidget *parent) : QWidget(parent) {
  plot_window_ = new Plot::PlotWindow(this);
  plot_ = new Plot::Plot2D(plot_window_);
}

qtPlot::~qtPlot() {
  delete plot_;
  delete plot_window_;
}

void qtPlot::paintEvent(QPaintEvent *event) {
  plot_window_->Reset();
  plot_->Draw();
  plot_window_->DoneDrawing();
}

void qtPlot::mousePressEvent(QMouseEvent *event) {
  HandleMouse(event);
}

void qtPlot::mouseReleaseEvent(QMouseEvent *event) {
  HandleMouse(event);
}

void qtPlot::mouseDoubleClickEvent(QMouseEvent *event) {
  HandleMouse(event);
}

void qtPlot::mouseMoveEvent(QMouseEvent *event) {
  HandleMouse(event);
}

void qtPlot::HandleMouse(QMouseEvent *event) {
  int buttons = 0;
  if (event->buttons() & Qt::LeftButton) buttons |= 1;
  if (event->buttons() & Qt::MidButton) buttons |= 2;
  if (event->buttons() & Qt::RightButton) buttons |= 4;
  plot_->EventMouse(event->x(), height() - 1 - event->y(), buttons);
}

#endif  // QT_CORE_LIB
