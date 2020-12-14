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

// A simple, abstract window drawing and event interface. This allows GUI code
// to be written that is portable across windowing systems. Of course,
// libraries like Qt purport to do this, but those libraries tend to be
// gigantic complicated things with features that are 99% unnecessary for most
// applications. This interface is vastly simpler and contains just the basics
// necessary for other toolkit pieces.
//
// * The GUI::Window base class defines virtual drawing functions and event
//   handling functions.
// * The GUI::QtWindow class is a QWidget that also inherits GUI::Window. It
//   handles Qt events and implements the GUI::Window drawing functions.
// * Clients of this library inherit from GUI::Window (e.g. making a new
//   AppLogic class) and implement the event handling functions functions
//   there. To hook up AppLogic to a Qt window, make a class that inherits from
//   both AppLogic and GUI::QtWindow.

#ifndef __TOOLKIT_GUI_H__
#define __TOOLKIT_GUI_H__

#include <stdint.h>
#include "text_alignment.h"

class QWidget;

namespace GUI {

// An abstract window class.

class Window {
 public:
  virtual ~Window() = 0;

  // **********
  // Basic interactions with the window.

  // Return current window width, in pixels.
  virtual double Width() const = 0;

  // Return current window height, in pixels.
  virtual double Height() const = 0;

  // Clear the window and trigger redrawing. The PaintEvent() function will
  // eventually be called.
  virtual void Refresh() = 0;

  // **********
  // Drawing. All coordinates are in pixels. Visible coordinates go from (0,0)
  // at the bottom left to (width-1,height-1). Drawing functions can only be
  // called from within the PaintEvent() function. All properties are reset to
  // their defaults on each entry to the PaintEvent() function. Anti aliasing
  // is used, but if integer coordinates are given to drawing functions then
  // the minimal amount of fuzzyness will result for horizontal or vertical
  // lines or filled rectangles.

  // Change the drawing color to 0xRRGGBB. The default is 0.
  virtual void SetColor(uint32_t rrggbb) = 0;

  // Use the given width for lines and arcs. The default is 1.
  virtual void SetLineWidth(double w) = 0;

  // Use the given style for lines and arcs. The default is SOLID.
  enum class LineStyle {
    SOLID,              // A solid line for traces
    DOTTED,             // A dotted line
  };
  virtual void SetLineStyle(LineStyle style) = 0;

  // Draw a line segment.
  virtual void DrawLine(double x1, double y1, double x2, double y2) = 0;

  // Draw an arc centered at x,y, with radius r, and with starting and ending
  // angles (in radians) given by a1 and a2. The arc is drawn counter clockwise
  // from a1 until a2 is reached. A complete circle is drawn if a2 >= a1+2*pi.
  virtual void DrawArc(double x, double y, double r,
                       double a1 = 0, double a2 = 10) = 0;

  // Draw a filled rectangle with corner pixel x,y that is w pixels wide and h
  // pixels heigh. The width and height can be negative.
  virtual void FillRectangle(double x, double y, double w, double h) = 0;

  // Draw a filled circle.
  virtual void FillCircle(double x, double y, double r) = 0;

  // Draw text given by the string 's' in the current font. The x,y coordinates
  // of the point described by halign,valign are given. The angle is given in
  // degrees, at least 0 and 90 is supported.
  virtual void DrawText(const char *s, double x, double y,
                        TextAlignment halign, TextAlignment valign,
                        double angle = 0) = 0;

  // Draw a selection rectangle, as an overlay on the current window.
  virtual void ShowSelectionRectangle(double x1, double y1,
                                      double x2, double y2) = 0;

  // Hide the selection rectangle.
  virtual void HideSelectionRectangle() = 0;

  // **********
  // Drawing of images.

  // Create a handle to an internal representation of an image. RGB data is
  // stored row-wise in 'rgb_data', which is an array of width*height pixels.
  // Each pixel is one uint32_t, in 0xrrggbb format. The 'rgb_data' can be
  // reclaimed by the caller when this function returns.
  struct Handle {
    int width, height;          // Image size
    void *pointer;              // If pointer is 0 this is a 'null handle'
    Handle() : width(0), height(0), pointer(0) {}
  };
  virtual Handle CreateImage(int width, int height, uint32_t *rgb_data) =0;

  // Destroy a previously created image given its handle. Images may consume
  // significant memory so they should be destroyed when no longer needed. If
  // the handle is null nothing is done.
  virtual void DestroyImage(Handle h) = 0;

  // Draw an image scaled to the given rectangle. The top left is at (x,y) and
  // is the first pixel of rgb_data.
  virtual void DrawImage(Handle h, double x, double y,
                                   double width, double height) = 0;

  // **********
  // Fonts.

  // Set the size of the ascender of the current font, in pixels.
  virtual void SetFontSize(double size) = 0;

  // Get the size of the string 's' in the current font, in pixels.
  virtual double TextWidth(const char *s) = 0;
  virtual double TextHeight(double *descent = 0) = 0;

  // **********
  // Events.

  // Redraw the window.
  virtual void PaintEvent() = 0;

  // Window event handling functions. The current mouse position in the window
  // is x,y. The bottom 3 bits of button_state indicate the left (bit 0),
  // middle (bit 1) and right (bit 2) button state. When the button is pressed
  // the bit is 1. This function is called whenever x,y or button_state
  // changes.
  virtual void MouseEvent(double x, double y, int button_state);

  // Handle key presses. The key code for alphanumeric keys is their ASCII
  // code, other key codes are defined in this enum:
  enum Key {
    ESCAPE = 256, TAB, BACKTAB, BACKSPACE, RETURN, INSERT, DELETE, HOME,
    END, LEFT, UP, RIGHT, DOWN, PAGEUP, PAGEDOWN, F1, F2, F3, F4, F5, F6,
    F7, F8, F9, F10, F11, F12
  };
  virtual void HandleKeyPress(int keycode);
};

}  // namespace GUI

//***************************************************************************
// An implementation in Qt.

#ifdef QT_CORE_LIB

#include <QWidget>
#include <QPen>
class QRubberBand;
class QFont;
class QFontMetrics;

namespace GUI {

class QtWindow : public QWidget, public virtual Window {
 public:
  explicit QtWindow(QWidget *parent);
  ~QtWindow();

  // Override Window functions.
  double Width() const override;
  double Height() const override;
  void Refresh() override;
  void SetColor(uint32_t rrggbb) override;
  void SetLineWidth(double w) override;
  void SetLineStyle(LineStyle style) override;
  void DrawLine(double x1, double y1, double x2, double y2) override;
  void DrawArc(double x, double y, double r, double a1 , double a2) override;
  void FillRectangle(double x, double y, double w, double h) override;
  void FillCircle(double x, double y, double r) override;
  void DrawText(const char *s, double x, double y, TextAlignment halign,
                TextAlignment valign, double angle) override;
  void ShowSelectionRectangle(double x1, double y1,
                              double x2, double y2) override;
  void HideSelectionRectangle() override;
  Handle CreateImage(int width, int height, uint32_t *rgb_data) override;
  void DestroyImage(Handle h) override;
  void DrawImage(Handle h, double x, double y,
                 double width, double height) override;
  void SetFontSize(double size) override;
  double TextWidth(const char *s) override;
  double TextHeight(double *descent) override;

  // QWidget event handling.
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;
  void paintEvent(QPaintEvent *event) override;
  void HandleMouse(QMouseEvent *event);

 private:
  QPainter *painter_ = 0;       // Widget painter, only nonzero in PaintEvent()
  QRubberBand *rubber_band_ = 0;

  // Drawing state.
  Window::LineStyle style_ = LineStyle::SOLID;  // Line style
  QColor color_;                                // Line and fill color
  double line_width_ = 0;                       // Line width
  QPen pen_;                                    // Pen used to draw lines
  std::map<double, QFont*> fonts_;              // Fonts indexed by size
  std::map<double, QFontMetrics*> metrics_;     // Font metrics indexed by size
  double current_font_size_ = 0;

  // Window state.
  double units_per_pixel_ = 1;                  // Usually 1 or 0.5

  void ResetDrawingState();
  void SetPen();
  int MapKeyCode(int keycode);
};

}  // namespace GUI

#endif

#endif
