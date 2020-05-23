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

// Draw 2D graphs. This is a generic mechanism that can be used to draw to
// OpenGL, Qt, etc. The rendering style, choice of label positions etc is
// somewhat in the style of Matlab.

#ifndef __TOOLKIT_PLOT_H__
#define __TOOLKIT_PLOT_H__

#include <stdint.h>
#include "myvector"
#include <string>
#include "text_alignment.h"
#include "error.h"
#include "colormaps.h"

namespace Plot {

// Window drawing and event interface. All coordinates are in pixels. Visible
// coordinates go from (0,0) at the bottom left to (width-1,height-1).
class Window {
 public:
  virtual ~Window();

  // ********** Interaction with the windowing system.

  // Return current window width.
  virtual double Width() const = 0;
  // Return current window height.
  virtual double Height() const = 0;
  // Clear the window and trigger redrawing.
  virtual void Refresh() const = 0;

  // ********** Drawing. All properties are reset to defaults on Refresh().

  // Change the drawing color to 0xRRGGBB. The default is 0.
  virtual void Color(uint32_t rrggbb) = 0;
  // Use the given width for lines. The default is 1.
  virtual void LineWidth(double w) = 0;
  // Start drawing lines with the given style.
  enum Style {
    TRACE,              // A solid line for traces
    BORDER,             // A solid line used for ticks and borders
    GRID,               // A grid line
  };
  virtual void StartLine(Style style) = 0;
  // End a series of Line() calls..
  virtual void EndLine() = 0;
  // Draw a line segment. A series of these calls will always be bracketed by
  // StartLine() and EndLine().
  virtual void Line(double x1, double y1, double x2, double y2) = 0;
  // Draw a rectangle.
  virtual void Rectangle(double x, double y, double w, double h) = 0;
  // Draw text.
  virtual void DrawText(const char *s, double x, double y,
                        TextAlignment halign, TextAlignment valign,
                        bool rotate_90 = false) = 0;
  // Draw a selection rectangle. This is called in pairs with the same
  // coordinates with state=true to draw it then state=false to restore the
  // window to its previous state.
  virtual void SelectionRectangle(double x1, double y1,
                                  double x2, double y2, bool state) = 0;

  // ********** Drawing of images.

  // RGB data is stored row-wise in 'rgb_data', which is an array of
  // width*height pixels. Each pixel is 3 bytes. An image is created from this
  // data and a handle to it returned. The 'rgb_data' can be reclaimed by the
  // caller when this function returns.
  struct Handle {
    int width, height;          // Image size
    void *pointer;              // If pointer is 0 this is a 'null handle'
    Handle() : width(0), height(0), pointer(0) {}
  };
  virtual Handle CreateImage(int width, int height, uint8_t *rgb_data) = 0;
  // Destroy a previously created image given its handle. Images may consume
  // significant memory so they should be destroyed when no longer needed. If
  // the handle is null nothing is done.
  virtual void DestroyImage(Handle h) = 0;
  // Draw an image in the given rectangle. The top left is at (x,y) and is the
  // first pixel of rgb_data.
  virtual void DrawImage(Handle h, double x, double y,
                                   double width, double height) = 0;

  // ********** Fonts.

  virtual void UseLabelFont() = 0;                      // For main labels
  virtual void UseLabelSmallFont() = 0;                 // For exponents
  virtual double TextWidth(const char *s) = 0;          // In current font
  virtual double TextHeight(double *descent = 0) = 0;   // In current font

  // ********** Utility functions, not supplied by the user.

  // Text measurement and drawing functions that allow for superscript and
  // subscript text using the label and small-label fonts. Superscript and
  // subscript text are denoted like ^{this} and _{this} respectively. Nested ^
  // and _ commands are allowed, and the special characters (^,_,{,}) can be
  // escaped with '\' if they need to be drawn. These functions leave the
  // current font in an undetermined state on exit.
  double FormattedTextWidth(const char *s);
  double FormattedTextHeight(const char *s, double *descent = 0);
  void DrawFormattedText(const char *s, double x, double y,
                         TextAlignment halign, TextAlignment valign);

 protected:
  typedef std::vector<std::pair<int, std::string> > Pieces;
  double FormattedTextWidthHelper(const Pieces &pieces);
  double FormattedTextHeightHelper(const Pieces &pieces, double *descent);
};

// Input to AxisLayout(). Coordinates suffixed in 'px' are in pixels, i.e.
// window coordinates, otherwise they are in graph coordinates.
struct AxisLayoutInput {
  bool x_axis;                  // True for the X axis, false for the Y axis
  double data_min;              // Smallest graph coordinates of data
  double data_max;              // Largest graph coordinates of data
  double size_px;               // Size in pixels of graph plot area
  double min_label_gap_px;      // Minimum acceptable pixels between labels
  bool tight_fit;               // True to set axis bounds to data_min & max

  AxisLayoutInput();            // Initialize everything to zero
};

// Output from AxisLayout().
struct AxisLayoutOutput {
  // The transformation from graph coordinates to window coordinates is:
  // window_coord = (graph_coord - axis_min) / (axis_max - axis_min) * size_px
  double log10_scale;   // Labels to be read as multiplied by 10^log10_scale
  double axis_min;      // Low side of axis in graph coordinates
  double axis_max;      // High side of axis in graph coordinates
  double tick;          // Space between axis ticks/labels in graph coordinates
  int64_t lmin;         // First label to draw in units of 'tick'
  int64_t lmax;         // Last label to draw in units of 'tick'
  // Text of all labels to draw, from lmin to lmax.
  std::vector<std::string> labels;

  AxisLayoutOutput();
};

// Describe the X and Y axis for a 2D plot.
struct Axis {
  AxisLayoutInput in[2];              // X axis, Y axis
  AxisLayoutOutput out[2];            // X axis, Y axis
};

// Compute the positions of labels for a single axis of the graph. This can be
// used for either the X or Y axis. The window to use for font measurements is
// 'win'.
void AxisLayout(Window *win, const AxisLayoutInput &in, AxisLayoutOutput *out);

// Manage plotting of a 2D graph.
class Plot2D {
 public:
  explicit Plot2D(Window *win);
  virtual ~Plot2D();

  // Reset all plot state.
  void Clear();

  // Add a new trace given by 'n' coordinates in 'x' and 'y'. If the rrggbb
  // color is not specified then a default color will be picked that is
  // different for each trace. The most recent trace added is displayed on top
  // of all of the others.
  void AddTrace(int n, const double *x, const double *y,
                uint32_t rrggbb = -1, double line_width = 1);

  // Add a new trace given by the vectors or matrices 'x' and 'y'. This is
  // compatible with STL vectors and eigen vectors.
  template<class Tx, class Ty>
  void AddTrace(const Tx &x, const Ty &y,
                uint32_t rrggbb = -1, double line_width = 1) {
    CHECK(x.size() == y.size());
    AddTrace(x.size(), &x[0], &y[0], rrggbb, line_width);
  }

  // Add a label for the most recently added trace (this will go in the plot
  // legend). If there are no traces this will do nothing.
  void AddTraceLabel(const char *label);

  // Add an image. RGB data is in CreateImage() format. Currently there can be
  // only one image at a time, so new images will replace any old images.
  void AddImage(int width, int height, uint8_t *rgb_data);

  // Add an image that represents data in the given matrix. This is compatible
  // with Eigen matrices. Matrix values between minvalue and maxvalue are
  // mapped to colors from the given colormap function.
  template<class T, class V>
  void AddImage(const T &matrix, V minvalue, V maxvalue,
                ColorMap::Function colormap = ColorMap::Jet) {
    uint8_t palette[256][3];
    uint8_t *rgb_data = new uint8_t[matrix.rows() * matrix.cols() * 3];
    ColorMap::ComputePalette(colormap, palette);
    int index = 0;
    for (int i = 0; i < matrix.rows(); i++) {
      for (int j = 0; j < matrix.cols(); j++) {
        double value = std::max(0.0, std::min(255.0,
            (matrix(i, j) - minvalue) / (maxvalue - minvalue) * 255.0));
        int ivalue = static_cast<int>(value);
        rgb_data[index++] = palette[ivalue][0];
        rgb_data[index++] = palette[ivalue][1];
        rgb_data[index++] = palette[ivalue][2];
      }
    }
    AddImage(matrix.cols(), matrix.rows(), rgb_data);
    delete[] rgb_data;
  }
  template<class T>
  void AddImage(const T &matrix, ColorMap::Function colormap = ColorMap::Jet) {
    AddImage(matrix, matrix.minCoeff(), matrix.maxCoeff(), colormap);
  }

  // Set plot axis properties.
  void Grid(bool grid = true) { grid_ = grid; }
  void SetXAxisLabel(const char *label) { x_axis_label_ = label; }
  void SetYAxisLabel(const char *label) { y_axis_label_ = label; }
  void SetTitle(const char *title) { title_ = title; }
  void AxisXTight(bool tight = true) { axis_[0].in[0].tight_fit = tight; }
  void AxisYTight(bool tight = true) { axis_[0].in[1].tight_fit = tight; }
  void SetXAxis(double x1, double x2);
  void SetYAxis(double y1, double y2);

  // Draw this plot to the given window.
  void Draw();

  // Window event handling functions, to e.g. allow the user to zoom in to the
  // graph. The current mouse position in the window is x,y. The bottom 3 bits
  // of button_state indicate the left (bit 0), middle (bit 1) and right (bit
  // 2) button state. When the button is pressed the bit is 1. This function is
  // called whenever x,y or button_state changes, except that it does not have
  // to be called when only x or y change and the button_state is 0.
  virtual void EventMouse(double x, double y, int button_state);

  // Set and get the axis stack. This is useful to maintain the current zoomed
  // state even when the plot is recreated. If the user is zoomed in then the
  // stack will contain more than one entry.
  const std::vector<Axis> &GetAxisStack() const { return axis_; }
  void SetAxisStack(const std::vector<Axis> &axis) { axis_ = axis; }

 private:
  Window *win_;                         // Window this plot is attached to
  double plot_x1_, plot_y1_;            // Plot area lower left
  std::string x_axis_label_;            // X axis label
  std::string y_axis_label_;            // Y axis label
  std::string title_;                   // Title
  bool grid_;                           // Draw a grid?
  double first_x_, first_y_;            // First click position in a drag
  double last_x_, last_y_;              // Last click position in a drag
  int last_button_state_;               // Last button state

  struct Trace {
    std::vector<double> x, y;           // Same size, size >= 1
    uint32_t color;
    double line_width;
    std::string label;
  };
  std::vector<Trace> traces_;           // Active traces in plot
  std::vector<Axis> axis_;              // Stack of original and zoomed-in axes
  Window::Handle image_handle_;         // Handle to one active image

  // Convert graph coordinates to window coordinates and back.
  double GraphToWinX(double x) const;
  double GraphToWinY(double y) const;
  double WinXToGraph(double x) const;
  double WinYToGraph(double y) const;

  // Update the axis_[0] data bounds from the given trace. Return true if at
  // least one of the x,y points is finite (i.e. not infinite or NaN).
  bool UpdateDataBoundsFromTrace(int n, const double *x, const double *y);

  // Clip the line (x1,y1)-(x2,y2) to the rectangle (0,0)-(w,h). Return true if
  // any part of the line is left or false if the line is entirely clipped out.
  bool ClipLine(double w, double h,
                double *x1, double *y1, double *x2, double *y2);
};

// Run all internal checks that involve drawing to a window. This checks the
// window implementation too. Visual inspection is required to determine if the
// tests passed. The block_character can be \x02 for GLUT and e.g. \x08 for
// windows fonts.
void RunAllWindowTests(Window *win, const char *block_character);

}  // namespace Plot

#endif
