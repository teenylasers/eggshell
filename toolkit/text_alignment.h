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

#ifndef __TOOLKIT_TEXT_ALIGNMENT_H__
#define __TOOLKIT_TEXT_ALIGNMENT_H__

// Text alignment constants for use in DrawString().
enum TextAlignment {
  TEXT_ALIGN_BASELINE = 0,      // For V-alignment
  TEXT_ALIGN_CENTER,            // For H- or V-alignment
  TEXT_ALIGN_TOP,
  TEXT_ALIGN_BOTTOM,
  TEXT_ALIGN_LEFT,
  TEXT_ALIGN_RIGHT,
};

// Given the width, height and descent of a text box, with x,y as the
// coordinates of the reference point described by halign,valign, adjust x,y so
// that it refers to the top left of the text box (or the bottom left, if
// rotated). Y coordinates increase going up. This is helpful to some
// implementations of text drawing functions. The 'height' is the the total
// height of the rendered text box, the 'descent' is the height between the
// bottom of that box and the baseline of the text.

inline void AlignText(double width, double height, double descent,
                      TextAlignment halign, TextAlignment valign,
                      bool rotate_90, double *x_arg, double *y_arg) {
  double x = *x_arg;
  double y = *y_arg;

  // Rotate x,y before alignment adjustment, if necessary.
  if (rotate_90) {
    double tmp = x;
    x = y;
    y = -tmp;
  }

  // Adjust x for text alignment.
  if (halign == TEXT_ALIGN_CENTER) {
    x -= width / 2;
  } else if (halign == TEXT_ALIGN_RIGHT) {
    x -= width - 1;
  }

  // Adjust y for text alignment.
  if (valign == TEXT_ALIGN_BASELINE) {
    y = y + height - descent - 1;
  } else if (valign == TEXT_ALIGN_CENTER) {
    y = y + height / 2 - 1;
  } else if (valign == TEXT_ALIGN_BOTTOM) {
    y += height - 1;
  }

  // Rotate x,y back after alignment adjustment, if necessary.
  if (rotate_90) {
    double tmp = x;
    x = -y;
    y = tmp;
  }

  *x_arg = x;
  *y_arg = y;
}

#endif
