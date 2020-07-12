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

#ifndef __TOOLKIT_COLOR_BASED_SELECTION_H__
#define __TOOLKIT_COLOR_BASED_SELECTION_H__

#include "Eigen/Dense"
#include "error.h"
#include "shaders.h"

namespace gl {

// Color based selection mechanism. First set the viewport then create this
// object to prepare the GL state for selection geometry rendering. This will
// reset the viewport and various glEnables. Then set the camera transform to
// PickMatrix() * Projection() * ModelView() and render geometry using
// SetColorForSelection() to mark different objects.
class ColorBasedSelection {
 public:
  // The window coordinates of the point to be selected are x,y (0,0 is the
  // bottom left).
  ColorBasedSelection(int x, int y);

  // Once the scene is rendered, call this to read back the selected object
  // (>=0) or return -1 if there is none.
  int GetSelection();

  // Set a RGBA color value to used for for selection index n (>= 0). This sets
  // the "color" uniform in the current program. Return true on success or
  // false if n is out of range.
  bool SetColorForSelection(int n) const MUST_USE_RESULT;

  Eigen::Matrix4d PickMatrix() { return pick_matrix_; }

 private:
  gl::PushShader push_shader_;
  Eigen::Matrix4d pick_matrix_;
  int x_, y_;                   // x,y point passed to constructor
  int width_, height_;          // original viewport size

  // Given a color value that has been read back from the color buffer (with
  // dithering turned off!), return the selection number it corresponds to, or
  // -1 if it is invalid or the background color.
  int GetSelectionFromColor(unsigned char col[3]);
};

}  // namespace gl

#endif
