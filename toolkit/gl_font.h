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

// OpenGL font support.
//
// The approaches used to render text to opengl on wxWidgets and Qt are very
// different, and are therefore abstracted away in the API below. On wxWidgets
// we render text to a pixmap and then blend that into the scene. On Qt we use
// the text rendering capabilites of QPainter. QPainter touches a lot of opengl
// state and allows only heavyweight switching to and from user opengl
// rendering, so we batch all drawn strings and then emit them at the end of
// each draw.

#ifndef __TOOLKIT_GL_FONT_H__
#define __TOOLKIT_GL_FONT_H__

#include "text_alignment.h"
#include "Eigen/Dense"

struct Font;

#ifdef __TOOLKIT_WXWINDOWS__
typedef void QWidget;           // Unused type
#endif

#ifdef QT_CORE_LIB
class QWidget;
#endif

#ifdef GL_FONT_IMPLEMENTATION
  #ifdef __TOOLKIT_WXWINDOWS__
  class wxFont;
  struct Font {
    Font() : font(0) {}
    wxFont *font;
  };
  #endif

  #ifdef QT_CORE_LIB
  class QFont;
  class QFontMetrics;
  struct Font {
    Font() : font(0), fm(0) {}
    QFont *font;
    QFontMetrics *fm;
  };
  #endif
#endif

// Schedule a string draw to pixel coordinates (x,y) with (0,0) being the
// bottom left of the viewport.
void DrawString(const char *s, double x, double y, const Font *font,
                float red, float green, float blue,
                TextAlignment halign = TEXT_ALIGN_LEFT,
                TextAlignment valign = TEXT_ALIGN_BASELINE);

// Like DrawString, but (x,y,z) are in model coordinates with respect to the
// given transform matrix.
void DrawStringM(const char *s, double x, double y, double z,
                 const Eigen::Matrix4d &T, const Font *font,
                 float red, float green, float blue,
                 TextAlignment halign = TEXT_ALIGN_LEFT,
                 TextAlignment valign = TEXT_ALIGN_BASELINE);

// Render all text passed to DrawString() and friends since the last call to
// this function. This should be called at the end of each draw, as it will
// unpredictably change the opengl state.
void RenderDrawStrings(QWidget *win);

#endif
