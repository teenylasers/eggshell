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

#define GL_FONT_IMPLEMENTATION

#include "gl_font.h"
#include "error.h"
#include "gl_utils.h"
#include "shaders.h"
#include "myvector"
#include <stdint.h>

struct StringToDraw {
  std::string s;
  const Font *font;
  double x, y;
  float red, green, blue;
  TextAlignment halign, valign;
};

static std::vector<StringToDraw> strings;

void DrawString(const char *s, double x, double y, const Font *font,
                float red, float green, float blue,
                TextAlignment halign, TextAlignment valign) {
  if (!s || !s[0]) {
    return;
  }
  strings.resize(strings.size() + 1);
  strings.back().s = s;
  strings.back().font = font;
  strings.back().x = x;
  strings.back().y = y;
  strings.back().red = red;
  strings.back().green = green;
  strings.back().blue = blue;
  strings.back().halign = halign;
  strings.back().valign = valign;
}

void DrawStringM(const char *s, double x, double y, double z,
                 const Eigen::Matrix4d &T, const Font *font,
                 float red, float green, float blue,
                 TextAlignment halign, TextAlignment valign) {
  GLint viewport[4];
  GL(GetIntegerv)(GL_VIEWPORT, viewport);
  Eigen::Vector3d win;
  gl::Project(x, y, z, T, viewport, &win);
  DrawString(s, win[0], win[1], font, red, green, blue, halign, valign);
}

//***************************************************************************
// Qt implementation

#ifdef QT_CORE_LIB

#include <QPainter>
#include <QWidget>
#include <QFont>

void RenderDrawStrings(QWidget *win) {
  double scale = win->devicePixelRatio();
  QPainter painter(win);
  for (int i = 0; i < strings.size(); i++) {
    StringToDraw &s = strings[i];

    // Convert from opengl pixel coordinates to Qt window coordinates.
    double x = s.x / scale;
    double y = win->height() - s.y / scale;

    // Measure and align the text.
    double w = s.font->fm->size(0, s.s.c_str()).width();
    double h = s.font->fm->height();
    double ascent = s.font->fm->ascent();
    double descent = s.font->fm->descent();
    y = -y;     // Since Y coordinates increase going up in AlignText
    AlignText(w, h, descent, s.halign, s.valign, false, &x, &y);
    y = -y;

    // Draw the text.
    painter.setPen(QColor(s.red, s.green, s.blue, 255));
    painter.setFont(*s.font->font);
    painter.drawText(QPointF(x, y + ascent - 1), s.s.c_str());
  }
  strings.clear();
}

#endif  // QT_CORE_LIB
