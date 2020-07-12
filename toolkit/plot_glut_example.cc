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

// An example of how to use the plotting code on freeglut. Note that freeglut
// does not support the rendering of rotated text so the vertical labels do not
// render properly.
//
//   g++ -Wall plot_glut_example.cc plot.cc error.cc testing.cc \
//     -I/opt/local/include -L/opt/local/lib -lglut -lGL -lGLU

#include "plot.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "GL/freeglut.h"
#include "error.h"

// Globals.
int window_width  = 800;
int window_height = 800;
int mouse_button_state = 0;
int mouse_x = 0;
int mouse_y = 0;

//***************************************************************************
// MyWindow.

struct MyWindow : public Plot::Window {
  void *font;
  bool selection_rectangle_visible;

  MyWindow() {
    font = GLUT_BITMAP_9_BY_15;
    selection_rectangle_visible = false;
  }

  double Width() const {
    return window_width;
  }

  double Height() const {
    return window_height;
  }

  void Refresh() const {
    glutPostRedisplay();
  }

  void Color(uint32_t rrggbb) {
    glColor3f(float((rrggbb >> 16) & 0xff) / 255.0f,
              float((rrggbb >> 8) & 0xff) / 255.0f,
              float((rrggbb >> 0) & 0xff) / 255.0f);
  }

  void LineWidth(double w) {
    glLineWidth(std::max(1, int(round(w))));
  }

  void StartLine(Style style) {
    if (style == GRID) {
      // On windows GL the order of the following two lines seems to be
      // significant. If the glEnable() is called before the glLineStipple()
      // then the pattern may not stick.
      glLineStipple(1, 0x3333);
      glEnable(GL_LINE_STIPPLE);
    } else {
      // style is TRACE or BORDER.
      glDisable(GL_LINE_STIPPLE);
    }
    glBegin(GL_LINES);
  }

  void EndLine() {
    glEnd();
    glDisable(GL_LINE_STIPPLE);
  }

  void Line(double x1, double y1, double x2, double y2) {
    glVertex3f(x1, y1, 0);
    glVertex3f(x2, y2, 0);
  }

  void Rectangle(double x, double y, double w, double h) {
    glBegin(GL_TRIANGLE_STRIP);
    glVertex3f(x + w, y, 0);
    glVertex3f(x + w, y+h, 0);
    glVertex3f(x, y, 0);
    glVertex3f(x, y + h, 0);
    glEnd();
  }

  void DrawText(const char *s, double x, double y,
                TextAlignment halign, TextAlignment valign, bool rotate_90) {
    double descent;
    double height = TextHeight(&descent);
    AlignText(TextWidth(s), height, descent, halign, valign, rotate_90, &x, &y);
    glRasterPos2d(x, y - height + descent);
    glutBitmapString(font, (const unsigned char *) s);
  }

  void SelectionRectangle(double x1, double y1, double x2, double y2,
                          bool state) {
    if (selection_rectangle_visible ^ state) {
      glLineStipple(1, 0xf0f0);
      glEnable(GL_LINE_STIPPLE);
      glDrawBuffer(GL_FRONT);
      glEnable(GL_COLOR_LOGIC_OP);
      glLogicOp(GL_INVERT);
      glBegin(GL_LINE_LOOP);
      glVertex3f(x1, y1, 0);
      glVertex3f(x2, y1, 0);
      glVertex3f(x2, y2, 0);
      glVertex3f(x1, y2, 0);
      glEnd();
      glDisable(GL_COLOR_LOGIC_OP);
      glDisable(GL_LINE_STIPPLE);
      glDrawBuffer(GL_BACK);
      glFlush();
    }
    selection_rectangle_visible = state;
  }

  void UseLabelFont() {
    font = GLUT_BITMAP_HELVETICA_18;
  }

  void UseLabelSmallFont() {
    font = GLUT_BITMAP_HELVETICA_12;
  }

  double TextWidth(const char *s) {
    const unsigned char *us = (const unsigned char *) s;
    return glutBitmapLength(font, us);
  }

  double TextHeight(double *descent) {
    int height = glutBitmapHeight(font);
    if (descent) {
      *descent = 0.3 * height;  // A guess as glut does not supply this info
    }
    return height;
  }

  void StartDraw() {
    glClearColor(1, 1, 1, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    Color(0);
    LineWidth(1);
    selection_rectangle_visible = false;
  }
};

MyWindow mywindow;
Plot::Plot2D plot;

//***************************************************************************
// GLUT interface.

void myGlutDisplayFunc() {
  glViewport(0, 0, window_width, window_height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  double left = 0;
  double right = window_width;
  double bottom = 0;
  double top = window_height;
  glOrtho(left, right, bottom, top, -100, 100);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  mywindow.StartDraw();
  plot.Draw(&mywindow);

  // Done.
  while (glGetError() != GL_NO_ERROR) {
    printf("GL error\n");
  }
  glutSwapBuffers();
}

void myGlutKeyboardFunc(unsigned char key, int x, int y) {
  switch (key) {
    case '\e':
      exit(0);
      break;
  }
}

void myGlutMouseFunc(int button, int state, int x, int y) {
  mouse_x = x;
  mouse_y = window_height - 1 - y;
  if (button >= 0 && button <= 2) {
    if (state) {
      mouse_button_state &= ~(1 << (2 - button));
    } else {
      mouse_button_state |= (1 << (2 - button));
    }
    plot.EventMouse(&mywindow, mouse_x, mouse_y, mouse_button_state);
  }
}

void myGlutMotionFunc(int x, int y) {
  mouse_x = x;
  mouse_y = window_height - 1 - y;
  plot.EventMouse(&mywindow, mouse_x, mouse_y, mouse_button_state);
}

void myGlutReshapeFunc(int width, int height) {
  window_width = width;
  window_height = height;
}

int main(int argc, char **argv) {
  // For convenient printf() debugging it is necessary to turn off stdout
  // buffering in the C runtime library on windows, as the only alternative is
  // full buffering which means that printf()s do not show up immediately.
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  // Plot::RunAllTests();

  // Setup an example plot.
  std::vector<double> x, y;
  for (int i = 0; i <= 100; i++) {
    x.push_back((i / 100.0) + 0.0201);
    y.push_back((double(rand()) / double(RAND_MAX)) * 1e8);
  }
  plot.AddTrace(x, y);
  plot.AddTraceLabel("XYZ^{prq}");
  x.clear();
  y.clear();
  double yy = 0;
  for (int i = 0; i <= 100; i++) {
    x.push_back(((i / 100.0) + 0.0201) * 1e21);
    y.push_back(yy * 1e9);
    yy += (double(rand()) / double(RAND_MAX)) - 0.5;
  }
  plot.AddTrace(x, y, 0xff0000, 2);
  plot.AddTraceLabel("ABC_{def}");
  plot.SetXAxisLabel("X Axis Label");
  plot.SetYAxisLabel("Y Axis Label");
  // plot.AxisXTight();
  // plot.AxisYTight();
  plot.Grid();

  // GLUT setup.
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(window_width, window_height);
  glutCreateWindow("Foo");
  glutDisplayFunc(&myGlutDisplayFunc);
  glutKeyboardFunc(&myGlutKeyboardFunc);
  glutReshapeFunc(&myGlutReshapeFunc);
  glutMouseFunc(&myGlutMouseFunc);
  glutMotionFunc(&myGlutMotionFunc);
  glutMainLoop();

  return 0;
}
