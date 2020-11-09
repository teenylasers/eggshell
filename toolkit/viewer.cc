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

#include "viewer.h"
#include "color_based_selection.h"
#include <algorithm>

using namespace Eigen;

//***************************************************************************
// GLViewerBase.

GLViewerBase::GLViewerBase() {
  have_depth_buffer_ = false;
  camera_ = &cameras_[0];
  allow_rotation_ = true;
  last_x_ = last_y_ = buttons_ = the_button_ = 0;
  model_pt_ << 0, 0, 0;
  rotate_click_ = false;
  pan_click_ = false;
}

void GLViewerBase::SetPerspective(bool perspective) {
  camera_->perspective = perspective;
  Redraw();
}

void GLViewerBase::SetViewAngle(double angle) {
  camera_->view_angle = angle;
  Redraw();
}

void GLViewerBase::ZoomExtents() {
  double bounds[6];
  GetBoundingBox(bounds);
  camera_->EnsureBoxVisible(bounds, GetAspectRatio());
  Redraw();
}

void GLViewerBase::Zoom(double scale_factor) {
  int width, height;
  GetScaledClientSize(&width, &height);
  Vector3d p;
  PixelToModelCoords(width / 2, height / 2, &p);
  Zoom(scale_factor, p);
}

void GLViewerBase::Zoom(double scale_factor, Vector3d center) {
  camera_->Zoom(scale_factor, center);
  Redraw();
}

void GLViewerBase::SwitchCamera(int camera_number) {
  CHECK(camera_number >= 0 && camera_number < MAX_CAMERAS);
  camera_ = &cameras_[camera_number];
  Redraw();
}

void GLViewerBase::Look(Direction direction) {
  // Set the camera direction then ensure the bounding box is visible.
  switch (direction) {
    case LOOK_AT_XY_PLANE_FROM_PLUS_Z:
      camera_->SetCameraPlane(Vector3d(1, 0, 0), Vector3d(0, 1, 0));
      break;
    case LOOK_AT_YX_PLANE_FROM_PLUS_Z:
      camera_->SetCameraPlane(Vector3d(0, 1, 0), Vector3d(-1, 0, 0));
      break;
    case LOOK_AT_XY_PLANE_FROM_MINUS_Z:
      camera_->SetCameraPlane(Vector3d(-1, 0, 0), Vector3d(0, 1, 0));
      break;
    case LOOK_AT_YX_PLANE_FROM_MINUS_Z:
      camera_->SetCameraPlane(Vector3d(0, 1, 0), Vector3d(1, 0, 0));
      break;
    case LOOK_AT_XZ_PLANE_FROM_PLUS_Y:
      camera_->SetCameraPlane(Vector3d(-1, 0, 0), Vector3d(0, 0, 1));
      break;
    case LOOK_AT_ZX_PLANE_FROM_PLUS_Y:
      camera_->SetCameraPlane(Vector3d(0, 0, 1), Vector3d(1, 0, 0));
      break;
    case LOOK_AT_XZ_PLANE_FROM_MINUS_Y:
      camera_->SetCameraPlane(Vector3d(1, 0, 0), Vector3d(0, 0, 1));
      break;
    case LOOK_AT_ZX_PLANE_FROM_MINUS_Y:
      camera_->SetCameraPlane(Vector3d(0, 0, 1), Vector3d(-1, 0, 0));
      break;
    case LOOK_AT_YZ_PLANE_FROM_PLUS_X:
      camera_->SetCameraPlane(Vector3d(0, 1, 0), Vector3d(0, 0, 1));
      break;
    case LOOK_AT_ZY_PLANE_FROM_PLUS_X:
      camera_->SetCameraPlane(Vector3d(0, 0, 1), Vector3d(0, -1, 0));
      break;
    case LOOK_AT_YZ_PLANE_FROM_MINUS_X:
      camera_->SetCameraPlane(Vector3d(0, -1, 0), Vector3d(0, 0, 1));
      break;
    case LOOK_AT_ZY_PLANE_FROM_MINUS_X:
      camera_->SetCameraPlane(Vector3d(0, 0, 1), Vector3d(0, 1, 0));
      break;
  }
  double bounds[6];
  GetBoundingBox(bounds);
  camera_->EnsureBoxVisible(bounds, GetAspectRatio());
  Redraw();
}

void GLViewerBase::PixelToModelCoords(int x, int y, Vector3d *model_pt) {
  // Find the model point at x,y. If no model point was found then project the
  // far-clip-plane point returned so that the depth is in the center of the
  // object's bounding volume. This will help the camera adjustment code do
  // sensible things.
  if (have_depth_buffer_) {
    MakeOpenGLContextCurrent();         // So reading the depth buffer can work

    // A depth buffer is available to compute the full tranformation.
    if (!gl::PixelToModelCoordinates(FramebufferObject(), x, y,
                                     gl::Transform(), model_pt)) {
      // x,y is at maximum depth (the far clip plane) so instead assume a depth
      // at the middle of the bounding box.
      double bounds[6];
      GetBoundingBox(bounds);
      Vector3d center((bounds[0] + bounds[1]) / 2.0,
                      (bounds[2] + bounds[3]) / 2.0,
                      (bounds[4] + bounds[5]) / 2.0);
      Vector3d ev = camera_->EyeVector();
      if (camera_->perspective) {
        Vector3d pos_to_farclip = *model_pt - camera_->pos;
        *model_pt = camera_->pos + (ev.dot(center - camera_->pos) /
                                    ev.dot(pos_to_farclip)) * pos_to_farclip;
      } else {
        *model_pt += ev * ev.dot(center - *model_pt);
      }
    }
  } else {
    // No depth buffer is available, choose a depth to get a model z coordinate
    // of zero.
    Vector3d m, dm_by_ddepth;
    gl::PixelToModelCoordinates(Vector3d(x, y, 0), gl::Transform(),
                                &m, &dm_by_ddepth);
    double alpha = -m[2] / dm_by_ddepth[2];
    *model_pt = m + alpha * dm_by_ddepth;
  }
}

bool GLViewerBase::ModelToPixelCoords(const Vector3d &model_pt,
                                      double *px, double *py) {
  MakeOpenGLContextCurrent();           // So reading the depth buffer can work
  int width, height;
  GetScaledClientSize(&width, &height);
  Vector3d v;
  gl::ModelToPixelCoordinates(model_pt, gl::Transform(), &v);
  *px = v[0];
  *py = v[1];
  if (v[0] >= 0 && v[0] <= width && v[1] >= 0 && v[1] <= height && v[2] > 0) {
    return true;
  }
  return false;
}

void GLViewerBase::HandleClick(int x, int y, bool button,
                               const Vector3d &model_pt) {
}

void GLViewerBase::HandleDrag(int x, int y, const Vector3d &model_pt) {
}

void GLViewerBase::FindModelPoint(int x, int y, Eigen::Vector3d *model_pt) {
  PixelToModelCoords(x, y, model_pt);
}

void GLViewerBase::ApplyViewport() {
  int window_width, window_height;
  GetScaledClientSize(&window_width, &window_height);
  GL(Viewport)(0, 0, window_width, window_height);
}

void GLViewerBase::ApplyCameraTransformations(const Matrix4d &M) {
  double bounds[6];
  GetBoundingBox(bounds);
  gl::ApplyTransform(M * camera_->Projection(bounds, GetAspectRatio()),
                     camera_->ModelView());
}

double GLViewerBase::GetAspectRatio() {
  int width, height;
  GetScaledClientSize(&width, &height);
  return double(width) / double(height);
}

uint32_t GLViewerBase::FramebufferObject() {
  return 0;
}

//***************************************************************************
// GLViewer for Qt.

#ifdef QT_CORE_LIB

#include "QMouseEvent"
#include "gl_utils.h"

GLViewer::GLViewer(QWidget *parent) : QOpenGLWidget(parent) {
  have_depth_buffer_ = (QSurfaceFormat::defaultFormat().depthBufferSize() > 0);

  // Set up event handling.
  setMouseTracking(true);
  grabGesture(Qt::PinchGesture);
}

void GLViewer::mouseDoubleClickEvent(QMouseEvent *event) {
  MouseEvent(event, false, "d click");
}

void GLViewer::mouseMoveEvent(QMouseEvent *event) {
  MouseEvent(event, true, "move   ");
}

void GLViewer::mousePressEvent(QMouseEvent *event) {
  MouseEvent(event, false, "press  ");
}

void GLViewer::mouseReleaseEvent(QMouseEvent *event) {
  MouseEvent(event, false, "release");
}

void GLViewer::wheelEvent(QWheelEvent *event) {
  makeCurrent();                // So reading the depth buffer can work

  double scale = devicePixelRatio();    // Usually 1 or 2
  FindModelPoint(last_x_, last_y_, &model_pt_);

  if (event->source() == Qt::MouseEventNotSynthesized) {
    // The event comes from an actual wheel on an actual mouse. Use this for
    // zooming.
    double r = event->angleDelta().y() * 0.1;
    camera_->Zoom(pow(2, -r * 0.02), model_pt_);
  } else {
    // The event probably comes from a scroll gesture on a touchpad. Use this
    // for panning.
    int dx = event->pixelDelta().x();
    int dy = event->pixelDelta().y();
    camera_->Pan(-dx*2, dy*2, model_pt_, width() * scale);
  }
  update();
}

void GLViewer::leaveEvent(QEvent *event) {
  // If we are not dragging and the mouse leaves the window, set the "last
  // model coordinate" to the center of the window, so that zoom in and out
  // will operate with respect to the window center.
  if (buttons_ == 0) {
    double scale = devicePixelRatio();    // Usually 1 or 2
    last_x_ = width() * scale / 2;
    last_y_ = height() * scale / 2;
  }

  QOpenGLWidget::leaveEvent(event);
}

void GLViewer::gestureEvent(QGestureEvent *event) {
  // OS X pinch-to-zoom events.
  if (QGesture *gesture = event->gesture(Qt::PinchGesture)) {
    QPinchGesture *pinch = static_cast<QPinchGesture *>(gesture);
    double scale = pinch->scaleFactor();

    makeCurrent();                // So reading the depth buffer can work

    FindModelPoint(last_x_, last_y_, &model_pt_);
    camera_->Zoom(1.0 / scale, model_pt_);
    update();
  }
}

bool GLViewer::event(QEvent *event)  {
  if (event->type() == QEvent::Gesture) {
    gestureEvent(static_cast<QGestureEvent*>(event));
    return true;
  }
  return QOpenGLWidget::event(event);
}

void GLViewer::MouseEvent(QMouseEvent *event, bool move_event,
                          const char *name) {
  makeCurrent();                // So reading the depth buffer can work

  // Multiple buttons held down at once have no special UI meaning here. To
  // prevent confusion for the user (and the code below), once a button is
  // pressed we ignore the other buttons until all buttons are released. This
  // prevents us from having to specially handle tricky cases like LeftDown ->
  // RightDown -> LeftUp -> RightUp, which could otherwise easily result in
  // inconsistent UI state.
  int last_the_button = the_button_;
  if (buttons_ == 0 && event->buttons()) {
    // No buttons were pressed, then one button was pressed, this will become
    // "the button" until all buttons are released.
    the_button_ = event->buttons();   // Will be 1, 2 or 4
  } else if (event->buttons() == 0) {
    the_button_ = 0;
  }
  buttons_ = event->buttons();

  // The bits in event->modifiers() depend on the OS:
  //
  //   Constant          | Mac key   | Windows key | Bootcamp key |
  // --------------------+-----------+-------------+--------------+
  // Qt::ShiftModifier   |  Shift    |  Shift      |  Shift       |
  // Qt::ControlModifier |  Command  |  Control    |  Control     |
  // Qt::AltModifier     |  Option   |  Alt        |  Option      |
  // Qt::MetaModifier    |  Control  |  Windows    |  -           |
  // --------------------+-----------+-------------+--------------+
  //
  // The Mac key order:     control option  command
  // The Windows key order: control windows alt
  // We don't want to use the windows key for things since it does other
  // things in Windows, so we'll ignore the mac option key too for consistency.
  //
  // On the mac trackpad, control + single (or double) finger tap is translated
  // into control + right.

#ifdef __APPLE__
  const int kZoomBit = Qt::MetaModifier;
  const int kRotateBit = Qt::ControlModifier;
  const int kPanBit = Qt::MetaModifier;
#else
  const int kZoomBit = Qt::ControlModifier;
  const int kRotateBit = Qt::AltModifier;
  const int kPanBit = Qt::ControlModifier;
#endif

  // For debugging:
  // printf("Modifiers: s=%d c=%d a=%d m=%d  Buttons: %d\n",
  //     int(event->modifiers() & Qt::ShiftModifier) != 0,
  //     int(event->modifiers() & Qt::ControlModifier) != 0,
  //     int(event->modifiers() & Qt::AltModifier) != 0,
  //     int(event->modifiers() & Qt::MetaModifier) != 0,
  //     (int) event->buttons());

  // Get window properties.
  double scale = devicePixelRatio();    // Usually 1 or 2

  // Get mouse position in opengl viewport coordinates (Y inverted from window
  // coordinates).
  // @@@ We can possibly get the cursor position to native resolution here.
  int x = event->x() * scale;
  int y = (height() - 1 - event->y()) * scale;

  // Compute the delta position since the last mouse event.
  int dx = x - last_x_;
  int dy = y - last_y_;

  // Handle events.
  if (last_the_button == 0 && the_button_) {
    setFocus();

    // Find the model point that was clicked on.
    FindModelPoint(x, y, &model_pt_);

    // Handle unmodified left button clicks in subclass code. Modified left or
    // right clicks start a rotation or pan.
    rotate_click_ = false;
    pan_click_ = false;
    if (the_button_ == Qt::LeftButton) {
      if (event->modifiers() == kRotateBit) {
        rotate_click_ = true;
      } else if (event->modifiers() == kPanBit) {
        pan_click_ = true;
      } else {
        HandleClick(x, y, true, model_pt_);
      }
    }
    if (the_button_ == Qt::RightButton && event->modifiers() == kPanBit) {
      pan_click_ = true;
    }
    last_x_ = x;
    last_y_ = y;
  }
  else if (last_the_button == Qt::LeftButton && the_button_ == 0 &&
          !rotate_click_ && !pan_click_) {
    HandleClick(x, y, false, model_pt_);
  }
  else if (move_event && buttons_ != 0) {
    if (the_button_ == Qt::LeftButton && event->modifiers() == 0) {
      HandleDrag(x, y, model_pt_);
    } else if (the_button_ == Qt::MidButton ||
               (the_button_ == Qt::LeftButton &&
                event->modifiers() == kRotateBit)) {
      // Rotating.
      if (allow_rotation_) {
        camera_->Trackball(dx / 100.0, dy / 100.0, model_pt_);
        update();
      }
    } else if ((the_button_ == Qt::RightButton && event->modifiers() == 0) ||
               (the_button_ == Qt::LeftButton &&
                event->modifiers() == kPanBit)) {
      // Panning.
      camera_->Pan(dx, dy, model_pt_, width() * scale);
      update();
    } else if (the_button_ == Qt::RightButton &&
               event->modifiers() == kZoomBit) {
      // Zooming.
      camera_->Zoom(pow(2, -dy / 50.0), model_pt_);
      update();
    }
    last_x_ = x;
    last_y_ = y;
  }
  else if (move_event && buttons_ == 0) {
    if (rotate_click_) {
      // Rotating.
      if (event->modifiers() == kRotateBit) {
        if (allow_rotation_) {
          camera_->Trackball(dx / 100.0, dy / 100.0, model_pt_);
          update();
        }
      } else {
        rotate_click_ = false;
      }
    } else if (pan_click_) {
      // Panning.
      if (event->modifiers() == kPanBit) {
        camera_->Pan(dx, dy, model_pt_, width() * scale);
        update();
      } else {
        pan_click_ = false;
      }
    }
    last_x_ = x;
    last_y_ = y;
  }
}

void GLViewer::Redraw() {
  update();
}

void GLViewer::MakeOpenGLContextCurrent() {
  makeCurrent();
}

void GLViewer::GetScaledClientSize(int *window_width, int *window_height) {
  const qreal retinaScale = devicePixelRatio();
  *window_width = width() * retinaScale;
  *window_height = height() * retinaScale;
}

uint32_t GLViewer::FramebufferObject() {
  return defaultFramebufferObject();
}

void GLViewer::initializeGL() {
  gl::InitializeOpenGLFunctions(context());
}

void GLViewer::resizeGL(int w, int h) {
}

void GLViewer::paintGL() {
  // Nothing to do if the window is not visible.
  if(!isVisible()) {
    return;
  }

  // Reset the opengl error state in case the caller did something bad. We want
  // to trap errors that our own code generates and not worry about any bad
  // things the gui library did.
  GL(GetError)();

  // Reset GL state.
  ApplyViewport();

  // Draw everything.
  Draw();

  // Complain if there were OpenGL errors.
  int err;
  while ((err = GL(GetError)()) != GL_NO_ERROR) {
    Error("GL error %d (%s)", err, gl::ErrorString(err));
  }
}

#endif  // QT_CORE_LIB
