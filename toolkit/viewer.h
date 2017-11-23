
#ifndef __TOOLKIT_VIEWER_H__
#define __TOOLKIT_VIEWER_H__

#include "camera.h"
#include "error.h"

// An opengl window that maintains camera state and allows camera motion via
// mouse and key commands:
//
//   * Pan: Right button drag, or on a mac trackpad use cmd + two-finger swipe.
//   * Zoom: Scroll wheel, or ctrl + right button drag, or on a mac use a
//     two-finger swipe.
//   * Rotate: Middle button drag, or on mac use cmd + left click to start a
//     rotation, continue holding down cmd and move the mouse to complete the
//     rotation.

class GLViewerBase {
 public:
  GLViewerBase();

  // The maximum number of independently movable cameras.
  enum { MAX_CAMERAS = 10 };

  // Return the current camera, and its index for SwitchCamera().
  Camera &GetCamera() const { return *camera_; }
  int GetCameraIndex() const { return camera_ - cameras_; };

  // Set/get the perspective flag for the current camera. True means a
  // perspective projection, false means an orthographic projection. The
  // default is perspective.
  void SetPerspective(bool perspective);
  bool GetPerspective() const { return camera_->perspective; }

  // Set the perspective view angle for the current camera. The default is 45
  // degrees.
  void SetViewAngle(double angle);

  // Allow or disallow camera rotation. Disallowing it is useful in
  // orthographic mode when dealing with strictly 2D content. The default is
  // allowed.
  void AllowCameraRotation(bool allow) { allow_rotation_ = allow; }

  // Ensure that the entire bounding box defined by GetBoundingBox() is
  // visible. This maintains the current camera orientation.
  void ZoomExtents();

  // Zoom in or out, enlarging the view around the center point by the given
  // scale factor. A scale factor less than 1 zooms in. If 'center' is given it
  // is used as the center point, otherwise the center point is read from the
  // current GL window size and depth buffer.
  void Zoom(double scale_factor, Eigen::Vector3d center);
  void Zoom(double scale_factor);

  // Switch to the given camera (0..MAX_CAMERAS-1).
  void SwitchCamera(int camera_number);

  // Adjust the camera direction as specified and ensure that the entire
  // bounding box defined by GetBoundingBox() is visible.
  enum Direction {
    LOOK_AT_XY_PLANE_FROM_PLUS_Z,  LOOK_AT_YX_PLANE_FROM_PLUS_Z,
    LOOK_AT_XY_PLANE_FROM_MINUS_Z, LOOK_AT_YX_PLANE_FROM_MINUS_Z,
    LOOK_AT_XZ_PLANE_FROM_PLUS_Y,  LOOK_AT_ZX_PLANE_FROM_PLUS_Y,
    LOOK_AT_XZ_PLANE_FROM_MINUS_Y, LOOK_AT_ZX_PLANE_FROM_MINUS_Y,
    LOOK_AT_YZ_PLANE_FROM_PLUS_X,  LOOK_AT_ZY_PLANE_FROM_PLUS_X,
    LOOK_AT_YZ_PLANE_FROM_MINUS_X, LOOK_AT_ZY_PLANE_FROM_MINUS_X,
  };
  void Look(Direction direction);

  // Convert window pixel coordinates to/from model coordinates. If no depth
  // buffer is available then PixelToModelCoords() chooses a depth to get a
  // model z coordinate of zero. This generally only makes sense with an
  // orthographic camera. ModelToPixelCoords() returns false if the resulting
  // pixel/depth is outside the view frustum, but in that case it will still
  // try to return useful coordinates.
  void PixelToModelCoords(int x, int y, Eigen::Vector3d *model_pt);
  bool ModelToPixelCoords(const Eigen::Vector3d &model_pt,
                          double *px, double *py);

  // ********** Virtual functions supplied in subclasses. Non-abstract
  //            functions have default implementations that do nothing.

  // Draw everything into the current GL context. 2D objects render themselves
  // in the Z=0 plane but are otherwise indistinguishable from 3D objects.
  virtual void Draw() = 0;

  // Call this from Draw() to apply the camera transformation M * projection *
  // model-view to the current program.
  void ApplyCameraTransformations(const Eigen::Matrix4d &M =
                                  Eigen::Matrix4d::Identity());

  // Return the bounding box of the entire scene so that e.g. ZoomExtents()
  // knows what region to display. The array elements are [xmin, xmax, ymin,
  // ymax, zmin, zmax].
  virtual void GetBoundingBox(double bounds[6]) = 0;

  // Handle mouse events at x,y (in opengl viewport coordinates). If button is
  // true the mouse button was pressed, otherwise it was released. The model_pt
  // is the model point corresponding to the mouse location at the last button
  // press or scroll wheel motion. All camera-moving mouse actions will be
  // filtered out.
  virtual void HandleClick(int x, int y, bool button,
                           const Eigen::Vector3d &model_pt);
  virtual void HandleDrag(int x, int y, const Eigen::Vector3d &model_pt);

  // Trigger a windowing system redraw of the window.
  virtual void Redraw() = 0;

  // Allow use of OpenGL outside of painting functions.
  virtual void MakeOpenGLContextCurrent() = 0;

  // Return the OpenGL viewport size in raw pixels not points (e.g. on retina
  // displays each point can be 2x2 pixels).
  virtual void GetScaledClientSize(int *window_width, int *window_height) = 0;

 protected:
  // Drawing state.
  bool have_depth_buffer_;              // Property of current GL context
  Camera cameras_[MAX_CAMERAS];         // All cameras
  Camera *camera_;                      // Current camera, points into cameras_
  bool allow_rotation_;                 // Allow camera rotation
  int last_x_, last_y_;                 // Last mouse position
  int buttons_;                         // Mask of currently pressed buttons
  int the_button_;                      // The button that is UI-active
  Eigen::Vector3d model_pt_;            // Model coords at start of drag
  bool cmd_click_;                      // Cmd + left click mode on a mac?

  // Get the current window aspect ratio (width/height).
  double GetAspectRatio();

  // Apply the viewport transformation.
  void ApplyViewport();
};

//***************************************************************************
// wxWidgets implementation.

#ifdef __TOOLKIT_WXWINDOWS__

#include "stdwx.h"
#include "gl_utils.h"
#include "color_based_selection.h"

class GLViewer : public wxGLCanvas, public GLViewerBase {
 public:
  // gl_type is an argument to gl::GetAttributeList().
  typedef wxWindow ParentType;
  GLViewer(ParentType* parent, int gl_type);
  ~GLViewer();

 private:
  // Drawing state.
  wxGLContext *context_;                // OpenGL context

  // WX event handling and other functionality.
  void OnPaint(wxPaintEvent &event);
  void OnSize(wxSizeEvent &evt);
  void OnEraseBackground(wxEraseEvent &event);
  void OnMouseEvent(wxMouseEvent &event);
  void OnCaptureLost(wxMouseCaptureLostEvent &event);
  void Redraw();
  void MakeOpenGLContextCurrent() override;
  // Return the client size scaled by GetContentScaleFactor().
  void GetScaledClientSize(int *window_width, int *window_height);

  friend class GLViewerWithSelection;
  DECLARE_EVENT_TABLE()
};

class GLViewerWithSelection : public GLViewer {
 public:
  GLViewerWithSelection(wxWindow* parent, int gl_type);
  ~GLViewerWithSelection();

 protected:
  // Find the object that intersects x,y in the frame buffer, and return its
  // index. Return -1 if none intersects. The bottom left of the window is
  // x,y=0,0.
  int FindObject(int x, int y);

  // We are selecting an object. Draw all objects into the current GL context
  // Only draw the object geometry using the selection colors, don't alter the
  // texture (and etc.) state. Drawing may be optimized by only drawing objects
  // that intersect the much smaller selection frustum.
  virtual void DrawForSelection(const gl::ColorBasedSelection &sel) = 0;
};

#endif  // __TOOLKIT_WXWINDOWS__

//***************************************************************************
// Qt implementation.

#ifdef QT_CORE_LIB

#include "qopenglwidget.h"
#include <QGestureEvent>

class GLViewer : public QOpenGLWidget, public GLViewerBase {
 public:
  typedef QWidget ParentType;
  explicit GLViewer(ParentType *parent);

  // Event handling functions.
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  void leaveEvent(QEvent *event) override;
  void gestureEvent(QGestureEvent *event);
  bool event(QEvent *event) override;
  void MouseEvent(QMouseEvent *event, bool move_event, const char *name);

  //  Overridden virtual functions of GLViewerBase.
  void Redraw() override;
  void MakeOpenGLContextCurrent() override;
  void GetScaledClientSize(int *window_width, int *window_height) override;

 private:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
};

#endif  // QT_CORE_LIB

#endif
