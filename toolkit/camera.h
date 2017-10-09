
#ifndef __TOOLKIT_CAMERA_H__
#define __TOOLKIT_CAMERA_H__

#include "Eigen/Dense"
#include "Eigen/Geometry"

struct Camera {
  Eigen::Vector3d pos;   // Camera position, in model coordinates
  Eigen::Quaterniond q;  // Camera rotation. Identity looks at -Z
  bool perspective;      // Perspective or orthographic mode
  double scale;          // In orthographic mode, horizontal GL units visible
  double view_angle;     // In perspective mode, horizontal view angle (degrees)

  Camera();

  // Get the projection matrix for this camera, which transforms from
  // "eye-looks-from-origin-to-minus-z" homogeneous coordinates to window and
  // depth coordinates. The bounding box of all drawable objects in model
  // coordinates are given and are used to set the near and far clip planes for
  // maximum depth buffer resolution. The bounding box is [xmin, xmax, ymin,
  // ymax, zmin, zmax].
  Eigen::Matrix4d Projection(const double bounds[6], double aspect_ratio) const;

  // Get the model-view matrix for this camera, that rotates and translates the
  // model to "eye-looks-from-origin-to-minus-z".
  Eigen::Matrix4d ModelView() const;

  // Pan the camera so that the model coordinates 'c' move by dx,dy pixels in
  // the viewport. In orthographic mode 'c' is ignored.
  void Pan(int dx, int dy, const Eigen::Vector3d &c, int window_width);

  // Rotate the camera by angles ax,ay (in radians) around the viewport axes,
  // while keeping model coordinates 'c' at the same position in the viewport.
  void Trackball(double ax, double ay, const Eigen::Vector3d &c);

  // Zoom the camera towards model coordinates 'c' by scale factor 's'. In
  // orthographic mode the component of 'c' along the eye vector is ignored.
  void Zoom(double s, const Eigen::Vector3d &c);

  // Return the eye vector, i.e. that points in the direction of the camera.
  Eigen::Vector3d EyeVector() const;

  // Adjust position only (and for orthographic mode, scale) to ensure that the
  // given box [xmin, xmax, ymin, ymax, zmin, zmax] is visible. The aspect
  // ratio of the viewport (width / height) is 'aspect'.
  void EnsureBoxVisible(const double box[6], double aspect);

  // Set the camera orientation so that the viewport x and y axes are the given
  // unit vectors.
  void SetCameraPlane(const Eigen::Vector3d &vx, const Eigen::Vector3d &vy);
};

#endif
