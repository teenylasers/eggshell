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

// @@@ TODO
// In orthographic mode we can adjust the camera position along the eye vector
// without visible change. When we switch back to perspective mode the new
// camera position may cause the model to be too small or even behind the
// viewer.

#include "camera.h"
#include "gl_utils.h"

using namespace Eigen;

// We must set the near clip in front of the camera, along the eye vector. If
// objects are very close to or behind the camera then we must make an
// arbitrary choice about which parts of the scene to clip out. This constant
// scales the bounding box size (along the direction of the eye vector) to
// position the near clip plane. A too-large value clips out too much of the
// scene, a too-small values causes precision problems in the depth buffer.
static const double kNearLimitScale = 0.01;

Camera::Camera() {
  pos.setZero();
  pos(2) = 10;
  q.setIdentity();
  perspective = true;
  scale = 4;
  view_angle = 45;
}

Matrix4d Camera::Projection(const double bounds[6], double aspect_ratio) const {
  // Compute distance to near and far clip planes.
  Vector3d ev = EyeVector();
  double zmin, zmax;
  gl::ProjectBox(ev, bounds, &zmin, &zmax);
  double span = zmax - zmin;
  if (span <= 0) {              // Probably nothing to render so just ensure
    span = 1;                   // frustum near and far are valid
  }
  double frustum_near = zmin - pos.dot(ev);
  double frustum_far  = zmax - pos.dot(ev);
  frustum_near -= span * 0.01;  // Expand near-to-far span to allow for
  frustum_far  += span * 0.01;  // bounding box errors and GL depth precision
  if (perspective) {
    // Make sure both clipping planes are in front of the camera.
    double near_limit = span * kNearLimitScale;
    frustum_near = std::max(frustum_near, near_limit);
    if (frustum_far <= frustum_near) {
      // Everything is behind the viewer, so just ensure near and far are valid.
      frustum_near = 1;
      frustum_far  = 2;
    }
  }

  // Set viewport projection.
  if (perspective) {
    // tva = view scale, 1 = +/- 45 degrees
    const double tva = tan(view_angle * 0.5 * M_PI/180.0);
    double frustum_width = 2.0 * frustum_near * tva;
    double frustum_height = 2.0 * frustum_near * tva / aspect_ratio;
    return gl::PerspectiveProjection(frustum_width, frustum_height,
                                     frustum_near, frustum_far);
  } else {
    return gl::OrthographicProjection(scale, scale / aspect_ratio,
                                      frustum_near, frustum_far);
  }
}

Matrix4d Camera::ModelView() const {
  Matrix4d M;
  Matrix3d R = q.toRotationMatrix();
  M.block(0, 0, 3, 3) = R;
  M.block(0, 3, 3, 1) = -R * pos;
  M.block(3, 0, 1, 4) << 0, 0, 0, 1;
  return M;
}

void Camera::Pan(int dx, int dy, const Vector3d &c, int window_width) {
  Matrix3d Rq = q.toRotationMatrix();
  double k = 0;
  if (perspective) {
    double tva = tan(view_angle * 0.5 * M_PI/180.0);
    double depth = Rq.row(2).dot(c - pos);
    k = 2.0 * depth * tva / window_width;
  } else {
    k = -scale / window_width;
  }
  pos += dx * k * Rq.row(0);
  pos += dy * k * Rq.row(1);
}

void Camera::Trackball(double ax, double ay, const Vector3d &c) {
  Matrix3d Rq = q.toRotationMatrix();
  AngleAxisd r1(ax, Rq.row(1));
  AngleAxisd r2(-ay, Rq.row(0));
  Matrix3d R = r1.toRotationMatrix() * r2.toRotationMatrix();
  Matrix3d new_R = Rq * R;
  Vector3d new_pos = R.transpose() * (pos - c) + c;
  pos = new_pos;
  q = new_R;
  q.normalize();
}

void Camera::Zoom(double s, const Vector3d &c) {
  if (perspective) {
    // Move camera position along the vector to c.
    pos = c + s * (pos - c);
  } else {
    // We adjust the scale and the position normal to the eye vector to make
    // sure that the projection of 'c' into the viewport is unchanged.
    Matrix3d Rq = q.toRotationMatrix();
    double px = (1.0 - s) * Rq.row(0).dot(c - pos);
    double py = (1.0 - s) * Rq.row(1).dot(c - pos);
    pos += px * Rq.row(0) + py * Rq.row(1);
    scale *= s;
  }
}

Vector3d Camera::EyeVector() const {
  Matrix3d Rq = q.toRotationMatrix();
  return -Rq.row(2);            // We look along -Z
}

void Camera::EnsureBoxVisible(const double box[6], double aspect) {
  Matrix3d Rq = q.toRotationMatrix();
  double xmin, xmax, ymin, ymax, zmin, zmax;
  gl::ProjectBox(Rq.row(0), box, &xmin, &xmax);
  gl::ProjectBox(Rq.row(1), box, &ymin, &ymax);
  gl::ProjectBox(Rq.row(2), box, &zmin, &zmax);
  scale = std::max(xmax - xmin, aspect * (ymax - ymin)) * 1.1;
  double d = 0.5 * scale / tan(view_angle * 0.5 * M_PI/180.0);
  Vector3d new_pos = Rq.row(0) * 0.5 * (xmin + xmax) +
                     Rq.row(1) * 0.5 * (ymin + ymax) +
                     Rq.row(2) * (zmax + d);
  pos = new_pos;
}

void Camera::SetCameraPlane(const Vector3d &vx, const Vector3d &vy) {
  Matrix3d R;
  Vector3d vx2 = vx.normalized();
  Vector3d vy2 = vy.normalized();
  vy2 = vy2 - (vx2.dot(vy2))*vx2;       // Orthogonalize vx2 to vy2
  R.row(0) = vx2;
  R.row(1) = vy2;
  R.row(2) = vx2.cross(vy2);
  q = R;
}
