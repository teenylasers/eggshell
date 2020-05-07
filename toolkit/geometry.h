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

#ifndef __TOOLKIT_GEOMETRY_H__
#define __TOOLKIT_GEOMETRY_H__

#include "Eigen/Dense"

namespace geom {

using Eigen::Vector2d;

// Compute the cross product of two 2D vectors.
static inline double Cross2(const Vector2d &a, const Vector2d &b) {
  return a[0] * b[1] - a[1] * b[0];
}

// Return true if two 2D lines 'a' and 'b' intersect and are not parallel.
// Return the intersection point if 'intersection' is not 0.
inline bool LineIntersection(const Vector2d &a1, const Vector2d &a2,
                             const Vector2d &b1, const Vector2d &b2,
                             Vector2d *intersection) {
  // Express the lines as p1=a1+s*dira, p2=b1+t*dirb, then solve for s,t.
  Vector2d dira = a2 - a1;
  Vector2d dirb = b2 - b1;
  double det = Cross2(dira, dirb);
  if (det == 0) {
    return false;               // Parallel lines
  }
  Vector2d delta = a1 - b1;
  double s = Cross2(dirb, delta) / det;
  double t = Cross2(dira, delta) / det;
  if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
    if (intersection) {
      *intersection = a1 + s*dira;
    }
    return true;
  }
  return false;
}

}  // namespace geom

#endif
