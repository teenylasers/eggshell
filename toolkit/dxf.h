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

// Tools for handling export of 2D shapes to DXF files.

#ifndef __TOOLKIT_DXF_H__
#define __TOOLKIT_DXF_H__

#include <stdio.h>
#include "Eigen/Dense"

namespace dxf {

typedef Eigen::Vector2d Point;

// Given three points, fit a circle to them and return the center and radius.

void FitCircleTo3Points(const Point &p1, const Point &p2, const Point &p3,
                        double *radius, Point *center);

// Given an array of points, represent the points as a series of line segments
// and arcs. Groups of points only qualify to be represented as an arc if there
// are 4 or more, they are less than arc_dist apart in distance and arc_angle
// apart in angle, and (obviously) they all lie on the same circle. The
// arc_angle argument is in degrees.
//
// If a group of points makes an arc that wraps around on itself multiple
// times, only a single arc will be emitted.
//
// The arc_tolerance is the distance within which fitted centers and radii must
// lie for groups of points to be considered part of the same arc.

struct LineOrArc {
  Point start, end;             // Start and end of line or arc
  Point center;                 // Center if arc or circle
  // The type of geometry. Arcs can go clockwise or counterclockwise from the
  // starting point (ARC_CW, ARC_CCW). The radius of the circle is the center
  // point to the start point (the end point is ignored).
  enum { LINE, ARC_CW, ARC_CCW, CIRCLE } type;
};

void ArcifyPointList(const std::vector<Point> &p,
                     double arc_dist, double arc_angle, double arc_tolerance,
                     std::vector<LineOrArc> *result);

// Write geometry to fout in DXF format, given a list of closed shapes in 'p'.
// Arcs are created where they match the criteria in ArcifyPointList(), given
// arc_dist, arc_angle, and arc_tolerance.

void WriteDXF(const std::vector<std::vector<Point>> &p,
              double arc_dist, double arc_angle, double arc_tolerance,
              FILE *fout);

}  // namespace dxf

#endif
