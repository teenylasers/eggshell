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

#include "dxf.h"
#include "error.h"
#include "testing.h"
#include "mystring.h"
#include "myvector"
#include "random.h"

#include <stdlib.h>
#include <math.h>
#include <string>
#include <algorithm>

using std::vector;
using std::string;

namespace dxf {

static inline double Cross2(const Point &a, const Point &b) {
  return a[0] * b[1] - a[1] * b[0];
}

void FitCircleTo3Points(const Point &p1, const Point &p2, const Point &p3,
                        double *radius, Point *center) {
  // Solve |p1-center|^2 == |p2-center|^2 and |p1-center|^2 == |p3-center|^2.
  Point m1 = p2 - p1;
  Point m2 = p3 - p1;
  // If the determinant (which is also the cross product of the m vectors) is
  // zero then the points are colinear and the center is infinitely far away.
  double determinant = m1[0]*m2[1] - m1[1]*m2[0];
  double q = p1.squaredNorm();
  double b1 = p2.squaredNorm() - q;
  double b2 = p3.squaredNorm() - q;
  (*center)[0] = 0.5*(b1*m2[1] - b2*m1[1]) / determinant;
  (*center)[1] = 0.5*(b2*m1[0] - b1*m2[0]) / determinant;
  *radius = (*center - p1).norm();
}

void ArcifyPointList(const vector<Point> &p, double arc_dist, double arc_angle,
                     double arc_tolerance, vector<LineOrArc> *result) {
  result->clear();

  // Convert the cosine of arc_angle.
  double caa = cos(arc_angle * M_PI / 180.0);

  // For every point, compute the circle that fits it and its two neighbors.
  const int n = p.size();
  vector<Point> center(n);
  vector<double> radius(n);
  for (int i = 0; i < n; i++) {
    FitCircleTo3Points(p[(i + n-1) % n], p[i], p[(i+1) % n],
                       &radius[i], &center[i]);
  }

  // Segment i is the path between point i and i+1. Identify all segments that
  // are arc-compatible, i.e. that are less than arc_dist/arc_angle from all
  // neighbors and whose bounding points have the same center and radius. Only
  // do this if we have at least 4 points.
  vector<bool> arc(n);  // arc[i] true if p[i] and p[i+1] are arc-compatible
  if (n >= 4) {
    for (int i1 = 0; i1 < n; i1++) {
      int i0 = (i1 + n-1) % n, i2 = (i1 + 1) % n, i3 = (i1 + 2) % n;
      const Point &c = center[i1];
      arc[i1] = (c - center[i2]).norm() < arc_tolerance &&
            // This radius check is probably superfluous:
            fabs(radius[i1] - radius[i2]) < arc_tolerance &&
            (p[i0] - p[i1]).norm() < arc_dist &&
            (p[i1] - p[i2]).norm() < arc_dist &&
            (p[i2] - p[i3]).norm() < arc_dist &&
            (p[i0] - c).normalized().dot((p[i1] - c).normalized()) > caa &&
            (p[i1] - c).normalized().dot((p[i2] - c).normalized()) > caa &&
            (p[i2] - c).normalized().dot((p[i3] - c).normalized()) > caa;
    }
  }

  // Deal with the special case where all points are on the same circle.
  bool is_circle = true, is_all_lines = true;
  for (int i = 0; i < arc.size(); i++) {
    is_circle &= arc[i];
    is_all_lines &= !arc[i];
  }
  if (is_circle) {
    result->resize(1);
    result->back().start = p[0];
    result->back().end.setZero();
    result->back().center = center[0];
    result->back().type = LineOrArc::CIRCLE;
    return;
  }

  // Now we know that at least one of the segments is a line. To make the
  // remaining code simpler, if there is at least one arc we find the first arc
  // group and rotate things so that it is first. This means we won't have arcs
  // that span the end->start of the list.
  int ofs = -1;
  if (is_all_lines) {
    ofs = 0;
  } else {
    for (int i = 0; i < n; i++) {
      if (!arc[i] && arc[(i + 1) % n]) {
        ofs = i;
        break;
      }
    }
    CHECK(ofs >= 0 && ofs < n);
  }

  // For all groups of one or more segments, emit a LineOrArc.
  for (int ii = 0; ii < n; ii++) {      // ii is offset from 'ofs'
    int i0 = (ii + ofs) % n;
    int i1 = (i0 + 1) % n;
    // Detect groups of one or more points that form an arc.
    if (!arc[i0] && arc[i1]) {
      // This is an arc, let's see how many segments are involved.
      int count = 0;
      while (count < n && arc[(i1 + count) % n]) {
        count++;
      }
      const Point &start = p[i0];
      const Point &end = p[(i1 + count + 1) % n];
      Point &c = center[i1];            // Center of the arc
      CHECK(fabs((start - c).norm() - (end - c).norm()) < arc_tolerance);
      result->resize(result->size() + 1);
      result->back().start = start;
      result->back().end = end;
      result->back().center = c;
      double winding_angle = 0;         // Compute total angle traversed by arc
      for (int k = 0; k < count + 2; k++) {  // for each piece of the arc
        double sin_angle = Cross2((p[(i0 + k) %n] - c).normalized(),
                                  (p[(i0 + k + 1) %n] - c).normalized());
        winding_angle += asin(std::min(1.0, std::max(-1.0, sin_angle)));
      }
      CHECK(fabs( cos(winding_angle) -
                  ((start - c).normalized()).dot((end - c).normalized()) )
              < 1e-9);
      result->back().type = (winding_angle >= 0) ?
                            LineOrArc::ARC_CCW : LineOrArc::ARC_CW;
      ii += count + 1;
      CHECK(ii < n + 1);        // Make sure we didn't jump too far
    } else {
      // This is a line segment.
      result->resize(result->size() + 1);
      result->back().start = p[i0];
      result->back().end = p[i1];
      result->back().center.setZero();
      result->back().type = LineOrArc::LINE;
    }
  }
}

void WriteDXF(const vector<vector<Point>> &p,
              double arc_dist, double arc_angle, double arc_tolerance,
              FILE *fout) {
  fprintf(fout, "0\nSECTION\n2\nENTITIES\n");
  for (int i = 0; i < p.size(); i++) {
    vector<LineOrArc> q;
    ArcifyPointList(p[i], arc_dist, arc_angle, arc_tolerance, &q);
    for (int i = 0; i < q.size(); i++) {
      if (q[i].type == LineOrArc::LINE) {
        fprintf(fout, "0\nLINE\n5\n0\n100\nAcDbEntity\n8\nCavity\n"
                      "100\nAcDbLine\n");
        fprintf(fout, "10\n%.15e\n20\n%.15e\n30\n0\n",
                q[i].start[0], q[i].start[1]);
        fprintf(fout, "11\n%.15e\n21\n%.15e\n31\n0\n",
                q[i].end[0], q[i].end[1]);
      } else {
        // Arc or circle.
        double start_angle = atan2(q[i].start[1] - q[i].center[1],
                                   q[i].start[0] - q[i].center[0]);
        double end_angle = atan2(q[i].end[1] - q[i].center[1],
                                 q[i].end[0] - q[i].center[0]);
        fprintf(fout, "0\n%s\n5\n0\n100\nAcDbEntity\n8\nCavity\n"
                      "100\nAcDbCircle\n",
                      (q[i].type == LineOrArc::CIRCLE) ? "CIRCLE" : "ARC");
        fprintf(fout, "10\n%.15e\n20\n%.15e\n30\n0\n40\n%.15e\n",
                q[i].center[0], q[i].center[1],
                (q[i].start - q[i].center).norm());
        if (q[i].type != LineOrArc::CIRCLE) {
          if (q[i].type == LineOrArc::ARC_CCW) {
            while (end_angle < start_angle) end_angle += 2.0 * M_PI;
          } else {
            double tmp = start_angle;
            start_angle = end_angle;
            end_angle = tmp;
            while (end_angle < start_angle) end_angle += 2.0 * M_PI;
          }
          fprintf(fout, "100\nAcDbArc\n");
          fprintf(fout, "50\n%.15e\n51\n%.15e\n",
                  start_angle * 180.0 / M_PI, end_angle * 180.0 / M_PI);
        }
      }

    }
  }
  fprintf(fout, "0\nENDSEC\n0\nEOF\n");
  fclose(fout);
}

//---------------------------------------------------------------------------
// Testing.

const double kTolerance = 1e-9;

static vector<string> LineOrArcListToStringList(const vector<LineOrArc> &a) {
  vector<string> result;
  for (int i = 0; i < a.size(); i++) {
    string s;
    StringAppendF(&s, "start=%g,%g end=%g,%g center=%g,%g type=",
                  a[i].start[0], a[i].start[1], a[i].end[0], a[i].end[1],
                  a[i].center[0], a[i].center[1]);
    if (a[i].type == LineOrArc::ARC_CW) {
      s += "ARC_CW\n";
    } else if (a[i].type == LineOrArc::ARC_CCW) {
      s += "ARC_CCW\n";
    } else if (a[i].type == LineOrArc::CIRCLE) {
      s += "CIRCLE\n";
    } else {
      s += "LINE\n";
    }
    result.push_back(s);
  }
  return result;
}

void PrintStringList(const vector<string> &s) {
  for (int i = 0; i < s.size(); i++) {
    printf("%s", s[i].c_str());
  }
}

// Return true if the first string vector is some rotation of the second string
// vector.
static bool IsSomeRotationOf(const vector<string> &a, const vector<string> &b) {
  if (a.size() != b.size())
    return false;
  for (int r = 0; r < a.size(); r++) {  // 'r' is amount to rotate by
    bool equal = true;
    for (int i = 0; i < a.size(); i++) {
      equal &= (a[i] == b[(i + r) % b.size()]);
    }
    if (equal)
      return true;
  }
  return false;
}

// Make a shape with two arcs. n is the number of points per arc. If n==2 then
// there are no arcs, just line segments.

static void MakeShapeWithTwoArcs(int n, vector<Point> &p,
                                 bool concave_arcs = true) {
  p.push_back(Point(1, 0));
  p.push_back(Point(1, 1));
  for (int i = 0; i < n; i++) {
    double angle = i / double(n - 1) * M_PI * 0.5;
    if (concave_arcs) {
      p.push_back(Point(2 + sin(angle), 2 - cos(angle)));
    } else {
      p.push_back(Point(3 - cos(angle), 1 + sin(angle)));
    }
  }
  for (int i = 0; i < n; i++) {
    double angle = i / double(n - 1) * M_PI * 0.5;
    if (concave_arcs) {
      p.push_back(Point(5 - cos(angle), 2 - sin(angle)));
    } else {
      p.push_back(Point(4 + sin(angle), 1 + cos(angle)));
    }
  }
  p.push_back(Point(6, 1));
  p.push_back(Point(6, 0));
}

TEST_FUNCTION(DXF_FitCircleTo3Points) {
  for (int iter = 0; iter < 10; iter++) {
    Point p[3];
    for (int i = 0; i < 3; i++) {
      p[i][0] = Random();
      p[i][1] = Random();
      // printf("p[%d] = %f,%f\n", i, p[i][0], p[i][1]);
    }
    double radius;
    Point center;
    FitCircleTo3Points(p[0], p[1], p[2], &radius, &center);
    // printf("center = %f,%f\n", center[0], center[1]);
    for (int i = 0; i < 3; i++) {
      double len = (p[i] - center).norm();
      // printf("radius error = %e\n", len - radius);
      CHECK(fabs(len - radius) < 1e-9);
    }
  }
}

TEST_FUNCTION(DXF_ArcifyPointList_TwoArcs) {
  // Test shapes with two arcs, with varying numbers of arc points and with
  // varying rotations of the data points.

  // The answer we expect in the forward direction is some rotation of this:
  vector<string> golden_fwd;
  golden_fwd.push_back("start=1,0 end=1,1 center=0,0 type=LINE\n");
  golden_fwd.push_back("start=1,1 end=2,1 center=0,0 type=LINE\n");
  golden_fwd.push_back("start=2,1 end=3,2 center=2,2 type=ARC_CCW\n");
  golden_fwd.push_back("start=3,2 end=4,2 center=0,0 type=LINE\n");
  golden_fwd.push_back("start=4,2 end=5,1 center=5,2 type=ARC_CCW\n");
  golden_fwd.push_back("start=5,1 end=6,1 center=0,0 type=LINE\n");
  golden_fwd.push_back("start=6,1 end=6,0 center=0,0 type=LINE\n");
  golden_fwd.push_back("start=6,0 end=1,0 center=0,0 type=LINE\n");

  // The answer we expect in the reverse direction is some rotation of this:
  vector<string> golden_rev;
  golden_rev.push_back("start=6,0 end=6,1 center=0,0 type=LINE\n");
  golden_rev.push_back("start=6,1 end=5,1 center=0,0 type=LINE\n");
  golden_rev.push_back("start=5,1 end=4,2 center=5,2 type=ARC_CW\n");
  golden_rev.push_back("start=4,2 end=3,2 center=0,0 type=LINE\n");
  golden_rev.push_back("start=3,2 end=2,1 center=2,2 type=ARC_CW\n");
  golden_rev.push_back("start=2,1 end=1,1 center=0,0 type=LINE\n");
  golden_rev.push_back("start=1,1 end=1,0 center=0,0 type=LINE\n");
  golden_rev.push_back("start=1,0 end=6,0 center=0,0 type=LINE\n");

  // Check the checks.
  {
    for (int i = 0; i < golden_fwd.size(); i++) {
      vector<string> a(golden_fwd), b(golden_fwd);
      std::rotate(b.begin(), b.begin() + i, b.end());
      CHECK(IsSomeRotationOf(a, b));
      b.pop_back();
      CHECK(!IsSomeRotationOf(a, b));
      a.erase(a.begin() + (i + a.size() - 1) % a.size());
      CHECK(IsSomeRotationOf(a, b));
      a[0] = "abc";
      CHECK(!IsSomeRotationOf(a, b));
    }
    CHECK(!IsSomeRotationOf(golden_fwd, golden_rev));
  }

  for (int n = 4; n < 10; n++) {              // Number of arc points.
    int sz = 0;
    {
      vector<Point> p;
      MakeShapeWithTwoArcs(n, p);
      sz = p.size();
    }
    for (int r = 0; r < sz; r++) {            // All rotations of p
      vector<Point> p;
      MakeShapeWithTwoArcs(n, p);
      std::rotate(p.begin(), p.begin() + r, p.end());

      vector<LineOrArc> result;
      ArcifyPointList(p, 0.6, 45, kTolerance, &result);
      vector<string> s = LineOrArcListToStringList(result);
      // PrintStringList(s);
      CHECK(IsSomeRotationOf(s, golden_fwd));

      // The same shape, but in reverse.
      std::reverse(p.begin(), p.end());
      ArcifyPointList(p, 0.6, 45, kTolerance, &result);
      s = LineOrArcListToStringList(result);
      // PrintStringList(s);
      CHECK(IsSomeRotationOf(s, golden_rev));
    }
  }
}

TEST_FUNCTION(DXF_ArcifyPointList_TwoArcsConvex) {
  // Similar to the above but use convex arcs instead of concave arcs. Test
  // shapes with two arcs, with varying numbers of arc points and with
  // varying rotations of the data points.

  // The answer we expect in the forward direction is some rotation of this:
  vector<string> golden_fwd;
  golden_fwd.push_back("start=1,0 end=1,1 center=0,0 type=LINE\n");
  golden_fwd.push_back("start=1,1 end=2,1 center=0,0 type=LINE\n");
  golden_fwd.push_back("start=2,1 end=3,2 center=3,1 type=ARC_CW\n");
  golden_fwd.push_back("start=3,2 end=4,2 center=0,0 type=LINE\n");
  golden_fwd.push_back("start=4,2 end=5,1 center=4,1 type=ARC_CW\n");
  golden_fwd.push_back("start=5,1 end=6,1 center=0,0 type=LINE\n");
  golden_fwd.push_back("start=6,1 end=6,0 center=0,0 type=LINE\n");
  golden_fwd.push_back("start=6,0 end=1,0 center=0,0 type=LINE\n");

  // The answer we expect in the reverse direction is some rotation of this:
  vector<string> golden_rev;
  golden_rev.push_back("start=6,0 end=6,1 center=0,0 type=LINE\n");
  golden_rev.push_back("start=6,1 end=5,1 center=0,0 type=LINE\n");
  golden_rev.push_back("start=5,1 end=4,2 center=4,1 type=ARC_CCW\n");
  golden_rev.push_back("start=4,2 end=3,2 center=0,0 type=LINE\n");
  golden_rev.push_back("start=3,2 end=2,1 center=3,1 type=ARC_CCW\n");
  golden_rev.push_back("start=2,1 end=1,1 center=0,0 type=LINE\n");
  golden_rev.push_back("start=1,1 end=1,0 center=0,0 type=LINE\n");
  golden_rev.push_back("start=1,0 end=6,0 center=0,0 type=LINE\n");

  for (int n = 4; n < 10; n++) {              // Number of arc points.
    int sz = 0;
    {
      vector<Point> p;
      MakeShapeWithTwoArcs(n, p, false);
      sz = p.size();
    }
    for (int r = 0; r < sz; r++) {            // All rotations of p
      vector<Point> p;
      MakeShapeWithTwoArcs(n, p, false);
      std::rotate(p.begin(), p.begin() + r, p.end());

      vector<LineOrArc> result;
      ArcifyPointList(p, 0.6, 45, kTolerance, &result);
      vector<string> s = LineOrArcListToStringList(result);
      // PrintStringList(s);
      CHECK(IsSomeRotationOf(s, golden_fwd));

      // The same shape, but in reverse.
      std::reverse(p.begin(), p.end());
      ArcifyPointList(p, 0.6, 45, kTolerance, &result);
      s = LineOrArcListToStringList(result);
      // PrintStringList(s);
      CHECK(IsSomeRotationOf(s, golden_rev));
    }
  }
}

TEST_FUNCTION(DXF_ArcifyPointList_NoArcs) {
  // Test shapes with no arcs, with varying rotations of the data points.

  // The answer we expect is some rotation of this:
  vector<string> golden;
  golden.push_back("start=1,0 end=1,1 center=0,0 type=LINE\n");
  golden.push_back("start=1,1 end=2,1 center=0,0 type=LINE\n");
  golden.push_back("start=2,1 end=3,2 center=0,0 type=LINE\n");
  golden.push_back("start=3,2 end=4,2 center=0,0 type=LINE\n");
  golden.push_back("start=4,2 end=5,1 center=0,0 type=LINE\n");
  golden.push_back("start=5,1 end=6,1 center=0,0 type=LINE\n");
  golden.push_back("start=6,1 end=6,0 center=0,0 type=LINE\n");
  golden.push_back("start=6,0 end=1,0 center=0,0 type=LINE\n");

  for (int r = 0; r < 8; r++) {            // All rotations of p
    vector<Point> p;
    MakeShapeWithTwoArcs(2, p);
    CHECK(p.size() == 8);
    std::rotate(p.begin(), p.begin() + r, p.end());
    vector<LineOrArc> result;
    ArcifyPointList(p, 0.6, 45, kTolerance, &result);
    vector<string> s = LineOrArcListToStringList(result);
    // PrintStringList(s);
    CHECK(IsSomeRotationOf(s, golden));
  }
}

TEST_FUNCTION(DXF_ArcifyPointList_OneCircle) {
  // A single circle.
  for (int n = 4; n < 15; n++) {      // Number of points in circle
    vector<Point> p;
    for (int i = 0; i < n; i++) {
      double a = i / double(n) * M_PI * 2.0;
      p.push_back(Point(1+cos(a), 2+sin(a)));
    }
    vector<LineOrArc> result;
    ArcifyPointList(p, 1.5, 360.0 / n + 0.1, kTolerance, &result);
    vector<string> s = LineOrArcListToStringList(result);
    // PrintStringList(s);
    CHECK(s.size() == 1 &&
          s[0] == "start=2,2 end=0,0 center=1,2 type=CIRCLE\n");
  }
}

TEST_FUNCTION(DXF_ArcifyPointList_Semicircle) {
  // A semicircle with a single line segment across the diameter.
  for (int n = 4; n < 15; n++) {      // Number of point in semicircle
    for (int r = 0; r < n; r++) {     // All rotations of p
      vector<Point> p;
      for (int i = 0; i < n; i++) {
        double a = i / double(n - 1) * M_PI;
        p.push_back(Point(1+cos(a), 2+sin(a)));
      }
      std::rotate(p.begin(), p.begin() + r, p.end());
      vector<LineOrArc> result;
      ArcifyPointList(p, 10, 180.0 / (n-1) + 0.1, kTolerance, &result);
      vector<string> s = LineOrArcListToStringList(result);
      // PrintStringList(s);
      CHECK(s.size() == 2 &&
            s[0] == "start=2,2 end=0,2 center=1,2 type=ARC_CCW\n" &&
            s[1] == "start=0,2 end=2,2 center=0,0 type=LINE\n");
    }
  }
}

TEST_FUNCTION(DXF_ArcifyPointList_ClippedCircle) {
  // A circle with a slice taken out of one side
  const int n = 100;            // Number of point in arc
  vector<Point> p;
  double start_angle = atan2(1, 10);
  for (int i = 0; i < n; i++) {
    double a = start_angle + i / double(n - 1) * (2*M_PI - 2*start_angle);
    p.push_back(Point(cos(a), sin(a)));
  }
  vector<LineOrArc> result;
  ArcifyPointList(p, 100, 4, kTolerance, &result);
  vector<string> s = LineOrArcListToStringList(result);
  PrintStringList(s);
  CHECK(s.size() == 2 &&
        s[0] == "start=0.995037,0.0995037 end=0.995037,-0.0995037 center=0,0 type=ARC_CCW\n" &&
        s[1] == "start=0.995037,-0.0995037 end=0.995037,0.0995037 center=0,0 type=LINE\n");
}

TEST_FUNCTION(DXF_ArcifyPointList_BackToBackArcs) {
  // Back-to-back arcs.
  vector<Point> p;
  for (int i = 0; i < 5; i++) {
    const int n = 10;         // Number of points per arc
    for (int j = 0; j < n - 1; j++) {
      double angle = j / double(n - 1) * M_PI;
      double sign = (i & 1) ? -1 : 1;
      p.push_back(Point(i + 0.5 - 0.5 * cos(angle),
                        1 + 0.5 * sign * sin(angle)));
    }
  }
  const int n2 = 100;       // Number of points for the return arc
  for (int j = 0; j < n2 - 1; j++) {
    double angle = j / double(n2 - 1) * M_PI;
    p.push_back(Point(2.5 + 2.5 * cos(angle), 1 - 2.5 * sin(angle)));
  }
  vector<LineOrArc> result;
  ArcifyPointList(p, 0.5, 60.0, kTolerance, &result);
  vector<string> s = LineOrArcListToStringList(result);
  // PrintStringList(s);
  CHECK(s.size() == 6 &&
        s[0] == "start=0,1 end=1,1 center=0.5,1 type=ARC_CW\n" &&
        s[1] == "start=1,1 end=2,1 center=1.5,1 type=ARC_CCW\n" &&
        s[2] == "start=2,1 end=3,1 center=2.5,1 type=ARC_CW\n" &&
        s[3] == "start=3,1 end=4,1 center=3.5,1 type=ARC_CCW\n" &&
        s[4] == "start=4,1 end=5,1 center=4.5,1 type=ARC_CW\n" &&
        s[5] == "start=5,1 end=0,1 center=2.5,1 type=ARC_CW\n");
}

}  // namespace dxf
