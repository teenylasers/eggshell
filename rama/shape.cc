// Rama Simulator, Copyright (C) 2014-2020 Russell Smith.
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

#include <math.h>
#include <algorithm>
#include "common.h"
#include "shape.h"
#include "mesh.h"
#include "../toolkit/gl_utils.h"
#include "../toolkit/shaders.h"
#include "../toolkit/gl_font.h"
#include "../toolkit/testing.h"
#include "../toolkit/dxf.h"
#include "../toolkit/collision.h"

using ClipperLib::IntPoint;
using ClipperLib::Clipper;
using ClipperLib::Path;
using ClipperLib::Paths;
using ClipperLib::ClipType;
using ClipperLib::ptClip;
using ClipperLib::ptSubject;
using ClipperLib::pftPositive;
using ClipperLib::ctIntersection;
using ClipperLib::ctUnion;
using ClipperLib::ctDifference;
using ClipperLib::ctXor;
using Eigen::Vector3f;
using Eigen::Vector4f;

// All clipper coordinates are 64 bit integers. Our double precision
// coordinates are scaled to use 32 of those bits to the left and right of the
// decimal point. See comments in Shape::ClipperBounds.
const double kCoordScale = 4294967296;  // =2^32

// Geometry tolerances
const double kTol = 1e-20;              // Generic
const double kTolClean = 1e-9;          // Multiplies largest side length
const JetNum kMaxStepsAllowed = 1e4;    // For grow() round style, etc

//***************************************************************************
// Utility.

// Rotate a JetPoint by 90 degrees counterclockwise.
static inline JetPoint Rotate90(const JetPoint &p) {
  return JetPoint(-p[1], p[0]);
}

// Compute the cross product of two JetPoints.
static inline JetNum Cross2(const JetPoint &a, const JetPoint &b) {
  return a[0] * b[1] - a[1] * b[0];
}

// Raise an error if the size of the stack is not n. Assume index 1 on the
// stack is the Shape userdata.
static void Expecting(lua_State *L, int n, const char *func_name) {
  if (lua_gettop(L) != n) {
    LuaError(L, "Shape:%s() expecting %d arguments", func_name, n - 1);
  }
}
static void GlobalExpecting(lua_State *L, int n, const char *func_name) {
  if (lua_gettop(L) != n) {
    LuaError(L, "%s() expecting %d arguments", func_name, n);
  }
}

static Shape::CornerStyle CheckStyle(lua_State *L, int n, bool for_endcaps,
                                     const char *func_name) {
  const char *s = luaL_checkstring(L, n);
  if (strcmp(s, "square") == 0) {
    return Shape::SQUARE;
  } else if (strcmp(s, "round") == 0) {
    return Shape::ROUND;
  } else if (!for_endcaps && strcmp(s, "miter") == 0) {
    return Shape::MITER;
  } else if (for_endcaps && strcmp(s, "butt") == 0) {
    return Shape::BUTT;
  } else {
    LuaError(L, "%sstyle for Shape:%s() should be 'square', 'round' or '%s'",
             for_endcaps ? "endcap " : "", func_name,
             for_endcaps ? "butt" : "miter");
  }
}

// If there is a table at the given (absolute) index containing {p=piece,
// e=edge} numbers then return true and return the numbers in piece, edge.
// Otherwise, return false. In either case the stack will be left unchanged.
static bool GetPieceEdge(lua_State *L, int index, int *piece, int *edge) {
  int top = lua_gettop(L);
  CHECK(index >= 1);            // Absolute indexes simplify code below
  if (lua_type(L, index) == LUA_TTABLE) {
    lua_pushstring(L, "p");
    lua_gettable(L, index);
    if (lua_type(L, -1) == LUA_TNUMBER) {
      *piece = ToInt64(lua_tonumber(L, -1));
      lua_pop(L, 1);
      lua_pushstring(L, "e");
      lua_gettable(L, index);
      if (lua_type(L, -1) == LUA_TNUMBER) {
        *edge = ToInt64(lua_tonumber(L, -1));
        lua_settop(L, top);
        return true;
      }
    }
  }
  lua_settop(L, top);
  return false;
}

// If there is an array at the given (absolute) index containing {p=piece,
// e=edge} tables then return true and return the numbers in piece, edge.
// Otherwise, return false. In either case the stack will be left unchanged.
static bool GetPieceEdgeArray(lua_State *L, int index,
                              vector<int> *pieces, vector<int> *edges) {
  int top = lua_gettop(L);
  CHECK(index >= 1);            // Absolute indexes simplify code below
  if (lua_type(L, index) != LUA_TTABLE) {
    return false;
  }
  pieces->clear();
  edges->clear();
  lua_len(L, index);
  int n = ToInt64(lua_tonumber(L, -1));
  lua_pop(L, 1);
  for (int i = 1; i <= n; i++) {
    lua_rawgeti(L, index, i);
    int piece, edge;
    if (!GetPieceEdge(L, lua_gettop(L), &piece, &edge)) {
      lua_settop(L, top);
      return false;
    }
    pieces->push_back(piece);
    edges->push_back(edge);
    lua_pop(L, 1);
  }
  lua_settop(L, top);
  return true;
}

// Return the shortest distance between x,y and the line segment p1..p2. Return
// in 'excursion' the distance that x,y is beyond the endcap of the line (this
// is used to break ties when finding the shortest distance to multiple line
// segments).
static JetNum ShortestDistanceToLineSeg(JetPoint xy, JetPoint p1,
                                        JetPoint p2, JetNum *excursion) {
  JetNum len2 = (p2 - p1).squaredNorm();
  JetNum alpha = (len2 == 0) ? 0.0 : (((xy - p1).dot(p2 - p1)) / len2);
  if (alpha < 0) {
    *excursion = -alpha * sqrt(len2);
    alpha = 0;
  } else if (alpha > 1) {
    *excursion = (alpha - 1.0) * sqrt(len2);
    alpha = 1;
  } else {
    *excursion = 0;
  }
  JetPoint q = p1 + alpha * (p2 - p1);
  return (q - xy).norm();
}

// Return true if the line p1..p2 intersects the box.
static bool DoesLineIntersectBox(const JetPoint &p1, const JetPoint &p2,
                                 JetNum xmin, JetNum xmax,
                                 JetNum ymin, JetNum ymax) {
  // From from computational geometry we know that two nonintersecting convex
  // polyhedra can be separated by a plane that is either parallel to a face of
  // one of the polyhedra or that contains an edge from each of the polyhedra.
  // Thus the 2D line-box intersection test checks 4 faces of the box and one
  // line axis.
  if ( (p1[0] < xmin && p2[0] < xmin) || (p1[0] > xmax && p2[0] > xmax) ||
       (p1[1] < ymin && p2[1] < ymin) || (p1[1] > ymax && p2[1] > ymax) ) {
    return false;
  }
  // Compute cross products between p2-p1 and all box vertices. If they're all
  // of the same sign then we have a separating axis.
  xmin -= p1[0];
  xmax -= p1[0];
  ymin -= p1[1];
  ymax -= p1[1];
  JetPoint v = p2 - p1;
  JetNum s1 = xmin * v[1] - ymin * v[0];
  JetNum s2 = xmax * v[1] - ymin * v[0];
  JetNum s3 = xmin * v[1] - ymax * v[0];
  JetNum s4 = xmax * v[1] - ymax * v[0];
  return !( (s1 > 0 && s2 > 0 && s3 > 0 && s4 > 0) ||
            (s1 < 0 && s2 < 0 && s3 < 0 && s4 < 0) );
}

// Return true if the line start1->end1 intersects the line start2->end2.
// Merely toucting at the end points does not count.
static bool DoesLineIntersectLine(const JetPoint &start1, const JetPoint &end1,
                                  const JetPoint &start2, const JetPoint &end2)
{
  // Check separating half spaces.
  JetNum s1 = Cross2(end1 - start1, start2 - start1);
  JetNum s2 = Cross2(end1 - start1, end2 - start1);
  if ((s1 <= 0 && s2 <= 0) || (s1 >= 0 && s2 >= 0)) {
    return false;
  }
  JetNum s3 = Cross2(end2 - start2, start1 - start2);
  JetNum s4 = Cross2(end2 - start2, end1 - start2);
  return !((s3 <= 0 && s4 <= 0) || (s3 >= 0 && s4 >= 0));
}

// Return the area of a polygon. The sign of the area depends on the
// orientation (positive for anticlockwise). The 'size' is the number of points
// in 'p' to consider as the polygon, or -1 to use them all.
static JetNum PolyArea(const vector<RPoint> &p, int size = -1) {
  size = (size == -1) ? p.size() : size;
  if (size < 3) {
    return 0;
  }
  JetNum a = 0;
  for (int i = 0, j = size-1; i < size; i++) {
    a += (p[j].p[0] + p[i].p[0]) * (p[j].p[1] - p[i].p[1]);
    j = i;
  }
  return -a * 0.5;
}

// Return the area of a triangle given its vertices. This is a specialization
// of PolyArea().
static JetNum TriangleArea(const JetPoint &a, const JetPoint &b,
                           const JetPoint &c) {
  return -0.5 * ((c[0] + a[0]) * (c[1] - a[1]) +
                 (a[0] + b[0]) * (a[1] - b[1]) +
                 (b[0] + c[0]) * (b[1] - c[1]));
}

bool TriangleIntersectsBox(const JetPoint *p[3],
                           double xmin, double xmax,
                           double ymin, double ymax) {
  // From from computational geometry we know that two nonintersecting convex
  // polyhedra can be separated by a plane that is either parallel to a face of
  // one of the polyhedra or that contains an edge from each of the polyhedra.
  // Thus the intersection test uses separating lines from 4 sides of the box
  // and 3 sides of the triangle.
  if ( ((*p[0])[0] < xmin && (*p[1])[0] < xmin && (*p[2])[0] < xmin) ||
       ((*p[0])[0] > xmax && (*p[1])[0] > xmax && (*p[2])[0] > xmax) ||
       ((*p[0])[1] < ymin && (*p[1])[1] < ymin && (*p[2])[1] < ymin) ||
       ((*p[0])[1] > ymax && (*p[1])[1] > ymax && (*p[2])[1] > ymax) ) {
    return false;
  }
  // Compute cross products between the delta vector for each triangle side and
  // all box vertices. If they're all positive (for a positive-area triangle,
  // or vice versa) then we have a separating axis.
  JetNum area = TriangleArea(*p[0], *p[1], *p[2]);
  for (int i = 0; i < 3; i++) {
    const JetPoint &pi = *p[i];
    const JetPoint &pj = *p[(i + 1) % 3];
    JetPoint v = pj - pi;
    JetNum s1 = (xmin - pi[0]) * v[1] - (ymin - pi[1]) * v[0];
    JetNum s2 = (xmax - pi[0]) * v[1] - (ymin - pi[1]) * v[0];
    JetNum s3 = (xmin - pi[0]) * v[1] - (ymax - pi[1]) * v[0];
    JetNum s4 = (xmax - pi[0]) * v[1] - (ymax - pi[1]) * v[0];
    if ((area <  0 && s1 <  0 && s2 <  0 && s3 <  0 && s4 <  0) ||
        (area >= 0 && s1 >= 0 && s2 >= 0 && s3 >= 0 && s4 >= 0)) {
      return false;
    }
  }
  return true;
}

// Returns 0 if 'p' is not in the triangle defined by a,b,c, +1 if it is, or -1
// if pt is on the triangle boundary.
int PointInTriangle(const JetPoint &p, const JetPoint &a,
                    const JetPoint &b, const JetPoint &c) {
  // Use a barycentric coordinate approach.
  JetPoint q = b - a;
  JetPoint r = c - a;
  JetNum det = 1.0 / (q[0] * r[1] - q[1] * r[0]);
  JetNum alpha = ( r[1] * (p[0] - a[0]) - r[0]*(p[1] - a[1])) * det;
  JetNum beta  = (-q[1] * (p[0] - a[0]) + q[0]*(p[1] - a[1])) * det;
  if (alpha < 0 || beta < 0 || (alpha + beta) > 1) {
    return 0;
  }
  if (alpha > 0 && beta > 0 && (alpha + beta) < 1) {
    return 1;
  }
  return -1;
}

// Returns 0 if 'pt' is not in the polygon, +1 if it is, or -1 if pt is on the
// polygon boundary.
static int PointInPolygon(const vector<RPoint> &path, const RPoint &pt) {
  // For just 3 points use the faster PointInTriangle(). This also allows
  // PointInTriangle() to be tested from the lua script using shape:Contains().
  size_t cnt = path.size();
  if (cnt < 3) {
    return 0;
  }
  if (cnt == 3) {
    return PointInTriangle(pt.p, path[0].p, path[1].p, path[2].p);
  }

  // What follows is a direct port of the clipper function, so that we don't
  // have to convert to/from integer coordinates.
  JetNum pt_x = pt.p[0];
  JetNum pt_y = pt.p[1];
  int result = 0;
  JetPoint ip = path[0].p;
  for(size_t i = 1; i <= cnt; ++i) {
    JetPoint ipNext = (i == cnt ? path[0].p : path[i].p);
    if (ipNext[1] == pt_y) {
      if ((ipNext[0] == pt_x) || (ip[1] == pt_y &&
          ((ipNext[0] > pt_x) == (ip[0] < pt_x))))
        return -1;
    }
    if ((ip[1] < pt_y) != (ipNext[1] < pt_y)) {
      if (ip[0] >= pt_x) {
        if (ipNext[0] > pt_x)
          result = 1 - result;
        else {
          JetNum d = (ip[0] - pt_x) * (ipNext[1] - pt_y) -
                     (ipNext[0] - pt_x) * (ip[1] - pt_y);
          if (d == 0)
            return -1;
          if ((d > 0) == (ipNext[1] > ip[1]))
            result = 1 - result;
        }
      } else {
        if (ipNext[0] > pt_x) {
          JetNum d = (ip[0] - pt_x) * (ipNext[1] - pt_y) -
                     (ipNext[0] - pt_x) * (ip[1] - pt_y);
          if (d == 0)
            return -1;
          if ((d > 0) == (ipNext[1] > ip[1]))
            result = 1 - result;
        }
      }
    }
    ip = ipNext;
  }
  return result;
}

void AnyPointInPoly(const vector<RPoint> &poly,
                    int poly_size, JetPoint *point) {
  // From http://apodeline.free.fr/FAQ/CGAFAQ/CGAFAQ-3.html, here is a method
  // based on the proof that there exists an internal diagonal, in [O'Rourke
  // (C), 13-14]. The idea is that the midpoint of a diagonal is interior to
  // the polygon.
  //
  // 1. Identify a convex vertex v; let its adjacent vertices be a and b.
  //    RLS modification: Take the convex vertex where abv has the largest
  //    area, to improve the chance that q is well away from the border.
  // 2. For each other vertex q do:
  //   2a. If q is inside avb, compute distance to v (orthogonal to ab).
  //   2b. Save point q if distance is a new min.
  // 3. If no point is inside, return midpoint of ab, or centroid of avb.
  // 4. Else if some point inside, qv is internal: return its midpoint.

  // Find convex vertex 'v' containing largest area avb triangle.
  if (poly_size == -1) {
    poly_size = poly.size();
  }
  CHECK(poly_size >= 3);
  JetNum area = PolyArea(poly, poly_size);
  CHECK(area != 0);             // Doesn't work for zero area polygons
  JetNum area_sign = (area >= 0) ? 1 : -1;
  int best_ia = -1;             // Index of best 'a' point
  JetNum best_area = 0;         // Largest area found so far
  for (int ia = 0; ia < poly_size; ia++) {
    int iv = (ia + 1) % poly_size;
    int ib = (ia + 2) % poly_size;
    JetNum a = area_sign * TriangleArea(poly[ia].p, poly[iv].p, poly[ib].p);
    if (a > best_area) {
      best_area = a;
      best_ia = ia;
    }
  }
  if (best_ia < 0) {
    // If we can't find a convex vertex the polygon is probably zero area. In
    // this case there is no vertex inside the polygon so we just return the
    // first vertex, which is as good as any other.
    *point = poly[0].p;
    return;
  }
  int ia = best_ia;
  int iv = (ia + 1) % poly_size;
  int ib = (ia + 2) % poly_size;

  // Compute the unit length vector orthogonal to the a-b axis.
  JetPoint axis = poly[ib].p - poly[ia].p;
  JetNum len = axis.norm() * area_sign;
  CHECK(len != 0);
  JetPoint ortho(-axis[1] / len, axis[0] / len);  // Rot left 90

  // Find the point q inside abv that is most distant from the a-b line.
  bool found_q = false;
  JetNum min_dist = __DBL_MAX__;
  JetPoint min_q;
  for (int iq = 0; iq < poly.size(); iq++) {
    if (iq != ia && iq != iv && iq != ib) {
      if (PointInTriangle(poly[iq].p, poly[ia].p,
                          poly[iv].p, poly[ib].p) == 1) {
        JetNum dist = ortho.dot(poly[iq].p);
        if (dist < min_dist) {
          found_q = true;
          min_dist = dist;
          min_q = poly[iq].p;
        }
      }
    }
  }

  // Return the midpoint of qv if we found q, or the abv centroid otherwise.
  if (found_q) {
    *point = (min_q + poly[iv].p) * 0.5;
  } else {
    (*point)[0] = (poly[ia].p[0] + poly[iv].p[0] + poly[ib].p[0]) / 3.0;
    (*point)[1] = (poly[ia].p[1] + poly[iv].p[1] + poly[ib].p[1]) / 3.0;
  }
}

// Clipper callback to set the 'Z' property for intersecting edges. e1bot and
// e1top are the vertices that define one line segment, e2bot and e2top define
// the other. pt is the intersection point.
void MyZFillCallback(IntPoint &e1bot, IntPoint &e1top,
                     IntPoint &e2bot, IntPoint &e2top, IntPoint &pt) {
  EdgeInfo e1b(e1bot.Z);
  EdgeInfo e1t(e1top.Z);
  EdgeInfo e2b(e2bot.Z);
  EdgeInfo e2t(e2top.Z);
  ClipperEdgeInfo new_et;
  float D1b, D1t, D2b, D2t;
  new_et.kind[0] = e1b.SharedKind(e1t, &D1b, &D1t);
  new_et.kind[1] = e2b.SharedKind(e2t, &D2b, &D2t);

  // If two port lines with the same kind are intersecting we will only have
  // one non-default kind in new_et, but there will be an ambiguity about which
  // interpolated distance to use. Simply discard one of them.
  if (!new_et.kind[0].IsDefault() && new_et.kind[0] == new_et.kind[1]) {
    new_et.kind[1].SetDefault();
  }

  // Interpolate port distances as necessary (no interpolation necessary for
  // other kinds of edges).
  new_et.dist[0] = 0;
  new_et.dist[1] = 0;
  if (new_et.kind[0].PortNumber()) {
    double len1 = sqr(double(pt.X - e1bot.X)) + sqr(double(pt.Y - e1bot.Y));
    double len2 = sqr(double(e1top.X - e1bot.X)) +
                  sqr(double(e1top.Y - e1bot.Y));
    double alpha = sqrt(len1 / len2);
    new_et.dist[0] = alpha * (D1t - D1b) + D1b;
  }
  if (new_et.kind[1].PortNumber()) {
    double len1 = sqr(double(pt.X - e2bot.X)) + sqr(double(pt.Y - e2bot.Y));
    double len2 = sqr(double(e2top.X - e2bot.X)) +
                  sqr(double(e2top.Y - e2bot.Y));
    double alpha = sqrt(len1 / len2);
    new_et.dist[1] = alpha * (D2t - D2b) + D2b;
  }

  // Compute the derivative of 'pt' with respect to the parameter. The simplest
  // way to do this is to recompute the coordinates of 'pt' using JetNum.
  JetPoint a(e1bot.X, e1bot.Y);
  JetPoint b(e2bot.X, e2bot.Y);
  JetPoint c(e1top.X, e1top.Y);
  JetPoint d(e2top.X, e2top.Y);
  a[0].Derivative() = e1bot.Z.derivative_x;
  b[0].Derivative() = e2bot.Z.derivative_x;
  c[0].Derivative() = e1top.Z.derivative_x;
  d[0].Derivative() = e2top.Z.derivative_x;
  a[1].Derivative() = e1bot.Z.derivative_y;
  b[1].Derivative() = e2bot.Z.derivative_y;
  c[1].Derivative() = e1top.Z.derivative_y;
  d[1].Derivative() = e2top.Z.derivative_y;
  JetPoint u = c - a, v = d - b;        // The two lines we're intersecting
  u = u / u.norm();                     // Their unit length vectors
  v = v / v.norm();
  JetNum k = ((b[0] - a[0]) * v[1] + (a[1] - b[1]) * v[0]) /
             (u[0]*v[1] - u[1]*v[0]);
  JetPoint p = a + u * k;
  new_et.derivative_x = p[0].Derivative();
  new_et.derivative_y = p[1].Derivative();

  pt.Z = new_et;
}

// Given two unit length vectors (v1,v2) that describe rays starting from the
// origin, return a positive 'beta' such that an arc of the given radius is
// tangent to the rays at beta*v1 and beta*v2
static JetNum ArcTwoTangents(JetNum radius, const JetPoint &v1,
                             const JetPoint &v2) {
  return abs(radius * 2.0 * Cross2(v2, v1) / (v1 - v2).squaredNorm());
}

// Given a unit length vector 'v' that describes a ray starting from the
// origin, return a positive 'beta' such that an arc of the given radius goes
// through point 'p' and is tangent to the ray at beta*v. If there are two
// possible solutions return the one with the largest beta. Return -1 if no
// solution can be found. Set 'lr' to +/- 1 depending on whether p is to the
// left or right of v.
static JetNum ArcOneTangent(JetNum radius, const JetPoint &p,
                            const JetPoint &v, double *lr) {
  // See if p is to the left or right of v.
  *lr = ToDouble(Cross2(v, p));  // lr positive if p to the left of v
  *lr = (*lr >= 0) ? 1 : -1;

  JetNum q = p.dot(Rotate90(v));
  JetNum sqrt_arg = q * ((*lr)*2.0*radius - q);
  if (sqrt_arg < 0) {
    return -1;          // p is too far away for the arc to reach v
  }

  return p.dot(v) + sqrt(sqrt_arg);
}

//***************************************************************************
// Lua global functions.

static int LuaRectangle(lua_State *L) {
  GlobalExpecting(L, 4, "Rectangle");
  Shape *s = LuaUserClassCreateObj<Shape>(L);
  s->SetRectangle(luaL_checknumber(L, 1), luaL_checknumber(L, 2),
                  luaL_checknumber(L, 3), luaL_checknumber(L, 4));
  return 1;
}

static int LuaCircle(lua_State *L) {
  GlobalExpecting(L, 4, "Circle");
  Shape *s = LuaUserClassCreateObj<Shape>(L);
  s->SetCircle(luaL_checknumber(L, 1), luaL_checknumber(L, 2),
               luaL_checknumber(L, 3), luaL_checkinteger(L, 4));
  return 1;
}

// Internal function used by user_script_util.lua, to examine a vertex edge
// kind and return information about it.
static int Lua__EdgeKind__(lua_State *L) {
  GlobalExpecting(L, 1, "__EdgeKind__");
  EdgeKind e;
  e.SetFromInteger(ToDouble(luaL_checknumber(L, 1)));
  lua_pushnumber(L, e.PortNumber());
  return 1;
}

//***************************************************************************
// EdgeInfo.

EdgeKind EdgeInfo::SharedKind(EdgeInfo e, float *Dthis, float *De) const {
  float dummy;
  if (!Dthis)
    Dthis = &dummy;
  if (!De)
    De = &dummy;
  if (!kind[0].IsDefault()) {
    *Dthis = dist[0];
    if (kind[0] == e.kind[0]) {
      *De = e.dist[0];
      return kind[0];
    } else if (kind[0] == e.kind[1]) {
      *De = e.dist[1];
      return kind[0];
    }
  }
  if (!kind[1].IsDefault()) {
    *Dthis = dist[1];
    if (kind[1] == e.kind[0]) {
      *De = e.dist[0];
      return kind[1];
    } else if (kind[1] == e.kind[1]) {
      *De = e.dist[1];
      return kind[1];
    }
  }
  *Dthis = 0;
  *De = 0;
  return EdgeKind();
}

bool EdgeInfo::SetUnused(EdgeKind new_kind, float new_dist) {
  CHECK(new_dist >= 0 && new_dist <= 1);
  if (kind[0].IsDefault() || kind[0] == new_kind) {
    kind[0] = new_kind;
    dist[0] = new_dist;
    return true;
  } else if (kind[1].IsDefault() || kind[1] == new_kind) {
    kind[1] = new_kind;
    dist[1] = new_dist;
    return true;
  } else {
    return false;
  }
}

//***************************************************************************
// Material.

bool Material::RunCallback(Lua *lua, bool within_lua,
                           vector<MaterialParameters> *result) {
  // Check that we have the callback function and two vector arguments on the
  // stack.
  int n = lua_gettop(lua->L()) - 2;     // First return argument slot
  CHECK(n >= 1);
  CHECK(lua_type(lua->L(), n) == LUA_TFUNCTION);
  LuaVector *x = LuaCastTo<LuaVector>(lua->L(), -2);
  LuaVector *y = LuaCastTo<LuaVector>(lua->L(), -1);
  CHECK(x && y && x->size() == y->size());
  // Call the function, check the return values.
  int code = lua->PCall(2, LUA_MULTRET);
  if (code != LUA_OK) {
    return false;
  }
  int nret = lua_gettop(lua->L()) - n + 1;
  if (nret != 1 && nret != 4 && nret != 5) {
    if (nret == 2) {
      Error("Two material parameters were returned so you might "
            "be using a deprecated complex number API. Use Complex() instead.");
    }
    Error("Callback should return 1, 4 or 5 material parameters");
    if (within_lua) {
      LuaError(lua->L(), "Bad material callback");
    }
    return false;
  }
  vector<vector<JetComplex>> vec(nret);
  for (int i = 0; i < nret; i++) {
    if (!ToJetComplexVector(lua->L(), n + i, &vec[i])) {
      Error("Invalid vectors (or complex vectors) returned");
      if (within_lua) {
        LuaError(lua->L(), "Bad material callback");
      }
      return false;
    }
    if (vec[i].size() != x->size()) {
      Error("Callback should return vectors (or complex vectors) "
            "of the same size as the x,y arguments");
      if (within_lua) {
        LuaError(lua->L(), "Bad material callback");
      }
      return false;
    }
    for (int j = 0; j < vec[i].size(); j++) {
      if (IsNaNValue(vec[i][j].real()) || IsNaNValue(vec[i][j].imag())) {
        Error("A NaN (not-a-number) was returned by a callback, "
              "in position %d of return value %d", j+1, i+1);
        if (within_lua) {
          LuaError(lua->L(), "Bad material callback");
        }
        return false;
      }
    }
  }
  result->clear();
  result->resize(x->size());    // Sets material parameters to their defaults
  for (int i = 0; i < x->size(); i++) {
    (*result)[i].epsilon = vec[0][i];
    if (vec.size() >= 4) {
      (*result)[i].SetSigmas(vec[1][i], vec[2][i], vec[3][i]);
    }
    if (vec.size() >= 5) {
      (*result)[i].excitation = vec[4][i];
    }
  }
  lua_settop(lua->L(), n - 1);
  return true;
}

//***************************************************************************
// Shape.

Shape::~Shape() {
}

const char *Shape::GeometryError(bool enforce_positive_area) const {
  if (IsEmpty()) {
    return "The shape is empty.";
  }
  int holecount = 0;
  for (int i = 0; i < polys_.size(); i++) {
    if (polys_[i].p.size() < 3) {
      return "At least one piece of the shape has less than three vertices.";
    }
    JetNum area = Area(i);
    if (area == 0) {
      return "At least one piece of the shape has zero area";
    }
    holecount += (area < 0);
  }
  if (enforce_positive_area && holecount >= NumPieces()) {
    //@@@ It doesn't seem possible to trigger this problem, and I can't even
    //@@@ remember what this check is for. Discard? Ditch the argument too.
    return "Shape has more holes than outer boundaries. "
           "Have you used the correct vertex winding order for outer "
           "boundaries and holes?";
  }

  //@@@ Also: test outer boundaries have positive area, inner holes have
  //    negative area. BUT some users of GeometryError() can accept Reverse()d
  //    shapes.
  //@@@ More checks.
  return 0;
}

void Shape::SetLuaGlobals(lua_State *L) {
  lua_pushcfunction(L, LuaRectangle);
  lua_setglobal(L, "Rectangle");
  lua_pushcfunction(L, LuaCircle);
  lua_setglobal(L, "Circle");
  lua_pushcfunction(L, Lua__EdgeKind__);
  lua_setglobal(L, "__EdgeKind__");
}

void Shape::Clear() {
  polys_.clear();
  port_callbacks_.clear();
}

void Shape::Dump() const {
  for (int i = 0; i < polys_.size(); i++) {
    printf("Poly %d (color=0x%x, area=%.4g):\n", i, polys_[i].material.color,
           ToDouble(Area(i)));
    for (int j = 0; j < polys_[i].p.size(); j++) {
      printf("\t(pt=%g,%g, kind=%d,%d, dist=%.2g,%.2g)\n",
             ToDouble(polys_[i].p[j].p[0]), ToDouble(polys_[i].p[j].p[1]),
             polys_[i].p[j].e.kind[0].IntegerForDebugging(),
             polys_[i].p[j].e.kind[1].IntegerForDebugging(),
             polys_[i].p[j].e.dist[0],
             polys_[i].p[j].e.dist[1]);
    }
  }
}

void Shape::DrawInterior(double alpha) const {
  if (IsEmpty() || GeometryError()) {
    return;
  }

  // Compute a triangulation of minimal triangle count that allows the polygon
  // set to be painted. Using the mesher to triangulate the polygon is probably
  // sub-optimal as we don't need any of the special mesh properties, and
  // faster computer-graphics-specific algorithms exist. On the other hand,
  // this is probably fast enough as the shapes we triangulate are usually
  // simple.
  // @@@ Cache the result for next time, for speed?
  Mesh mesh(*this, -1, NULL);
  if (!mesh.IsValidMesh()) {
    return;
  }

  // Draw the resulting triangles using the colors that come from the original
  // polygons.
  const vector<Material> &materials = mesh.materials();
  vector<Vector3f> points;
  vector<Vector4f> colors;
  gl::PushShader push_shader(gl::SmoothShader());
  for (int i = 0; i < mesh.triangles().size(); i++) {
    uint32 color = materials[mesh.triangles()[i].material].color;
    colors.push_back(Vector4f(((color & 0xff0000) >> 16) / 255.0f,
                              ((color & 0xff00) >> 8   ) / 255.0f,
                              ((color & 0xff)          ) / 255.0f, alpha));
    colors.push_back(colors.back());
    colors.push_back(colors.back());
    for (int j = 0; j < 3; j++) {
      int k = mesh.triangles()[i].index[j];
      points.push_back(Vector3f(ToDouble(mesh.points()[k].p[0]),
                                ToDouble(mesh.points()[k].p[1]), 0));
    }
  }
  gl::Draw(points, colors, GL_TRIANGLES);
}

void Shape::DrawBoundary(const Eigen::Matrix4d &camera_transform,
                         bool show_lines, bool show_ports, bool show_vertices,
                         double boundary_derivatives_scale) const {
  // First pass: regular polygon outline, regardless of edge kinds.
  if (show_lines) {
    gl::SetUniform("color", 0, 0, 0);
    vector<Vector3f> points;
    for (int i = 0; i < polys_.size(); i++) {
      // Some windows openGL implementations have a bug where a large
      // GL_LINE_LOOP doesn't work: every 1250 or so vertices the first vertex
      // will be reinserted into the list and mess up the shape. That's why we
      // use a bunch of GL_LINES here to draw a loop.
      const int n = polys_[i].p.size();
      for (int j = 0; j < n; j++) {
        points.push_back(Vector3f(ToDouble(polys_[i].p[j].p[0]),
                                  ToDouble(polys_[i].p[j].p[1]), 0));
        points.push_back(Vector3f(ToDouble(polys_[i].p[(j + 1) % n].p[0]),
                                  ToDouble(polys_[i].p[(j + 1) % n].p[1]), 0));
      }
    }
    gl::Draw(points, GL_LINES);
  }

  // Second pass: highlight port edges and draw port numbers.
  if (show_ports) {
    gl::PushShader push_shader(gl::SmoothShader());
    vector<Vector3f> points, colors;
    for (int i = 0; i < polys_.size(); i++) {
      for (int j = 0; j < polys_[i].p.size(); j++) {
        int j2 = (j + 1) % polys_[i].p.size();
        float d1, d2;
        EdgeKind etype = polys_[i].p[j].e.SharedKind(polys_[i].p[j2].e,
                                                     &d1, &d2);
        int pnum = etype.PortNumber();
        if (!etype.IsDefault()) {
          // Draw the edge from vertex j to j+1
          double x1 = ToDouble(polys_[i].p[j].p[0]);
          double y1 = ToDouble(polys_[i].p[j].p[1]);
          double x2 = ToDouble(polys_[i].p[j2].p[0]);
          double y2 = ToDouble(polys_[i].p[j2].p[1]);
          float col1 = d1;
          float col2 = d2;
          if (pnum) {
            colors.push_back(Vector3f(0, col1, 1 - col1));
          } else {
            colors.push_back(Vector3f(1, 0, 1));
          }
          points.push_back(Vector3f(x1, y1, 0));
          if (pnum) {
            colors.push_back(Vector3f(0, col2, 1 - col2));
          } else {
            colors.push_back(colors.back());
          }
          points.push_back(Vector3f(x2, y2, 0));

          if (pnum) {
            char s[100];
            snprintf(s, sizeof(s), " %d ", pnum);
            DrawStringM(s, (x1 + x2)/2, (y1 + y2)/2, 0, camera_transform,
                &port_number_font, 0,0,0,
                (y1 < y2) ? TEXT_ALIGN_LEFT : TEXT_ALIGN_RIGHT,
                (x1 < x2) ? TEXT_ALIGN_TOP : TEXT_ALIGN_BOTTOM);
          }
        }
      }
    }

    // Draw thick lines.
    gl::DrawThick(3, 3, false, [&]() {
      gl::Draw(points, colors, GL_LINES);
    });
  }

  // Third pass: draw vertices.
  if (show_vertices) {
    gl::SetUniform("color", 0, 0, 0);
    GL(PointSize)(5);
    vector<Vector3f> p;
    for (int i = 0; i < polys_.size(); i++) {
      for (int j = 0; j < polys_[i].p.size(); j++) {
        p.push_back(Vector3f(ToDouble(polys_[i].p[j].p[0]),
                             ToDouble(polys_[i].p[j].p[1]), 0));
      }
    }
    gl::Draw(p, GL_POINTS);
  }

  // Fourth pass: draw vertex derivatives.
  if (boundary_derivatives_scale > 0) {
    gl::SetUniform("color", 0, 0, 0);
    vector<Vector3f> p;
    for (int i = 0; i < polys_.size(); i++) {
      for (int j = 0; j < polys_[i].p.size(); j++) {
        p.push_back(Vector3f(ToDouble(polys_[i].p[j].p[0]),
                             ToDouble(polys_[i].p[j].p[1]), 0));
        p.push_back(Vector3f(
          ToDouble(polys_[i].p[j].p[0]) +
            boundary_derivatives_scale * polys_[i].p[j].p[0].Derivative(),
          ToDouble(polys_[i].p[j].p[1]) +
            boundary_derivatives_scale * polys_[i].p[j].p[1].Derivative(), 0));
      }
    }
    gl::Draw(p, GL_LINES);
  }
}

void Shape::AddPoint(JetNum x, JetNum y, const EdgeInfo *e) {
  if (polys_.empty()) {
    polys_.resize(1);
  }
  polys_.back().p.push_back(RPoint(x, y));
  if (e) {
    polys_.back().p.back().e = *e;
  }
}

void Shape::MakePolyline() {
  if (!polys_.empty()) {
    int n = polys_.back().p.size();
    for (int i = n - 2; i > 0; i--) {
      polys_.back().p.push_back(polys_.back().p[i]);
    }
  }
}

void Shape::SetToPiece(int n, const Shape &p) {
  CHECK(n >= 0 && n < p.polys_.size());
  // Use swaps where possible to minimize the number of copies.
  vector<Polygon> new_polys_(1);
  if (&p == this) {
    new_polys_[0].Swap(polys_[n]);
  } else {
    new_polys_[0] = p.polys_[n];
  }
  new_polys_.swap(polys_);
}

bool Shape::AssignPort(int piece, int edge, EdgeKind kind, PointMap *pmap) {
  int n = polys_[piece].p.size();
  if (pmap) {
    bool valid = true;
    for (int i = 0; i < 2; i++) {
      auto &p = polys_[piece].p[(edge + i) % n].p;      // i'th point of edge
      auto & pivector = (*pmap)[std::make_pair(p[0], p[1])];
      for (int j = 0; j < pivector.size(); j++) {
        int piece_index = pivector[j].first;
        int poly_index = pivector[j].second;
        valid |= polys_[piece_index].p[poly_index].e.SetUnused(kind, 1-i);
      }
    }
    return valid;
  } else {
    return polys_[piece].p[ edge         ].e.SetUnused(kind, 1) &&
           polys_[piece].p[(edge + 1) % n].e.SetUnused(kind, 0);
  }
}

JetNum Shape::Area(int n) const {
  return PolyArea(polys_[n].p);
}

JetNum Shape::TotalArea() const {
  JetNum area = 0;
  for (int i = 0; i < polys_.size(); i++) {
    area += Area(i);
  }
  return area;
}

bool Shape::SelfIntersection() const {
  for (int i = 0; i < polys_.size(); i++) {
    // Do the intersection check separately for each piece.
    const int n = polys_[i].p.size();
    vector<collision::AABB<2>> aabb(n);
    for (int j0 = 0; j0 < n; j0++) {
      int j1 = (j0 + 1) % n;
      auto &p0 = polys_[i].p[j0].p;
      auto &p1 = polys_[i].p[j1].p;
      for (int k = 0; k < 2; k++) {
        aabb[j0].min[k] = std::min(ToDouble(p0[k]), ToDouble(p1[k]));
        aabb[j0].max[k] = std::max(ToDouble(p0[k]), ToDouble(p1[k]));
      }
    }
    std::set<std::pair<int, int>> overlaps;
    collision::SweepAndPrune(aabb, &overlaps);
    for (auto o : overlaps) {
      CHECK(aabb[o.first].Overlaps(aabb[o.second]));
      // Check to see if the lines in these two overlapping boxes actually
      // intersect. Ignore lines that are adjacent in the polygon.
      if ((o.first + 1) % n != o.second && (o.second + 1) % n != o.first) {
        auto &p1 = polys_[i].p[o.first].p;
        auto &p2 = polys_[i].p[(o.first + 1) % n].p;
        auto &p3 = polys_[i].p[o.second].p;
        auto &p4 = polys_[i].p[(o.second + 1) % n].p;
        if (DoesLineIntersectLine(p1, p2, p3, p4)) {
          return true;
        }
      }
    }
  }
  return false;
}

JetNum Shape::SharpestAngle() const {
  // Holes have opposite winding order from outer boundaries, this fact
  // naturally takes care of the different interpretation of "sharpest angle"
  // between outer boundaries and holes.
  JetNum sharpest = 0;          // pi is maximum sharpness, 0 is least sharp
  for (int i = 0; i < polys_.size(); i++) {
    const int n = polys_[i].p.size();
    if (n >= 3) {
      for (int j = 0; j < n; j++) {
        // Test the corner p1 -> p2 -> p3
        JetPoint p1 = polys_[i].p[(j + n-1) % n].p;     // Last point
        JetPoint p2 = polys_[i].p[j].p;                 // This point
        JetPoint p3 = polys_[i].p[(j + 1) % n].p;       // Next point
        JetPoint u1 = (p2 - p1).normalized();           // p1 -> p2 unit vector
        JetPoint u2 = (p3 - p2).normalized();           // p2 -> p3 unit vector
        JetNum theta = atan2(Cross2(u1, u2), u1.dot(u2));
        if (theta > 0) {
          sharpest = std::max(sharpest, theta);
        }
      }
    }
  }
  // The sharpest corner has theta=PI. We return the difference so that angles
  // near zero are the sharpest, but we use fabs() in case numerical error puts
  // us slightly over pi.
  return fabs(M_PI - sharpest);
}

void Shape::ExtremeSideLengths(double *ret_length_max,
                               double *ret_length_min) const {
  double length_max = 0;
  double length_min = __DBL_MAX__;
  for (int i = 0; i < polys_.size(); i++) {
    for (int j1 = 0; j1 < polys_[i].p.size(); j1++) {
      int j2 = (j1 + 1) % polys_[i].p.size();
      double length = ToDouble((polys_[i].p[j2].p - polys_[i].p[j1].p).norm());
      length_max = std::max(length_max, length);
      length_min = std::min(length_min, length);
    }
  }
  *ret_length_max = length_max;
  *ret_length_min = length_min;
}

void Shape::SetRectangle(JetNum x1, JetNum y1, JetNum x2, JetNum y2) {
  Clear();
  polys_.resize(1);
  // Sort coordinates so that we always have a positive area:
  polys_[0].p.push_back(RPoint(std::min(x1,x2), std::min(y1,y2)));
  polys_[0].p.push_back(RPoint(std::max(x1,x2), std::min(y1,y2)));
  polys_[0].p.push_back(RPoint(std::max(x1,x2), std::max(y1,y2)));
  polys_[0].p.push_back(RPoint(std::min(x1,x2), std::max(y1,y2)));
}

void Shape::SetCircle(JetNum x, JetNum y, JetNum radius, int npoints) {
  Clear();
  polys_.resize(1);
  for (int i = 0; i < npoints; i++) {
    double angle = double(i) / double(npoints) * 2.0 * M_PI;
    polys_[0].p.push_back(RPoint(x + radius*cos(angle), y + radius*sin(angle)));
  }
}

void Shape::SetIntersect(const Shape &c1, const Shape &c2) {
  RunClipper(&c1, &c2, ctIntersection);
}

void Shape::SetUnion(const Shape &c1, const Shape &c2) {
  RunClipper(&c1, &c2, ctUnion);
}

void Shape::SetDifference(const Shape &c1, const Shape &c2) {
  RunClipper(&c1, &c2, ctDifference);
}

void Shape::SetXOR(const Shape &c1, const Shape &c2) {
  RunClipper(&c1, &c2, ctXor);
}

void Shape::Paint(const Shape &s, const Material &mat) {
  // We can't simply run the clipper to subtract 's' from all pieces of 'this',
  // since that would unrecoverably merge pieces with different materials
  // (since clipper doesn't understand anything about polygon materials).
  // Instead we group the pieces by material and then deal with these 'material
  // shapes' separately, taking care to preserve the material of each piece.
  //
  // The documentation insists that painting be the last step in model
  // creation, because shape boolean operators will erase material types.
  // Therefore we can assume that the individual shapes for each material are
  // disjoint (because that's enforced here). We can also assume that the
  // interior holes will have the same materials as the outer boundaries that
  // they are holes for, or in other words, holes are specific to material
  // shapes and are not holes for the entire shape (because that's also
  // enforced here).
  std::map<Material, vector<int>> matmap;  // Material -> indexes into polys_
  for (int i = 0; i < polys_.size(); i++) {
    matmap[polys_[i].material].push_back(i);
  }

  // Compute the coordinate conversion that we're going to use for all
  // clipping. It's important to be consistent so that this shape and the area
  // to paint will end up sharing coincident vertices that will be
  // un-duplicated in Triangulate().
  JetNum offset_x, offset_y, scale;
  if (!ClipperBounds(this, &s, &offset_x, &offset_y, &scale)) {
    return;
  }

  // We deal with each material shape separately.
  Shape result;
  for (auto matshape : matmap) {
    Shape a;                    // Shape for just this material
    for (int i = 0; i < matshape.second.size(); i++) {
      a.polys_.push_back(polys_[matshape.second[i]]);
    }
    Shape b;                    // Material shape minus painted shape s
    b.RunClipper(&a, &s, ctDifference, offset_x, offset_y, scale);
    for (int j = 0; j < b.polys_.size(); j++) {
      b.polys_[j].material = matshape.first;
    }
    result.polys_.insert(result.polys_.end(),
                         b.polys_.begin(), b.polys_.end());
  }

  // The area to paint is 's' intersected with the entire current polygon.
  Shape area_to_paint;
  area_to_paint.RunClipper(this, &s, ctIntersection, offset_x, offset_y, scale);
  if (area_to_paint.IsEmpty()) {
    return;  // Throw away all the work above if nothing to paint.
  }
  for (int i = 0; i < area_to_paint.polys_.size(); i++) {
    area_to_paint.polys_[i].material = mat;
  }
  result.polys_.insert(result.polys_.end(), area_to_paint.polys_.begin(),
                                            area_to_paint.polys_.end());
  polys_.swap(result.polys_);
}

void Shape::SetMerge(const Shape &s) {
  // This merges everything regardless of material:
  RunClipper(&s, NULL, ctUnion);
}

void Shape::GetBounds(JetNum *min_x, JetNum *min_y,
                      JetNum *max_x, JetNum *max_y) const {
  CHECK(!IsEmpty());
  *min_x = __DBL_MAX__, *min_y = __DBL_MAX__;
  *max_x = -__DBL_MAX__, *max_y = -__DBL_MAX__;
  UpdateBounds(min_x, min_y, max_x, max_y);
}

int Shape::Contains(JetNum x, JetNum y) {
  RPoint pt(x, y);
  int count = 0;
  for (int i = 0; i < polys_.size(); i++) {
    int q = ::PointInPolygon(polys_[i].p, pt);
    if (q == -1) {      // If x,y is on the boundary of any polygon then it's
      return q;         //   on the shape boundary
    }
    if (q) {            // x,y is in the interior of this polygon
      if (PolyArea(polys_[i].p) > 0) {
        count++;
      } else {
        count--;
      }
    }
  }
  return count > 0;
}

void Shape::Offset(JetNum dx, JetNum dy) {
  for (int i = 0; i < polys_.size(); i++) {
    for (int j = 0; j < polys_[i].p.size(); j++) {
      polys_[i].p[j].p[0] += dx;
      polys_[i].p[j].p[1] += dy;
    }
  }
}

void Shape::Scale(JetNum scalex, JetNum scaley) {
  for (int i = 0; i < polys_.size(); i++) {
    for (int j = 0; j < polys_[i].p.size(); j++) {
      polys_[i].p[j].p[0] *= scalex;
      polys_[i].p[j].p[1] *= scaley;
    }
  }
}

void Shape::Rotate(JetNum theta) {
  theta *= M_PI / 180.0;                // Convert degrees to radians
  JetNum c = cos(theta), s = sin(theta);
  JetMatrix2d R;
  R << c, -s, s, c;
  for (int i = 0; i < polys_.size(); i++) {
    for (int j = 0; j < polys_[i].p.size(); j++) {
      polys_[i].p[j].p = R * polys_[i].p[j].p;
    }
  }
}

void Shape::MirrorX(JetNum x_coord) {
  for (int i = 0; i < polys_.size(); i++) {
    for (int j = 0; j < polys_[i].p.size(); j++) {
      polys_[i].p[j].p[0] = 2.0*x_coord - polys_[i].p[j].p[0];
    }
  }
  Reverse();
}

void Shape::MirrorY(JetNum y_coord) {
  for (int i = 0; i < polys_.size(); i++) {
    for (int j = 0; j < polys_[i].p.size(); j++) {
      polys_[i].p[j].p[1] = 2.0*y_coord - polys_[i].p[j].p[1];
    }
  }
  Reverse();
}

void Shape::Reverse() {
  for (int i = 0; i < polys_.size(); i++) {
    std::reverse(polys_[i].p.begin(), polys_[i].p.end());
  }
}

void Shape::Grow(JetNum delta, CornerStyle style, JetNum limit,
                 CornerStyle endcap_style) {
  // We use similar definitions as the clipper library for miter, square and
  // round corners. We follow the clipper library's lead and use the algorithms
  // of:
  //
  //   http://www.me.berkeley.edu/~mcmains/pubs/DAC05OffsetPolygon.pdf
  //
  // We can not use clipper's implementation directly since it doesn't use
  // JetNum and does not provide a callback to assist in the creation of new
  // points. Some styles require additional points to be created at corners
  // (e.g. "round" can create many new points) but we don't have a mechanism to
  // properly interpolate all port properties. Instead, at all points with
  // non-default edge kinds we fall back on miter style with no miter limit, so
  // that the point will only move and no additional points will be created.
  if (fabs(delta) < kTol) {
    return;
  }
  JetNum miter_limit = (limit > 2) ? 2.0 / sqr(limit) : 0.5;    // For miters
  JetNum steps = M_PI / acos(1.0 - limit / fabs(delta));        // For round
  steps = std::min(steps, kMaxStepsAllowed);
  for (int i = 0; i < polys_.size(); i++) {
    const int n = polys_[i].p.size();
    // The following code will work even for just 2 points, which represents a
    // two point polyline.
    if (n < 2) {
      continue;
    }
    vector<RPoint> new_poly;
    new_poly.reserve(n);                // Will need at least this many slots
    for (int j = 0; j < n; j++) {
      new_poly.push_back(polys_[i].p[j]);       // Copy point properties
      // Process the corner p1 -> p2 -> p3 (i.e. adjust vertex p2).
      JetPoint p1 = polys_[i].p[(j + n-1) % n].p;       // Last point
      JetPoint p2 = polys_[i].p[j].p;                   // This point
      JetPoint p3 = polys_[i].p[(j + 1) % n].p;         // Next point
      JetPoint u1 = p2 - p1;            // p1 -> p2
      JetPoint u2 = p3 - p2;            // p2 -> p3
      JetNum u1_length = u1.norm();
      JetNum u2_length = u2.norm();
      if (u1_length < kTol || u2_length < kTol) {
        // Skip processing (nearly) coincident points.
        continue;
      }
      u1 /= u1_length;                  // u1 = unit vector for p1 -> p2
      u2 /= u2_length;                  // u2 = unit vector for p2 -> p3
      JetNum sin_theta = Cross2(u1, u2);
      JetNum cos_theta = u1.dot(u2);
      JetNum theta = atan2(sin_theta, cos_theta);

      CornerStyle local_style = style;
      if (!new_poly.back().e.IsDefault()) {
        local_style = MITER;                    // Force miter style, no limit
      } else if (style == MITER) {
        if ((1.0 + cos_theta) < miter_limit) {
          local_style = SQUARE;                 // Square off long miters
        }
      }

      // Make sure that spikes that go from point A -> B -> A are treated as
      // outside corners so that we can make the ends of polylines this way.
      // This ensures that such spikes will receive proper end caps.
      if (fabs(theta) > M_PI - 1e-6) {
        theta = M_PI;
        sin_theta = 0;
        cos_theta = -1;
        local_style = endcap_style;
      }

      if (sin_theta * delta >= 0) {
        // Convex corner.
        if (local_style == ROUND) {
          // Round corner. See clipper's "offset_triginometry2.svg" for the
          // math.
          int num_steps = ToInt64(ceil(fabs(theta) * steps / (2.0 * M_PI))) + 1;
          num_steps = std::max(num_steps, 2);   // At least first and last pt
          JetNum da = theta / double(num_steps - 1);
          JetMatrix2d R;
          R << cos(da), -sin(da), sin(da), cos(da);
          JetPoint v = -Rotate90(u1) * delta;
          for (int k = 0; k < num_steps - 1; k++) {
            new_poly.back().p += v;
            new_poly.push_back(polys_[i].p[j]);
            v = R * v;                                  // Rotate v
          }
          new_poly.back().p -= Rotate90(u2) * delta;
        } else if (local_style == SQUARE) {
          // Square corner. See clipper's "offset_triginometry.svg" for the
          // math.
          JetNum length = delta * tan(theta / 4.0);
          new_poly.back().p += length * u1 - delta * Rotate90(u1);
          new_poly.push_back(polys_[i].p[j]);
          new_poly.back().p -= length * u2 + delta * Rotate90(u2);
        } else if (local_style == MITER) {
          // Miter corner. The update below is equivalent to this formula but
          // is more numerically robust for small theta:
          //     p += (delta / sin_theta) * (u1 - u2)
          new_poly.back().p -= (delta / (1.0 + cos_theta)) * Rotate90(u1 + u2);
        } else {
          // Butt endcaps.
          new_poly.back().p -= delta * Rotate90(u1);
          new_poly.push_back(polys_[i].p[j]);
          new_poly.back().p -= delta * Rotate90(u2);
        }
      } else {
        // Concave corner. Using a miter at this corner is the wrong thing to
        // do as it will cause geometry problems when convex curves lead into
        // concave corners. Instead we insert two additional points,
        // essentially making fat lines that intersect. However this is the
        // wrong thing to do for edges with non-default edge kinds as the
        // SetMerge() below will cause interpolation of port distance values
        // and mess things up. So for ports we fall back on using a miter,
        // which usually works because ports typically have 90 degree angles.
        if (new_poly.back().e.IsDefault()) {
          new_poly.back().p -= delta * Rotate90(u1);
          new_poly.push_back(polys_[i].p[j]);
          new_poly.push_back(polys_[i].p[j]);
          new_poly.back().p -= delta * Rotate90(u2);
        } else {
          new_poly.back().p -= (delta / (1.0 + cos_theta)) * Rotate90(u1 + u2);
        }
      }
    }
    polys_[i].p.swap(new_poly);
  }

  // The above vertex manipulations are likely to leave a self-intersecting
  // polygon. Remove all areas with winding number <= 0.
  SetMerge(*this);

  // @@@ do we need to CleanPolygon?
}

void Shape::Clean(JetNum threshold, JetNum angle_threshold, int mode) {
  CHECK(mode == 1 || mode == 2);

  // The default threshold is the maximum side length * kTolClean.
  if (threshold == 0) {
    double length_max, length_min;
    ExtremeSideLengths(&length_max, &length_min);
    threshold = kTolClean * length_max;
  }

  // We don't delete points that are shared by multiple pieces, as these are
  // topologically important and might be referenced by different material's
  // polygons. But make sure we *do* consider repeated points in the same
  // piece. The use_count maps a point to piece number and use count.
  std::map<RPoint, std::pair<int, int>> use_count;
  for (int i = 0; i < polys_.size(); i++) {
    for (int j = 0; j < polys_[i].p.size(); j++) {
      std::pair<int, int> &uc = use_count[polys_[i].p[j]];
      if (uc.second == 0) {
        uc.first = i;           // First use detected on this piece
        uc.second = 1;
      } else if (uc.first != i) {
        uc.second++;            // N'th use detected on a different piece
      }
    }
  }

  for (int i = 0; i < polys_.size(); i++) {
    if (polys_[i].p.size() < 3) {
      continue;
    }
    for (int pass = 0; pass < mode; pass++) {
      // On the first pass delete only co-linear points that are closer to
      // their neighbors than the threshold. This is done first so that we have
      // the best chance of cleaning up the shape without modifying the
      // boundary. On the second pass delete all points that are closer to
      // their neighbors than the threshold.
      int n = polys_[i].p.size();
      for (int j1 = 0; j1 < n; j1++) {
        int j2 = (j1 + 1) % n;        // We consider deleting j2
        int j3 = (j1 + 2) % n;
        if (use_count[polys_[i].p[j2]].second <= 1) {
          JetPoint delta1 = polys_[i].p[j2].p - polys_[i].p[j1].p;
          JetPoint delta2 = polys_[i].p[j3].p - polys_[i].p[j2].p;
          JetNum length1 = delta1.norm();
          JetNum length2 = delta2.norm();
          JetNum sin_theta = Cross2(delta1, delta2) / (length1 * length2);
          if (pass == 1 || fabs(sin_theta) < angle_threshold) {
            // If fabs(sin_theta) is small, j1-j2-j3 are colinear.
            // Delete j2 if it's too close to j1 or j2:
            if (length1 < threshold || length2 < threshold) {
              polys_[i].p.erase(polys_[i].p.begin() + j2);
              j1--;
              n--;
            }
          }
        }
      }
    }
  }
}

void Shape::SplitPolygonsAtNecks() {
  // Make a stack of piece indices to process. We keep processing pieces until
  // there's nothing left on the stack.
  vector<int> stack(polys_.size());
  for (int i = 0; i < polys_.size(); i++) {
    stack[i] = i;
  }

  while (!stack.empty()) {
    // Process piece 'i'.
    int i = stack.back();
    stack.pop_back();

    // This piece will have necks if there are shared points. Multiple shared
    // points could require that the piece be split into an arbitrary number of
    // pieces. Creating all the splits in one go requires a lot of book
    // keeping. Instead we solve the much simpler problem of identifying just
    // *one* split, and then adding the two resulting pieces to the stack for
    // further analysis. Since the common case is just one split, this
    // algorithm does not sacrifice much speed. So, first make an index of all
    // the points, stopping when we have found a duplicate.
    std::map<std::pair<JetNum, JetNum>, int> point_map;  // x,y --> one index
    int dup1 = -1, dup2 = -1;   // Indexes of two duplicated points
    {
      auto &p = polys_[i].p;
      for (int j = 0; j < p.size(); j++) {
        auto key = std::make_pair(p[j].p[0], p[j].p[1]);
        if (point_map.count(key) > 0) {
          dup1 = point_map[key];
          dup2 = j;
          break;
        }
        point_map[key] = j;
      }
    }
    if (dup1 == -1) {
      continue;                 // No necks in this piece
    }

    // Make two polygons from dup1 -> dup2 and dup2 -> dup1. But zero-area
    // slivers (where two edges are coincident) are discarded and not created
    // as separate pieces.
    auto &p = polys_[i].p;
    vector<RPoint> newp1,newp2;
    for (int j = dup2; j != dup1; j = (j + 1) % p.size()) {
      newp1.push_back(p[j]);
    }
    for (int j = dup1; j != dup2; j = (j + 1) % p.size()) {
      newp2.push_back(p[j]);
    }
    if (newp1.size() > 2 && newp2.size() > 2) {
      polys_.resize(polys_.size() + 1);
      polys_.back().material = polys_[i].material;
      polys_[i].p.swap(newp1);
      polys_.back().p.swap(newp2);
      stack.push_back(i);
      stack.push_back(polys_.size() - 1);
    } else if (newp1.size() > 2) {
      polys_[i].p.swap(newp1);
      stack.push_back(i);
    } else if (newp2.size() > 2) {
      polys_[i].p.swap(newp2);
      stack.push_back(i);
    } else {
      polys_.erase(polys_.begin() + i);
    }
  }
}

void Shape::FindClosestEdge(JetNum x, JetNum y, int *piece, int *edge) {
  // Find the line segment with the shortest distance to x,y. Break ties
  // using the excursion of x,y beyond the endcap of the line.
  *piece = -1;
  *edge = -1;
  JetNum best_dist = __DBL_MAX__, best_excursion = __DBL_MAX__;
  for (int i = 0; i < polys_.size(); i++) {
    for (int j1 = 0; j1 < polys_[i].p.size(); j1++) {
      int j2 = (j1 + 1) % polys_[i].p.size();
      JetNum excursion;
      JetNum d = ShortestDistanceToLineSeg(JetPoint(x, y),
                                           polys_[i].p[j1].p, polys_[i].p[j2].p,
                                           &excursion);
      if (d < best_dist - kTol ||
          (d < best_dist + kTol && excursion < best_excursion)) {
        best_dist = d;
        best_excursion = excursion;
        *piece = i;
        *edge = j1;
      }
    }
  }
  CHECK(*piece >= 0 && *edge >= 0);     // Make sure shape not empty
}

void Shape::FindClosestVertex(JetNum x, JetNum y, int *piece, int *index) {
  *piece = -1;
  *index = -1;
  JetNum best_dist = __DBL_MAX__;
  for (int i = 0; i < polys_.size(); i++) {
    for (int j = 0; j < polys_[i].p.size(); j++) {
      JetNum d = (JetPoint(x, y) - polys_[i].p[j].p).squaredNorm();
      if (d < best_dist) {
        best_dist = d;
        *piece = i;
        *index = j;
      }
    }
  }
  CHECK(*piece >= 0 && *index >= 0);     // Make sure shape not empty
}

bool Shape::APointInside(int i, double *x, double *y) {
  if (GeometryError(false)) {
    return false;       // AnyPointInPoly() needs good geometry
  }

  // If i == -1 find the single positive area polygon, or if there is just one
  // polygon then use that.
  if (i == -1) {
    if (polys_.size() == 1) {
      i = 0;
    } else {
      for (int j = 0; j < polys_.size(); j++) {
        if (Area(j) > 0) {
          if (i >= 0) {
            return false;       // Found more than one positive area piece
          }
          i = j;
        }
      }
      if (i == -1) {
        return false;           // Found no positive area pieces
      }
    }
  }

  vector<RPoint> poly = polys_[i].p;
  for (int j = 0; j < polys_.size(); j++) {
    if (j != i) {
      poly.insert(poly.end(), polys_[j].p.begin(), polys_[j].p.end());
    }
  }
  JetPoint point;
  AnyPointInPoly(poly, polys_[i].p.size(), &point);
  *x = ToDouble(point[0]);
  *y = ToDouble(point[1]);
  return true;
}

bool Shape::FilletVertex(JetNum x, JetNum y, JetNum radius, JetNum limit,
                         JetPoint *pstart, JetPoint *pend, JetPoint *center,
                         bool mutate) {
  // Find the piece/vertex that is closest to x,y.
  int piece, index;
  FindClosestVertex(x, y, &piece, &index);
  vector<RPoint> &poly = polys_[piece].p;
  JetPoint pt = poly[index].p;

  // Find adjacent vertices.
  int i1 = (index + poly.size() - 1) % poly.size();
  int i2 = (index + 1) % poly.size();
  JetPoint p1 = poly[i1].p;
  JetPoint p2 = poly[i2].p;

  // Compute unit vectors to the prior and next vertex.
  JetPoint v1 = p1 - pt;
  JetPoint v2 = p2 - pt;
  JetNum len1 = v1.norm();
  JetNum len2 = v2.norm();
  v1 /= len1;                   // Unit vector to prior vertex
  v2 /= len2;                   // Unit vector to next vertex

  // Compute two arc tangent points: beta0*v1 and beta0*v2. This will only work
  // if the tangent points don't go off the end of the line segments.
  JetNum beta0 = ArcTwoTangents(radius, v1, v2);
  if (beta0 < 1e-6) {
    return true;                // Lines are (almost) colinear, nothing to do
  }

  // See if the beta0 arc will work. If not, try others.
  JetPoint c(0,0);              // Set to arc center point
  bool found_arc = false;
  if (beta0 <= len1 && beta0 <= len2) {
    // Arc tangent to v1 and v2.
    c = pt + beta0/(1.0 + v1.dot(v2))*(v1 + v2);
    p1 = pt + beta0*v1;         // Start point
    p2 = pt + beta0*v2;         // End point
    found_arc = true;
  } else {
    // Try arcs that are tangent with one line segment only, and fixed to the
    // endpoint of the other segment. We do not want to create arcs that go
    // outside the space swept from v1 to v2 (an interior arc).
    // Arc 1: An arc fixed to p2 but tangent to v1 (at point beta1*v1).
    // Arc 2: An arc fixed to p1 but tangent to v2 (at point beta2*v2).
    // If arc1 is valid and an interior arc, beta1 > len2 and beta1 < len1.
    // If arc2 is valid and an interior arc, beta2 > len1 and beta2 < len2.
    // Therefore there is only one valid situation: we use the valid arc with
    // beta >= the opposite len.
    double lr1 = 0, lr2 = 0;
    JetNum beta1 = ArcOneTangent(radius, p2 - pt, v1, &lr1);
    JetNum beta2 = ArcOneTangent(radius, p1 - pt, v2, &lr2);
    bool arc1_valid = beta1 > 0 && beta1 <= len1;
    bool arc2_valid = beta2 > 0 && beta2 <= len2;
    if (arc1_valid && (!arc2_valid || beta1 >= len2)) {
      p1 = pt + beta1*v1;         // Start point
      c = p1 + lr1 * radius * Rotate90(v1);
      found_arc = true;
    } else if (arc2_valid && (!arc1_valid || beta2 >= len1)) {
      p2 = pt + beta2*v2;         // End point
      c = p2 + lr2 * radius * Rotate90(v2);
      found_arc = true;
    }
  }
  if (!found_arc) {
    // Can not satisfy fillet radius on either segment, so do nothing.
    return false;
  }

  // Compute angular extent of the fillet.
  JetNum a1 = atan2(p1(1) - c(1), p1(0) - c(0));
  JetNum a2 = atan2(p2(1) - c(1), p2(0) - c(0));
  while (a2 > a1 + M_PI) {
    a2 -= 2 * M_PI;
  }
  while (a2 < a1 - M_PI) {
    a2 += 2 * M_PI;
  }

  // Compute the number of steps in the fillet.
  JetNum steps = M_PI / acos(1.0 - limit / radius);     // In a full circle
  steps = std::min(steps, kMaxStepsAllowed);
  int num_steps = ToInt64(ceil(abs(a2 - a1) * steps / (2.0 * M_PI))) + 1;
  if (num_steps <= 1) {
    return false;
  }

  // Replace pt with the fillet arc points.
  if (mutate) {
    poly.insert(poly.begin() + index, num_steps - 1, RPoint());
    for (int i = 0; i < num_steps; i++) {
      JetNum a = a1 + (a2 - a1) * JetNum(i) / JetNum(num_steps-1);
      poly[index + i].p =
        JetPoint(c(0) + radius * cos(a), c(1) + radius * sin(a));
    }
    // Ensure that we didn't make duplicate points at the start or end of the
    // arc. This might happen if the radius was limited by the adjacent edge
    // lengths.
    Clean();
  }

  // Return values.
  if (pstart && pend && center) {
    *pstart = p1;
    *pend = p2;
    *center = c;
  }

  return true;
}

void Shape::ChamferVertex(JetNum x, JetNum y, JetNum predist, JetNum postdist,
                          JetPoint *p1_ret, JetPoint *p2_ret) {
  // Find the piece/vertex that is closest to x,y.
  int piece, index;
  FindClosestVertex(x, y, &piece, &index);
  vector<RPoint> &poly = polys_[piece].p;
  JetPoint pt = poly[index].p;

  // Find adjacent vertices, compute chamfer points.
  int i1 = (index + poly.size() - 1) % poly.size();
  int i2 = (index + 1) % poly.size();
  JetPoint p1 = poly[i1].p;
  JetPoint p2 = poly[i2].p;
  predist = std::min(predist, (p1 - pt).norm());
  postdist = std::min(postdist, (p2 - pt).norm());
  p1 = pt + predist * (p1 - pt).normalized();
  p2 = pt + postdist * (p2 - pt).normalized();

  // Adjust the polygon.
  poly.insert(poly.begin() + index, 1, RPoint());
  poly[index].p = p1;
  poly[index + 1].p = p2;
  Clean();

  // Optionally return the coordinates of the new vertices.
  if (p1_ret)
    *p1_ret = p1;
  if (p2_ret)
    *p2_ret = p2;
}

void Shape::SaveBoundaryAsDXF(const char *filename,
                              double arc_dist, double arc_angle) {
  FILE *fout = fopen(filename, "wb");
  if (!fout) {
    Error("Can not write to '%s' (%s)", filename, strerror(errno));
    return;
  }
  vector<vector<dxf::Point>> p(polys_.size());
  for (int i = 0; i < polys_.size(); i++) {
    for (int j = 0; j < polys_[i].p.size(); j++) {
      p[i].push_back(dxf::Point(ToDouble(polys_[i].p[j].p[0]),
                                ToDouble(polys_[i].p[j].p[1])));
    }
  }
  const double kArcTolerance = 1e-6;
  dxf::WriteDXF(p, arc_dist, arc_angle, kArcTolerance, fout);
  fclose(fout);
}

void Shape::SaveBoundaryAsXY(const char *filename) {
  // Write an x,y list that is readable as a matlab matrix. Pieces are
  // separated with NaN coordinates.
  FILE *fout = fopen(filename, "wb");
  if (!fout) {
    Error("Can not write to '%s' (%s)", filename, strerror(errno));
    return;
  }
  for (int i = 0; i < polys_.size(); i++) {
    for (int j = 0; j < polys_[i].p.size(); j++) {
      fprintf(fout, "%.10e %.10e\n", ToDouble(polys_[i].p[j].p[0]),
                                     ToDouble(polys_[i].p[j].p[1]));
    }
    if ((i + 1) < polys_.size()) {
      fprintf(fout, "nan nan\n");
    }
  }
  fclose(fout);
}

bool Shape::LoadSTL(const char *filename) {
  // STL file parameters.
  const int kHeaderSize = 80;
  const int kVertexSize = 50;
  const double kTolerance = 1e-6;

  Clear();

  // Load the STL file data into 'v'. Each group of three vertices in 'v' is a
  // single triangle. The right hand rule defines the triangle normal that
  // points out of the solid. The vertices are ordered by just the X,Y part.
  struct STLPoint : public Vector3f {
    bool operator<(const STLPoint &p) const {
      return (*this)[0] < p[0] || ((*this)[0] == p[0] && (*this)[1] < p[1]);
    }
  };
  std::vector<STLPoint> v;
  {
    FILE *fin = fopen(filename, "rb");
    if (!fin) {
      Error("Can't open STL file '%s' (%s)", filename, strerror(errno));
      return false;
    }
    // Get file size, read header.
    fseek(fin, 0, SEEK_END);
    long size = ftell(fin);
    fseek(fin, kHeaderSize, SEEK_SET);      // Unused header
    uint32_t num_triangles = 0;
    size_t nr = fread(&num_triangles, sizeof(num_triangles), 1, fin);
    // Check that the file size matches 'num_triangles'.
    if (nr != 1 || size < 84 || size != (84 + num_triangles * kVertexSize)) {
      Error("STL file '%s' has unexpected size", filename);
      return false;
    }
    // Read just the vertex part of the file data, discard the normals.
    vector<uint8_t> bytes(num_triangles * kVertexSize);
    nr = fread(bytes.data(), kVertexSize, num_triangles, fin);
    if (nr != num_triangles || ferror(fin)) {
      Error("Error reading STL file '%s'", filename);
      return false;
    }
    v.resize(num_triangles * 3);
    for (int i = 0; i < num_triangles; i++) {
      for (int j = 0; j < 3; j++) {
        v[i*3+j] = *reinterpret_cast<STLPoint*>(bytes.data() + i * kVertexSize +
                                                (j+1) * 3 * sizeof(float));
      }
    }
    fclose(fin);
  }

  // Find the minimum Z coordinate in 'v'.
  float minz = __FLT_MAX__;
  for (int i = 0; i < v.size(); i++) {
    minz = std::min(minz, v[i][2]);
  }

  // Identify the edges that are referenced by just one of the triangles in the
  // z=minz plane. This is the boundary of the polygons we will create.
  std::map<std::pair<STLPoint, STLPoint>, int> edge_map;
  for (int i = 0; i < v.size() / 3; i++) {
    if (fabs(v[i*3+0][2] - minz) < kTolerance &&
        fabs(v[i*3+1][2] - minz) < kTolerance &&
        fabs(v[i*3+2][2] - minz) < kTolerance) {
      // Triangle is in the z=minz plane, add its edges to edge_map.
      for (int j = 0; j < 3; j++) {
        int j2 = (j + 1) % 3;
        std::pair<STLPoint, STLPoint> key2(v[i*3+j2], v[i*3+j]);
        if (edge_map.count(key2) > 0) {
          edge_map[key2]++;
        } else  {
          std::pair<STLPoint, STLPoint> key1(v[i*3+j], v[i*3+j2]);
          edge_map[key1]++;
        }
      }
    }
  }

  // Map all exterior vertices to the next vertex in the polygon.
  std::map<STLPoint, STLPoint> next_vertex;
  for (auto it : edge_map) {
    if (it.second == 1) {
      next_vertex[it.first.second] = it.first.first;
    }
  }

  // Emit all exterior vertices to polygons.
  while (!next_vertex.empty()) {
    polys_.resize(polys_.size() + 1);
    STLPoint first_vertex = next_vertex.begin()->first;
    STLPoint v = first_vertex;
    do {
      polys_.back().p.push_back(RPoint(v[0], v[1]));
      assert(next_vertex[v].count() == 1);
      STLPoint next_v = next_vertex[v];
      next_vertex.erase(v);
      v = next_v;
    } while (v != first_vertex);
  }
  return true;
}

// Update coordinate bounds. Return the number of coordinates processed.

int Shape::UpdateBounds(JetNum *min_x, JetNum *min_y,
                        JetNum *max_x, JetNum *max_y) const {
  int coord_count = 0;
  for (int p = 0; p < polys_.size(); p++) {
    int n = polys_[p].p.size();
    coord_count += n;
    for (int i = 0; i < n; i++) {
      *min_x = std::min(*min_x, polys_[p].p[i].p[0]);
      *max_x = std::max(*max_x, polys_[p].p[i].p[0]);
      *min_y = std::min(*min_y, polys_[p].p[i].p[1]);
      *max_y = std::max(*max_y, polys_[p].p[i].p[1]);
    }
  }
  return coord_count > 0;
}

void Shape::ToPaths(JetNum scale, JetNum offset_x, JetNum offset_y,
                    Paths *paths) const {
  paths->clear();
  paths->resize(polys_.size());
  for (int i = 0; i < polys_.size(); i++) {
    Path &path = (*paths)[i];
    for (int j = 0; j < polys_[i].p.size(); j++) {
      // RPoint coordinate derivative information is not stored in Clipper's
      // IntPoint the same way it's stored in RPoint. Unpack it here. Note that
      // we store *unscaled* derivatives in IntPoint.
      ClipperEdgeInfo e(polys_[i].p[j].e,
                        polys_[i].p[j].p[0].Derivative(),     // derivative_x
                        polys_[i].p[j].p[1].Derivative());    // derivative_y
      path.push_back(
          IntPoint(ToInt64(round(scale * (polys_[i].p[j].p[0] - offset_x))),
                   ToInt64(round(scale * (polys_[i].p[j].p[1] - offset_y))),
                   e));
    }
  }
}

void Shape::FromPaths(JetNum scale, JetNum offset_x, JetNum offset_y,
                      const Paths &paths) {
  Clear();
  polys_.resize(paths.size());
  for (int i = 0; i < paths.size(); i++) {
    polys_[i].p.resize(paths[i].size());
    for (int j = 0; j < paths[i].size(); j++) {
      // RPoint coordinate derivative information is not stored in Clipper's
      // IntPoint the same way it's stored in RPoint. Repack it here. Note that
      // we store *unscaled* derivatives in IntPoint.
      RPoint &p = polys_[i].p[j];
      p.p[0] = JetNum(paths[i][j].X) / scale + offset_x;
      p.p[1] = JetNum(paths[i][j].Y) / scale + offset_y;
      p.p[0].Derivative() = paths[i][j].Z.derivative_x;
      p.p[1].Derivative() = paths[i][j].Z.derivative_y;
      p.e = paths[i][j].Z;
    }
  }
}

void Shape::RunClipper(const Shape *c1, const Shape *c2, ClipType clip_type) {
  // @@@ Now that ClipperBounds returns constants we can probably ditch this
  //     function and just move the other RunClipper's body here.
  JetNum offset_x, offset_y, scale;
  if (!ClipperBounds(c1, c2, &offset_x, &offset_y, &scale)) {
    Clear();
    return;
  }
  RunClipper(c1, c2, clip_type, offset_x, offset_y, scale);
}

bool Shape::ClipperBounds(const Shape *c1, const Shape *c2, JetNum *offset_x,
                          JetNum *offset_y, JetNum *scale) const {
  // Compute the coordinate scaling to used when running Clipper on shapes 'c1'
  // and 'c2'. We used to scale and offset things so that we used exactly 32 of
  // the bits in the integer coordinates passed to clipper. This has the
  // advantage of working despite the user's own weird scaling (e.g. if there's
  // a circle the diameter of the Earth, in microns, that's ~1.3e+13). However
  // it means that the same shape may get different scalings when combined with
  // different pieces, which can lead to slivers being generated in Clipper as
  // different integer coordinates are used to represent the same points. So
  // now we use a constant scaling that gives us 32 bits to the left and right
  // of the decimal point. This quantizes coordinates to 2.3e-10 and gives us a
  // maximum of 2.1e9. This is generally not a problem: for config.unit set to
  // 'meters', the quantization size is on the order of the atomic spacing in a
  // crystal lattice.
  *offset_x = 0;
  *offset_y = 0;
  *scale = kCoordScale;
  return true;

  /*
   * The old way of doing things:
   *
   *  // Compute bounds of c1, c2 or both.
   *  CHECK(c1 || c2);
   *  JetNum min_x = __DBL_MAX__, min_y = __DBL_MAX__;
   *  JetNum max_x = -__DBL_MAX__, max_y = -__DBL_MAX__;
   *  if ((c1 ? c1->UpdateBounds(&min_x, &min_y, &max_x, &max_y) : 0) +
   *      (c2 ? c2->UpdateBounds(&min_x, &min_y, &max_x, &max_y) : 0) == 0) {
   *    // No coordinates in either c1 or c2. The min/max wont be valid and the
   *    // result will have to be empty anyway.
   *    return false;
   *  }
   *
   *  // Compute the scale factor from doubles to clipper integer coordinates.
   *  *offset_x = (max_x + min_x) / 2.0;
   *  *offset_y = (max_y + min_y) / 2.0;
   *  JetNum max_bound = std::max(max_x - min_x, max_y - min_y);
   *  *scale = kCoordScale / max_bound;
   *  return true;
   */
}

void Shape::RunClipper(const Shape *c1, const Shape *c2, ClipType clip_type,
                       JetNum offset_x, JetNum offset_y, JetNum scale) {
  // Compute the clipper polygons in integer coordinates.
  Paths p1, p2;
  if (c1) {
    c1->ToPaths(scale, offset_x, offset_y, &p1);
  }
  if (c2) {
    c2->ToPaths(scale, offset_x, offset_y, &p2);
  }

  // When clipper runs on two shapes with coincident vertices, there doesn't
  // seem to be any properly defined behavior for which of the vertex Z values
  // will end up in the final shape. This is a problem when two shapes that
  // both contain port definitions are combined.
  //
  // If c1 and c2 have coincident vertices then we want the EdgeInfo (e.g. port
  // definitions) to come from c1. This is the behavior defined in the manual
  // for the boolean shape operators, and it also prevents painting from
  // accidentally erasing port definitons (because in painting, c2 is always
  // the dielectric shape to paint, which shouldn't have ports). Achieve this
  // behavior by setting c2 EdgeInfo from c1 where ever there are coincident
  // vertices. We could have attempted to fix the behavior in clipper itself,
  // but the snippet of code below is way simpler.
  //
  // If there are coincident vertices in c1 (e.g. because of multiple
  // dielectrics or necked polygons) this will take just one of them at each
  // point. However this is not an allowed case yet as boolean operations on
  // dielectric-containing shapes are not allowed.
  using ClipperLib::cInt;
  std::map<std::pair<cInt,cInt>, ClipperEdgeInfo> point_map;
  for (int i = 0; i < p1.size(); i++) {
    for (int j = 0; j < p1[i].size(); j++) {
      point_map[std::make_pair(p1[i][j].X, p1[i][j].Y)] = p1[i][j].Z;
    }
  }
  for (int i = 0; i < p2.size(); i++) {
    for (int j = 0; j < p2[i].size(); j++) {
      auto it = point_map.find(std::make_pair(p2[i][j].X, p2[i][j].Y));
      if (it != point_map.end()) {
        p2[i][j].Z = it->second;
      }
    }
  }

  // Run the clipper. Use the pftPositive fill type so that a polygon can have
  // external invisible holes (the Paint() function can generate these).
  Paths result;
  try {
    Clipper clipper;
    clipper.ZFillFunction(MyZFillCallback);
    if (c1) {
      clipper.AddPaths(p1, ptSubject, true);
    }
    if (c2) {
      clipper.AddPaths(p2, ptClip, true);
    }
    clipper.Execute(clip_type, result, pftPositive, pftPositive);
  } catch(const ClipperLib::clipperException &e) {
    Error("Clipper says: %s", e.what());
  }

  // Convert the result back into JetPoint coordinates.
  FromPaths(scale, offset_x, offset_y, result);

  // Combine the port callbacks from both shapes.
  CombinePortCallbacks(c1, c2);
}

bool Shape::CombinePortCallbacks(const Shape *c1, const Shape *c2) {
  // Combine the port callbacks from both shapes. Generate an error if there
  // are conflicts.
  port_callbacks_.clear();
  if (c1) {
    port_callbacks_ = c1->port_callbacks_;
  }
  if (c2) {
    for (auto it : c2->port_callbacks_) {
      if (port_callbacks_.count(it.first) > 0) {
        Error("Merged shapes contain port callbacks for the same port.");
        return false;
      }
      port_callbacks_[it.first] = it.second;
    }
  }
  return true;
}

const Shape &Shape::LuaCheckShape(lua_State *L, int argument_index) const {
  CHECK(lua_gettop(L) >= argument_index);
  Shape *s = LuaCastTo<Shape>(L, argument_index);
  if (!s) {
    LuaError(L, "Argument %d must be a Shape", argument_index);
  }
  return *s;
}

//...........................................................................
// Shape, lua interface.

int Shape::Index(lua_State *L) {
  CHECK(lua_gettop(L) >= 1);
  if (lua_type(L, -1) == LUA_TSTRING) {
    // @@@ We could probably use a lua table to make dispatch faster here.
    const char *s = lua_tostring(L, -1);
    if (strcmp(s, "Clone") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaClone>));
    } else if (strcmp(s, "AddPoint") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaAddPoint>));
    } else if (strcmp(s, "MakePolyline") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaMakePolyline>));
    } else if (strcmp(s, "Clean") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaClean>));
    } else if (strcmp(s, "Contains") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaContains>));
    } else if (strcmp(s, "Offset") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaOffset>));
    } else if (strcmp(s, "Scale") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaScale>));
    } else if (strcmp(s, "Rotate") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaRotate>));
    } else if (strcmp(s, "MirrorX") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaMirrorX>));
    } else if (strcmp(s, "MirrorY") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaMirrorY>));
    } else if (strcmp(s, "Reverse") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaReverse>));
    } else if (strcmp(s, "Grow") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaGrow>));
    } else if (strcmp(s, "Select") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaSelect>));
    } else if (strcmp(s, "SelectAll") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaSelectAll>));
    } else if (strcmp(s, "Port") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaPort>));
    } else if (strcmp(s, "ABC") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaABC>));
    } else if (strcmp(s, "APointInside") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaAPointInside>));
    } else if (strcmp(s, "FilletVertex") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaFilletVertex>));
    } else if (strcmp(s, "ChamferVertex") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaChamferVertex>));
    } else if (strcmp(s, "Paint") == 0) {  // __Paint__ in user_script_util.lua
      lua_getglobal(L, "__Paint__");       // calls RawPaint defined here.
    } else if (strcmp(s, "RawPaint") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaPaint>));
    } else if (strcmp(s, "HasPorts") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaHasPorts>));
    } else if (strcmp(s, "LoadSTL") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaLoadSTL>));
    } else if (strcmp(s, "empty") == 0) {
      lua_pushboolean(L, IsEmpty());
    } else if (strcmp(s, "pieces") == 0) {
      lua_pushinteger(L, NumPieces());
    } else if (strcmp(s, "area") == 0) {
      lua_pushnumber(L, TotalArea());
    } else if (strcmp(s, "orientation") == 0) {
      if (polys_.size() != 1) {
        LuaError(L, "Attempt to determine orientation in a shape with "
                    PRINTF_SIZET" pieces (one piece required)", polys_.size());
      }
      lua_pushboolean(L, Orientation(0));
    } else if (strcmp(s, "self_intersection") == 0) {
      lua_pushboolean(L, SelfIntersection());
    } else if (strcmp(s, "bounds") == 0) {
      if (IsEmpty()) {
        LuaError(L, "Can not compute bounds of empty shape");
      }
      JetNum min_x, min_y, max_x, max_y;
      GetBounds(&min_x, &min_y, &max_x, &max_y);
      lua_createtable(L, 0, 4);
      lua_pushnumber(L, min_x);
      LuaRawSetField(L, -2, "min_x");
      lua_pushnumber(L, min_y);
      LuaRawSetField(L, -2, "min_y");
      lua_pushnumber(L, max_x);
      LuaRawSetField(L, -2, "max_x");
      lua_pushnumber(L, max_y);
      LuaRawSetField(L, -2, "max_y");
    } else if (strcmp(s, "material") == 0) {
      if (polys_.size() != 1) {
        LuaError(L, "material requested for shape without exactly one piece");
      }
      lua_newtable(L);
      lua_pushstring(L, "color");
      lua_pushnumber(L, polys_[0].material.color);
      lua_rawset(L, -3);
      lua_pushstring(L, "epsilon");
      lua_getglobal(L, "Complex");
      lua_pushnumber(L, polys_[0].material.epsilon.real());
      lua_pushnumber(L, polys_[0].material.epsilon.imag());
      lua_call(L, 2, 1);
      lua_rawset(L, -3);
    } else {
      LuaError(L, "Unknown shape field '%s'", s);
    }
  } else if (lua_type(L, -1) == LUA_TNUMBER) {
    if (polys_.size() != 1) {
      LuaError(L, "Attempt to index vertices in a shape with " PRINTF_SIZET
                  " pieces (one piece required)", polys_.size());
    }
    int i = ToDouble(lua_tonumber(L, -1));
    if (i != ToDouble(lua_tonumber(L, -1)) || i < 1 || i > polys_[0].p.size()) {
      LuaError(L, "Vertex index must an an integer in the range 1.."
               PRINTF_SIZET" (is %g)", polys_[0].p.size(),
               ToDouble(lua_tonumber(L, -1)));
    }
    // Keep the fields below consistent with AddPoint(), which knows how to
    // consume them.
    lua_createtable(L, 0, 2);
    lua_pushnumber(L, polys_[0].p[i - 1].p[0]);
    LuaRawSetField(L, -2, "x");
    lua_pushnumber(L, polys_[0].p[i - 1].p[1]);
    LuaRawSetField(L, -2, "y");
    lua_pushnumber(L, polys_[0].p[i - 1].e.kind[0].IntegerForDebugging());
    LuaRawSetField(L, -2, "kind0");
    lua_pushnumber(L, polys_[0].p[i - 1].e.kind[1].IntegerForDebugging());
    LuaRawSetField(L, -2, "kind1");
    lua_pushnumber(L, polys_[0].p[i - 1].e.dist[0]);
    LuaRawSetField(L, -2, "dist0");
    lua_pushnumber(L, polys_[0].p[i - 1].e.dist[1]);
    LuaRawSetField(L, -2, "dist1");
  } else {
    LuaError(L, "Shape must be indexed with string not %s",
             luaL_typename(L, -1));
  }
  return 1;
}

int Shape::FunctionCall(lua_State *L) {
  // With a single integer argument, select the given piece (1-based indexing).
  if (lua_gettop(L) == 2 && lua_type(L, 2) == LUA_TNUMBER) {
    int n = ToDouble(lua_tonumber(L, 2));
    if (n != ToDouble(lua_tonumber(L, 2)) || n < 1 || n > polys_.size()) {
      LuaError(L, "shape(%g) is invalid, the valid range is an integer 1.."
               PRINTF_SIZET, ToDouble(lua_tonumber(L, 2)), polys_.size());
    }
    LuaUserClassCreateObj<Shape>(L)->SetToPiece(n - 1, *this);
    return 1;
  } else {
    LuaError(L, "Shape function call is not understood. "
                "Use shape(n) to select a piece.");
  }
}

int Shape::Length(lua_State *L) {
  // #shape is an index count for a single piece polygon (0 for empty polygon).
  // It's an error if this is not a single piece polygon.
  if (polys_.size() > 1) {
    LuaError(L, "# length operator applied to a shape with more than one "
                "piece");
  }
  if (polys_.size() == 0) {
    lua_pushinteger(L, 0);
  } else {
    lua_pushinteger(L, polys_[0].p.size());
  }
  return 1;
}

bool Shape::Operator(lua_State *L, int op, int pos) {
  if (op == LUA_OPADD || op == LUA_OPSUB || op == LUA_OPMUL ||
      op == LUA_OPBXOR) {
    Shape *op1 = LuaCastTo<Shape>(L, 1);
    Shape *op2 = LuaCastTo<Shape>(L, 2);
    if (!op1 || !op2 || pos != 1) {
      // Binary operands must both be Shapes. If pos != 1 then the first
      // operand wasn't.
      LuaError(L, "Both arguments to the operator must be Shape objects");
    }
    Shape *result = LuaUserClassCreateObj<Shape>(L);
    if (op == LUA_OPADD) {
      result->SetUnion(*op1, *op2);
    } else if (op == LUA_OPSUB) {
      result->SetDifference(*op1, *op2);
    } else if (op == LUA_OPMUL) {
      result->SetIntersect(*op1, *op2);
    } else if (op == LUA_OPBXOR) {
      result->SetXOR(*op1, *op2);
    } else {
      LuaError(L, "Internal");
    }
    return true;
  } else if (op == LUA_OPCONCAT) {
    Shape *op1 = LuaCastTo<Shape>(L, 1);
    Shape *op2 = LuaCastTo<Shape>(L, 2);
    if (!op1 || !op2 || pos != 1) {
      // Binary operands must both be Shapes. If pos != 1 then the first
      // operand wasn't.
      LuaError(L, "Both arguments to the '..' operator must be Shape objects");
    }
    Shape *result = LuaUserClassCreateObj<Shape>(L);
    result->polys_ = op1->polys_;
    result->polys_.insert(result->polys_.end(),
                          op2->polys_.begin(), op2->polys_.end());
    result->CombinePortCallbacks(op1, op2);
    return true;
  } else {
    return false;
  }
}

int Shape::LuaClone(lua_State *L) {
  Expecting(L, 1, "Clone");
  Shape *s = LuaUserClassCreateObj<Shape>(L);
  *s = *this;
  return 1;
}

int Shape::LuaAddPoint(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  if (lua_gettop(L) == 3) {
    AddPoint(luaL_checknumber(L, 2), luaL_checknumber(L, 3));
  } else if (lua_gettop(L) == 2 && lua_type(L, 2) == LUA_TTABLE) {
    // Keep the fields below consistent with the shape index operator, which
    // knows how to emit them.
    EdgeInfo e;
    lua_pushstring(L, "x");                                   lua_rawget(L, -2);
    JetNum x = lua_tonumber(L, -1);                           lua_pop(L, 1);
    lua_pushstring(L, "y");                                   lua_rawget(L, -2);
    JetNum y = lua_tonumber(L, -1);                           lua_pop(L, 1);
    lua_pushstring(L, "kind0");                               lua_rawget(L, -2);
    e.kind[0].SetFromInteger(ToDouble(lua_tonumber(L, -1)));  lua_pop(L, 1);
    lua_pushstring(L, "kind1");                               lua_rawget(L, -2);
    e.kind[1].SetFromInteger(ToDouble(lua_tonumber(L, -1)));  lua_pop(L, 1);
    lua_pushstring(L, "dist0");                               lua_rawget(L, -2);
    e.dist[0] = ToDouble(lua_tonumber(L, -1));                lua_pop(L, 1);
    lua_pushstring(L, "dist1");                               lua_rawget(L, -2);
    e.dist[1] = ToDouble(lua_tonumber(L, -1));                lua_pop(L, 1);
    AddPoint(x, y, &e);
  } else {
    LuaError(L, "Shape:AddPoint() expecting arguments (x,y) or (table)");
  }
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaMakePolyline(lua_State *L) {
  Expecting(L, 1, "MakePolyline");
  MakePolyline();
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaClean(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  if (lua_gettop(L) == 4) {
    int mode = ToInt64(luaL_checknumber(L, 4));
    if (mode != 1 && mode != 2) {
      LuaError(L, "Shape:Clean() mode must be 1 or 2");
    }
    Clean(luaL_checknumber(L, 2), luaL_checknumber(L, 3), mode);
  } else {
    Expecting(L, 2, "Clean");
    Clean(luaL_checknumber(L, 2));
  }
  return 1;
}

int Shape::LuaContains(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  Expecting(L, 3, "Contains");
  lua_pushboolean(L,
      Contains(luaL_checknumber(L, 2), luaL_checknumber(L, 3)) != 0);
  return 1;
}

int Shape::LuaOffset(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  Expecting(L, 3, "Offset");
  Offset(luaL_checknumber(L, 2), luaL_checknumber(L, 3));
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaScale(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  if (lua_gettop(L) == 2) {
    Scale(luaL_checknumber(L, 2), luaL_checknumber(L, 2));
  } else if (lua_gettop(L) == 3) {
    Scale(luaL_checknumber(L, 2), luaL_checknumber(L, 3));
  } else {
    LuaError(L, "Shape:Scale() expecting 1 or 2 arguments");
  }
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaRotate(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  Expecting(L, 2, "Rotate");
  Rotate(luaL_checknumber(L, 2));
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaMirrorX(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  Expecting(L, 2, "MirrorX");
  MirrorX(luaL_checknumber(L, 2));
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaMirrorY(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  Expecting(L, 2, "MirrorY");
  MirrorY(luaL_checknumber(L, 2));
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaReverse(lua_State *L) {
  Expecting(L, 1, "Reverse");
  Reverse();
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaGrow(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  CornerStyle endcap_style = BUTT;
  if (lua_gettop(L) == 5) {
    endcap_style = CheckStyle(L, 5, true, "Grow");
  } else {
    Expecting(L, 4, "Grow");
  }
  CornerStyle style = CheckStyle(L, 3, false, "Grow");
  JetNum limit = luaL_checknumber(L, 4);
  if (style == ROUND && fabs(limit) < kTol) {
    LuaError(L, "limit for 'round' style can not be zero");
  }
  Grow(luaL_checknumber(L, 2), style, limit, endcap_style);
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaSelect(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  if (lua_gettop(L) != 3 && lua_gettop(L) != 5) {
    LuaError(L, "Shape:Select() expecting 2 or 4 arguments");
  }
  if (IsEmpty()) {
    LuaError(L, "Shape:Select() called on an empty shape");
  }
  JetNum x1 = luaL_checknumber(L, 2);
  JetNum y1 = luaL_checknumber(L, 3);
  if (lua_gettop(L) == 3) {
    int piece, edge;
    FindClosestEdge(x1, y1, &piece, &edge);
    lua_createtable(L, 0, 2);
    lua_pushnumber(L, piece + 1);       // 1-based indexing
    LuaRawSetField(L, -2, "p");
    lua_pushnumber(L, edge + 1);        // 1-based indexing
    LuaRawSetField(L, -2, "e");
    return 1;
  } else {
    JetNum x2 = luaL_checknumber(L, 4);
    JetNum y2 = luaL_checknumber(L, 5);
    JetNum xmin = std::min(x1, x2);
    JetNum xmax = std::max(x1, x2);
    JetNum ymin = std::min(y1, y2);
    JetNum ymax = std::max(y1, y2);
    lua_createtable(L, 0, 0);           // Result table
    int count = 1;
    for (int i = 0; i < polys_.size(); i++) {
      for (int j1 = 0; j1 < polys_[i].p.size(); j1++) {
        int j2 = (j1 + 1) % polys_[i].p.size();
        if (DoesLineIntersectBox(polys_[i].p[j1].p, polys_[i].p[j2].p,
                                 xmin, xmax, ymin, ymax)) {
          lua_createtable(L, 0, 2);
          lua_pushnumber(L, i + 1);     // Edge (1-based indexing)
          LuaRawSetField(L, -2, "p");
          lua_pushnumber(L, j1 + 1);    // Edge (1-based indexing)
          LuaRawSetField(L, -2, "e");
          lua_rawseti(L, -2, count++);
        }
      }
    }
    return 1;
  }
}

int Shape::LuaSelectAll(lua_State *L) {
  Expecting(L, 1, "SelectAll");
  lua_pushnumber(L, -1e99);
  lua_pushnumber(L, -1e99);
  lua_pushnumber(L,  1e99);
  lua_pushnumber(L,  1e99);
  return LuaSelect(L);
}

// Magic argument passed to LuaPort() to indicate an ABC, only used by
// LuaABC().
enum { MAGIC_ABC_PORT = -15485863 };

int Shape::LuaPort(lua_State *L) {
  // Trim off nils at the end before we start counting arguments.
  LuaErrorIfNaNOrInfs(L);
  while (lua_gettop(L) >= 1 && lua_type(L, lua_gettop(L)) == LUA_TNIL) {
    lua_pop(L, 1);
  }

  // Check that this is called with the arguments:
  //   1: {p=#, e=#} table (piece and edge), or an array of such tables
  //   2: port number
  //   3: optional callback function
  // to mark the edge from vertex e to vertex e+1 as a port.
  if (lua_gettop(L) < 3 || lua_gettop(L) > 4) {
    LuaError(L, "Shape:Port() expecting 2 or 3 arguments");
  }
  if (lua_gettop(L) == 4 && lua_type(L, 4) != LUA_TFUNCTION) {
    LuaError(L, "Shape:Port() third argument must be a function");
  }
  EdgeKind edge_kind;
  int port_number = ToDouble(luaL_checknumber(L, 3));
  if (port_number >= 1 && port_number <= EdgeKind::MaxPort() &&
      port_number == ToDouble(luaL_checknumber(L, 3))) {
    edge_kind = EdgeKind(port_number);
  } else if (port_number == MAGIC_ABC_PORT) {
    edge_kind.SetABC();
  } else {
    LuaError(L, "Invalid port number (should be an integer in the range 1..%d)",
             EdgeKind::MaxPort());
  }
  if (lua_gettop(L) == 4) {
    // Save the port callback, if any.
    port_callbacks_[port_number] = LuaCallback(L);
  }
  vector<int> piece(1), edge(1);
  if (!GetPieceEdge(L, 2, &piece[0], &edge[0])) {
    if (!GetPieceEdgeArray(L, 2, &piece, &edge)) {
      LuaError(L, "Argument to Port() must be a {p=#, e=#} table, or an array "
                  "of such tables");
    }
  }

  // Map point coordinates to all piece,index values that have those
  // coordinates.
  std::map<std::pair<JetNum, JetNum>, vector<std::pair<int,int>>> point_map;
  for (int i = 0; i < polys_.size(); i++) {
    for (int j = 0; j < polys_[i].p.size(); j++) {
      auto &p = polys_[i].p[j].p;
      point_map[std::make_pair(p[0], p[1])].push_back(std::make_pair(i, j));
    }
  }

  // Assign ports for all piece,edge values. It is possible to have coincident
  // (duplicate) points where unmerged polygons with different material types
  // come together. AssignPort() will make sure that all coincident vertices
  // have the same EdgeInfo.
  for (int i = 0; i < piece.size(); i++) {
    if (piece[i] < 1 || piece[i] > polys_.size()) {
      LuaError(L, "Argument to Port() has p=%d but there are only %d pieces",
               piece[i], (int) polys_.size());
    }
    int num_edges = polys_[piece[i] - 1].p.size();
    if (edge[i] < 1 || edge[i] > num_edges) {
      LuaError(L, "Argument to Port() has e=%d but piece %d has only %d edges",
               edge[i], piece[i], num_edges);
    }
    // By the definition of EdgeInfo, both vertices that bound the edge have to
    // be marked.
    if (!AssignPort(piece[i] - 1, edge[i] - 1, edge_kind, &point_map)) {
      LuaError(L, "Piece %d edge %d already has a port number",
               piece[i], edge[i]);
    }
  }
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaABC(lua_State *L) {
  // Like LuaPort(), but use the special port number that designates an ABC.
  LuaErrorIfNaNOrInfs(L);
  Expecting(L, 2, "ABC");
  lua_pushnumber(L, MAGIC_ABC_PORT);
  return LuaPort(L);
}

int Shape::LuaAPointInside(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  Expecting(L, 1, "APointInside");
  double x, y;
  if (!APointInside(-1, &x, &y)) {
    LuaError(L, "APointInside() requires a nonempty single piece polygon of "
                "any orientation, or a single positive area polygon with any "
                "number of negative area holes.");
  }
  lua_pushnumber(L, x);
  lua_pushnumber(L, y);
  return 2;
}

int Shape::LuaFilletVertex(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  bool mutate = true;
  if (lua_gettop(L) == 6) {
    mutate = lua_toboolean(L, 6);
    lua_settop(L, 5);
  }
  Expecting(L, 5, "FilletVertex");
  if (IsEmpty()) {
    LuaError(L, "FilletVertex() requires a nonempty shape");
  }
  JetPoint pstart, pend, center;
  if (FilletVertex(luaL_checknumber(L, 2), luaL_checknumber(L, 3),
                   luaL_checknumber(L, 4), luaL_checknumber(L, 5),
                   &pstart, &pend, &center, mutate)) {
    LuaVector *result[3];
    for (int i = 0; i < 3; i++) {
      result[i] = LuaUserClassCreateObj<LuaVector>(L);
      result[i]->resize(2);
    }
    for (int i = 0; i < 2; i++) {
      (*result[0])[i] = pstart[i];
      (*result[1])[i] = pend[i];
      (*result[2])[i] = center[i];
    }
    return 3;
  } else {
    return 0;
  }
}

int Shape::LuaChamferVertex(lua_State *L) {
  LuaErrorIfNaNOrInfs(L);
  Expecting(L, 5, "ChamferVertex");
  if (IsEmpty()) {
    LuaError(L, "ChamferVertex() requires a nonempty shape");
  }
  JetPoint p1, p2;
  ChamferVertex(luaL_checknumber(L, 2), luaL_checknumber(L, 3),
                luaL_checknumber(L, 4), luaL_checknumber(L, 5), &p1, &p2);
  lua_pushnumber(L, p1[0]);
  lua_pushnumber(L, p1[1]);
  lua_pushnumber(L, p2[0]);
  lua_pushnumber(L, p2[1]);
  return 4;
}

int Shape::LuaPaint(lua_State *L) {
  // Trim off nils at the end before we start counting arguments.
  LuaErrorIfNaNOrInfs(L);
  while (lua_gettop(L) >= 1 && lua_type(L, lua_gettop(L)) == LUA_TNIL) {
    lua_pop(L, 1);
  }
  if (lua_gettop(L) < 4) {
    LuaError(L, "Expecting shape1:Paint(shape2, color, param, ...)");
  }
  CHECK(this == LuaCastTo<Shape>(L, 1));
  Shape *s2 = LuaCastTo<Shape>(L, 2);
  if (!s2) {
    LuaError(L, "Expecting shape1:Paint(shape2, color, param, ...)");
  }
  Material mat;
  mat.color = ToInt64(luaL_checknumber(L, 3));
  if (lua_type(L, 4) == LUA_TFUNCTION) {
    if (lua_gettop(L) != 4) {
      LuaError(L, "Expecting shape1:Paint(shape2, color, function)");
    }
    // Store the function to the registry and add the registry key to the
    // material.
    lua_pushvalue(L, -1);
    CHECK(!mat.callback.Valid());       // Should only do this once
    mat.callback = LuaCallback(L);

    // Run the callback function once to verify it can work. This gives an
    // early indication for some (though not all) runtime errors.
    LuaVector *x = LuaUserClassCreateObj<LuaVector>(L);
    LuaVector *y = LuaUserClassCreateObj<LuaVector>(L);
    x->resize(10);
    y->resize(10);
    vector<MaterialParameters> result;
    if (!Material::RunCallback(LuaGetObject(L), true, &result)) {
      LuaError(L, "Callback function can not be run");
    }
  } else {
    int num_params = lua_gettop(L) - 3;
    if (num_params != 1 && num_params != 4 && num_params != 5) {
      if (num_params == 2) {
        LuaError(L, "Two material parameters were given so you might be using "
            "a deprecated complex number API. Use Complex() instead.");
      } else {
        LuaError(L, "Expecting 1, 4 or 5 material parameters");
      }
    }
    vector<JetComplex> list;
    for (int i = 0; i < num_params; i++) {
      JetComplex value;
      if (!ToJetComplex(L, i + 4, &value)) {
        LuaError(L, "Material parameter %d is not a real or complex number",
                 i + 1);
      }
      list.push_back(value);
    }
    mat.SetParameters(list);
  }
  Paint(*s2, mat);
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaHasPorts(lua_State *L) {
  for (int i = 0; i < polys_.size(); i++) {
    for (int j = 0; j < polys_[i].p.size(); j++) {
      if (!polys_[i].p[j].e.IsDefault()) {
        lua_pushboolean(L, 1);
        return 1;
      }
    }
  }
  lua_pushboolean(L, 0);
  return 1;
}

int Shape::LuaLoadSTL(lua_State *L) {
  if (lua_gettop(L) != 2) {
    LuaError(L, "Expecting shape:LoadSTL(filename)");
  }
  const char *filename = lua_tostring(L, -1);
  LoadSTL(filename);
  lua_settop(L, 1);
  return 1;
}

//***************************************************************************
// Testing.

// Create a shape with M random vertices. Ensure that adjacent vertices are no
// closer than 'min_dist'.
static void RandomShape1(Shape *s, int M, bool positive_area_only,
                         double min_dist = 0) {
  double last_x = 10, last_y = 10;
  for (int i = 0; i < M; i++) {
    double x, y;
    do {
      x = RandDouble();
      y = RandDouble();
    } while (hypot(x - last_x, y - last_y) < min_dist);
    last_x = x;
    last_y = y;
    s->AddPoint(x, y);
  }
  if (positive_area_only) {
    if (s->Area(0) < 0) {
      s->Reverse();
    }
  }
}

// Create a shape with one positive area piece and no more than H holes by
// combining a number of rectangles.
static void RandomShape2(Shape *s, int H) {
  const int N = 4;              // Number of rectangles to combine
  while (true) {
    s->Clear();
    for (int i = 0; i < N; i++) {
      double w = RandDouble();
      double h = RandDouble();
      Shape r;
      r.SetRectangle(-w/2, -h/2, w/2, h/2);
      r.Rotate(RandDouble() * 360);
      r.Offset(RandDouble(), RandDouble());
      if (i == N - 1) {
        // Subtract the last rectangle to help make holes.
        s->SetDifference(*s, r);
      } else {
        s->SetUnion(*s, r);
      }
    }
    int posarea_pieces = 0, negarea_pieces = 0;
    for (int i = 0; i < s->NumPieces(); i++) {
      if (s->Area(i) > 0) {
        posarea_pieces++;
      } else {
        negarea_pieces++;
      }
    }
    if (posarea_pieces == 1 && negarea_pieces <= H) {
      return;
    }
  }
}

// Check if the centroid of piece 'i' is the expected value
static bool CentroidIs(const Shape &s, int i, double x, double y) {
  JetPoint c(0, 0);
  for (int j = 0; j < s.Piece(i).size(); j++) {
    c += s.Piece(i)[j].p;
  }
  c /= double(s.Piece(i).size());
  return c[0] == x && c[1] == y;
}

TEST_FUNCTION(Area) {
  {
    Shape s;
    s.SetRectangle(1, 1, 10, 5);
    CHECK(s.Area(0) == 36);

    // Test mirroring preserves area.
    s.MirrorX(4);
    CHECK(s.Area(0) == 36);
    s.MirrorY(2);
    CHECK(s.Area(0) == 36);

    // Test reversing negates area.
    s.Reverse();
    CHECK(s.Area(0) == -36);
  }

  // Test triangle areas.
  for (int i = 0; i < 100; i++) {
    double ax = RandDouble();
    double ay = RandDouble();
    double bx = RandDouble();
    double by = RandDouble();
    double cx = RandDouble();
    double cy = RandDouble();
    double triarea = -0.5 * ((cx + ax) * (cy - ay) + (ax + bx) * (ay - by) +
                             (bx + cx) * (by - cy));
    Shape s;
    s.AddPoint(ax, ay);
    s.AddPoint(bx, by);
    s.AddPoint(cx, cy);
    CHECK(fabs(s.Area(0) - triarea) < 1e-9);
  }
}

TEST_FUNCTION(TriangleArea) {
  for (int i = 0; i < 1000; i++) {
    Shape s;
    RandomShape1(&s, 3, false);
    const vector<RPoint> &points = s.Piece(0);
    JetNum area1 = s.Area(0);
    JetNum area2 = TriangleArea(points[0].p, points[1].p, points[2].p);
    CHECK(fabs(area1 - area2) < 1e-9);
  }
}

TEST_FUNCTION(Clean) {
  // Close colinear vertices.
  for (int i = 0; i < 100; i++) {
    Shape s1;
    RandomShape1(&s1, 10, false, 1e-3);

    // Add close colinear vertices to s1 to get s2.
    Shape s2;
    const vector<RPoint> &p = s1.Piece(0);
    for (int j1 = 0; j1 < p.size(); j1++) {
      int j2 = (j1 + 1) % p.size();
      s2.AddPoint(p[j1].p[0], p[j1].p[1]);
      JetPoint q = p[j1].p + (p[j2].p - p[j1].p) * 0.00001;
      s2.AddPoint(q[0], q[1]);
      q = p[j1].p + (p[j2].p - p[j1].p) * 0.99999;
      s2.AddPoint(q[0], q[1]);
    }

    // Clean out the close colinear vertices in s2, we should be back to s1.
    s2.Clean(1e-3);
    CHECK(s1 == s2);
  }

  // General close vertices.
  for (int i = 0; i < 100; i++) {
    Shape s1;
    RandomShape1(&s1, 10, false);

    // Add close vertices to s1 to get s2.
    Shape s2;
    const vector<RPoint> &p1 = s1.Piece(0);
    for (int j = 0; j < p1.size(); j++) {
      s2.AddPoint(p1[j].p[0], p1[j].p[1]);
      s2.AddPoint(p1[j].p[0] + RandDouble()*1e-6,
                  p1[j].p[1] + RandDouble()*1e-6);
    }

    // Clean out the close vertices in s2, we should be back to something that
    // is close to s1.
    s2.Clean(1e-3);
    const vector<RPoint> &p2 = s2.Piece(0);
    CHECK(p1.size() == p2.size());
    for (int j = 0; j < p1.size(); j++) {
      CHECK(fabs(p1[j].p[0] - p2[j].p[0]) < 2e-6);
      CHECK(fabs(p1[j].p[1] - p2[j].p[1]) < 2e-6);
    }
  }
}

TEST_FUNCTION(APointInside) {
  for (int i = 0; i < 1000; i++) {
    {
      // Create a shape with one positive area piece and any number of holes.
      Shape s;
      RandomShape2(&s, 1000);

      // Test APointInside.
      double px, py;
      CHECK(s.APointInside(-1, &px, &py));
      CHECK(s.Contains(px, py) == 1);
    }

    {
      // Create a shape with one positive area piece and no holes.
      Shape s;
      RandomShape2(&s, 0);

      // Test APointInside.
      double px, py;
      CHECK(s.APointInside(-1, &px, &py));
      CHECK(s.Contains(px, py) == 1);

      // Test reverse orientation polys too.
      Shape s2;
      s2 = s;
      s2.Reverse();
      CHECK(s2.APointInside(-1, &px, &py));
      CHECK(s.Contains(px, py) == 1);
    }

    {
      // Test triangles, make sure this simple case works too.
      Shape s;
      RandomShape1(&s, 3, true);
      double px, py;
      CHECK(s.APointInside(-1, &px, &py));
      CHECK(s.Contains(px, py) == 1);
      Shape s2;
      s2 = s;
      s2.Reverse();
      CHECK(s2.APointInside(-1, &px, &py));
      CHECK(s.Contains(px, py) == 1);
    }
  }
}

TEST_FUNCTION(TriangleIntersectsBox) {
  for (int i = 0; i < 1000; i++) {
    // Make a random box.
    double xmin = RandDouble() - 0.5, ymin = RandDouble() - 0.5;
    double xmax = xmin + RandDouble(), ymax = ymin + RandDouble();

    // Make a random triangle (with positive or negative area).
    Shape tri;
    RandomShape1(&tri, 3, true);
    const vector<RPoint> &points = tri.Piece(0);
    const JetPoint *pp[3];
    for (int j = 0; j < 3; j++) {
      pp[j] = &points[j].p;
    }

    // Intersect the triangle and box the slow way.
    Shape rect, trirect;
    rect.SetRectangle(xmin, ymin, xmax, ymax);
    trirect.SetIntersect(tri, rect);

    // Sometimes pass negative area triangles to TriangleIntersectsBox().
    if (RandDouble() > 0.5) {
      const JetPoint *tmp = pp[0];
      pp[0] = pp[1];
      pp[1] = tmp;
    }

    // Check that TriangleIntersectsBox() gives the same result.
    bool intersects = TriangleIntersectsBox(pp, xmin, xmax, ymin, ymax);
    CHECK(intersects == !trirect.IsEmpty());
  }
}

TEST_FUNCTION(SplitPolygonsAtNecks) {
  {
    // Two squares in a row.
    Shape s;
    s.AddPoint(0, 0);
    s.AddPoint(1, 0);
    s.AddPoint(1, 1);     // Duplicated point
    s.AddPoint(2, 1);
    s.AddPoint(2, 2);
    s.AddPoint(1, 2);
    s.AddPoint(1, 1);     // Duplicated point
    s.AddPoint(0, 1);
    s.SplitPolygonsAtNecks();
    CHECK(s.NumPieces() == 2);
    CHECK(s.Area(0) == 1);
    CHECK(s.Area(1) == 1);
    CHECK(CentroidIs(s, 0, 0.5, 0.5));
    CHECK(CentroidIs(s, 1, 1.5, 1.5));
  }
  {
    // Three squares in a row.
    Shape s;
    s.AddPoint(0, 0);
    s.AddPoint(1, 0);
    s.AddPoint(1, 1);     // Duplicated point
    s.AddPoint(2, 1);
    s.AddPoint(2, 2);     // Duplicated point
    s.AddPoint(3, 2);
    s.AddPoint(3, 3);
    s.AddPoint(2, 3);
    s.AddPoint(2, 2);     // Duplicated point
    s.AddPoint(1, 2);
    s.AddPoint(1, 1);     // Duplicated point
    s.AddPoint(0, 1);
    s.SplitPolygonsAtNecks();
    CHECK(s.NumPieces() == 3);
    CHECK(s.Area(0) == 1);
    CHECK(s.Area(1) == 1);
    CHECK(s.Area(2) == 1);
    CHECK(CentroidIs(s, 0, 1.5, 1.5));
    CHECK(CentroidIs(s, 1, 2.5, 2.5));
    CHECK(CentroidIs(s, 2, 0.5, 0.5));
  }
  {
    // Three squares in a star.
    Shape s;
    s.AddPoint(0, 0);
    s.AddPoint(1, 0);
    s.AddPoint(1, 1);     // Duplicated point
    s.AddPoint(2, 1);
    s.AddPoint(2, 2);
    s.AddPoint(1, 2);
    s.AddPoint(1, 1);     // Duplicated point
    s.AddPoint(1.5, 0);
    s.AddPoint(2, 0.5);
    s.AddPoint(1, 1);     // Duplicated point
    s.AddPoint(0, 1);
    s.SplitPolygonsAtNecks();
    CHECK(s.NumPieces() == 3);
    CHECK(s.Area(0) == 1);
    CHECK(s.Area(1) == 1);
    CHECK(s.Area(2) == 0.375);
    CHECK(CentroidIs(s, 0, 0.5, 0.5));
    CHECK(CentroidIs(s, 1, 1.5, 1.5));
    CHECK(CentroidIs(s, 2, 1.5, 0.5));
  }
  {
    // Square with sliver.
    Shape s;
    s.AddPoint(0, 0);
    s.AddPoint(1, 0);
    s.AddPoint(1, 1);
    s.AddPoint(2, 1);
    s.AddPoint(1, 1);
    s.AddPoint(0, 1);
    CHECK(s.Area(0) == 1);
    s.SplitPolygonsAtNecks();
    CHECK(s.NumPieces() == 1);
    CHECK(s.Piece(0).size() == 4);
    CHECK(s.Area(0) == 1);
    CHECK(CentroidIs(s, 0, 0.5, 0.5));
  }
}

TEST_FUNCTION(DoesLineIntersectLine) {
  for (int iter = 0; iter < 10000; iter++) {
    JetPoint p1, p2, p3, p4;
    for (int i = 0; i < 2; i++) {
      p1[i] = RandDouble();
      p2[i] = RandDouble();
      p3[i] = RandDouble();
      p4[i] = RandDouble();
    }
    bool result1 = DoesLineIntersectLine(p1, p2, p3, p4);

    // Now compute the actual intersection point a different way and compare.
    JetNum x1 = p1[0];
    JetNum y1 = p1[1];
    JetNum x2 = p3[0];
    JetNum y2 = p3[1];
    JetNum dx1 = p2[0] - p1[0];
    JetNum dy1 = p2[1] - p1[1];
    JetNum dx2 = p4[0] - p3[0];
    JetNum dy2 = p4[1] - p3[1];
    JetNum a1 = (dy2*x1 - dy2*x2 - dx2*y1 + dx2*y2) / (dx2*dy1 - dx1*dy2);
    JetNum a2 = (dy1*x1 - dy1*x2 - dx1*y1 + dx1*y2) / (dx2*dy1 - dx1*dy2);
    bool result2 = a1 >= 0 && a1 <= 1 && a2 >= 0 && a2 <= 1;
    CHECK(result1 == result2);
  }
}
