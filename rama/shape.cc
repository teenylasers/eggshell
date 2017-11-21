
#include <math.h>
#include <algorithm>
#include "common.h"
#include "shape.h"
#include "mesh.h"
#include "gl_utils.h"
#include "shaders.h"
#include "wxgl_font.h"
#include "testing.h"

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

// All clipper coordinates are 64 bit integers. Our double precision
// coordinates are scaled to use about 32 of those bits. We used to use 55 bits
// here, but that resulted in some kind of problem that caused invalid polygons
// to be generated in some cases. 32 bits is not a problem: for a polygon with
// a 1m x 1m boundary the quantization size (1/2^32) is 233 pm, on the order of
// the atomic spacing in a crystal lattice.
const double kCoordScale = 4294967296;  // =2^32

// Geometry tolerances
const double kTol = 1e-20;              // Generic
const double kTolColinear = 1.74e-8;    // 1 micro-degree (in radians)
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
  CHECK(best_ia >= 0);        // If can't find convex vertex (never happens)
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

  // Return q if we found it or the abv midpoint otherwise.
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

void Material::SetCallbackToRegistry(lua_State *L) {
  CHECK(lua_type(L, -1) == LUA_TFUNCTION);
  CHECK(callback.empty());                      // Should only do this once
  LuaGetObject(L)->Hash(&callback, true);       // Does not pop function
  lua_pushlstring (L, callback.data(), callback.size());
  lua_rotate(L, -2, 1);                         // Swap top 2 elements
  lua_rawset(L, LUA_REGISTRYINDEX);
}

void Material::GetCallbackFromRegistry(lua_State *L) {
  CHECK(callback.size() == 16);                 // Make sure it's an MD5 hash
  lua_pushlstring (L, callback.data(), callback.size());
  lua_rawget(L, LUA_REGISTRYINDEX);
  CHECK(lua_type(L, -1) == LUA_TFUNCTION);
}

bool Material::RunCallback(Lua *lua, LuaVector *result[2]) {
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
  if (nret < 1 || nret > 2) {
    LuaError(lua->L(),"Callback should return 1 or 2 values");
    return false;
  }
  result[0] = 0;
  result[1] = 0;
  for (int i = 0; i < nret; i++) {
    result[i] = LuaCastTo<LuaVector>(lua->L(), n + i);
    if (!result[i] || result[i]->size() != x->size()) {
      LuaError(lua->L(), "Callback should return vectors of the same size as "
                         "the x,y arguments");
      return false;
    }
  }
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
}

void Shape::Clear() {
  polys_.clear();
}

void Shape::Dump() const {
  for (int i = 0; i < polys_.size(); i++) {
    printf("Poly %d:\n", i);
    for (int j = 0; j < polys_[i].p.size(); j++) {
      printf("\t(%g, %g)\n", ToDouble(polys_[i].p[j].p[0]),
                             ToDouble(polys_[i].p[j].p[1]));
    }
  }
}

void Shape::DrawInterior() const {
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
  vector<Vector3f> points, colors;
  gl::PushShader push_shader(gl::SmoothShader());
  for (int i = 0; i < mesh.triangles().size(); i++) {
    uint32 color = materials[mesh.triangles()[i].material].color;
    colors.push_back(Vector3f(((color & 0xff0000) >> 16) / 255.0f,
                              ((color & 0xff00) >> 8   ) / 255.0f,
                              ((color & 0xff)          ) / 255.0f));
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
                         bool show_lines_and_ports, bool show_vertices,
                         double boundary_derivatives_scale) const {
  // First pass: regular polygon outline, regardless of edge kinds.
  if (show_lines_and_ports) {
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
  if (show_lines_and_ports) {
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
    glPointSize(5);
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

void Shape::AddPoint(JetNum x, JetNum y) {
  if (polys_.empty()) {
    polys_.resize(1);
  }
  polys_.back().p.push_back(RPoint(x, y));
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

bool Shape::AssignPort(int piece, int edge, EdgeKind kind) {
  int n = polys_[piece].p.size();
  return polys_[piece].p[ edge         ].e.SetUnused(kind, 1) &&
         polys_[piece].p[(edge + 1) % n].e.SetUnused(kind, 0);
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

void Shape::ExtremeSideLengths(JetNum *ret_length_max,
                               JetNum *ret_length_min) const {
  JetNum length_max = 0;
  JetNum length_min = __DBL_MAX__;
  for (int i = 0; i < polys_.size(); i++) {
    for (int j1 = 0; j1 < polys_[i].p.size(); j1++) {
      int j2 = (j1 + 1) % polys_[i].p.size();
      JetNum length = (polys_[i].p[j2].p - polys_[i].p[j1].p).norm();
      length_max = std::max(length_max, length);
      length_min = std::min(length_min, length);
    }
  }
  *ret_length_max = length_max;
  *ret_length_min = length_min;
}

void Shape::SetRectangle(JetNum x1, JetNum y1, JetNum x2, JetNum y2) {
  polys_.clear();
  polys_.resize(1);
  // Sort coordinates so that we always have a positive area:
  polys_[0].p.push_back(RPoint(std::min(x1,x2), std::min(y1,y2)));
  polys_[0].p.push_back(RPoint(std::max(x1,x2), std::min(y1,y2)));
  polys_[0].p.push_back(RPoint(std::max(x1,x2), std::max(y1,y2)));
  polys_[0].p.push_back(RPoint(std::min(x1,x2), std::max(y1,y2)));
}

void Shape::SetCircle(JetNum x, JetNum y, JetNum radius, int npoints) {
  polys_.clear();
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
  // Compute the coordinate conversion that we're going to use for all
  // clipping. It's important to be consistent so that this shape and the area
  // to paint will end up sharing coincident vertices that will be
  // un-duplicated in Triangulate().
  JetNum offset_x, offset_y, scale;
  if (!ClipperBounds(this, &s, &offset_x, &offset_y, &scale)) {
    return;
  }

  // Subtract 's' separately from each non-hole polygon piece, taking care to
  // preserve the polygon material (color etc) of each piece. We can't simply
  // run the clipper to subtract 's' from all pieces of 'this', since that
  // would unrecoverably merge pieces with different materials.
  Shape holes, not_holes, result;
  for (int i = 0; i < polys_.size(); i++) {
    if (PolyArea(polys_[i].p) > 0) {
      not_holes.polys_.push_back(polys_[i]);
    } else {
      holes.polys_.push_back(polys_[i]);
    }
  }
  for (int i = 0; i < not_holes.polys_.size(); i++) {
    // Each piece we subtract from is one of the not-holes combined with all of
    // the holes. This works because the pftPositive fill type allows the
    // polygon to have external holes that are invisible.
    Shape a, b;
    a.polys_.push_back(not_holes.polys_[i]);
    a.polys_.insert(a.polys_.end(), holes.polys_.begin(), holes.polys_.end());
    b.RunClipper(&a, &s, ctDifference, offset_x, offset_y, scale);
    for (int j = 0; j < b.polys_.size(); j++) {
      b.polys_[j].material = not_holes.polys_[i].material;
    }
    result.polys_.insert(result.polys_.end(), b.polys_.begin(), b.polys_.end());
  }

  // Compute the area to paint and add it to the result as a separate
  // (non-merged) polygon piece.
  Shape area_to_paint;
  area_to_paint.RunClipper(this, &s, ctIntersection, offset_x, offset_y, scale);
  if (IsEmpty()) {
    return;
  }
  for (int i = 0; i < area_to_paint.polys_.size(); i++) {
    area_to_paint.polys_[i].material = mat;
  }
  result.polys_.insert(result.polys_.end(), area_to_paint.polys_.begin(),
                                            area_to_paint.polys_.end());
  polys_.swap(result.polys_);
}

void Shape::SetMerge(const Shape &s) {
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
    if (n < 3) {
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

void Shape::Clean(JetNum threshold) {
  // The default threshold is the maximum side length * kTolClean.
  if (threshold == 0) {
    JetNum length_max, length_min;
    ExtremeSideLengths(&length_max, &length_min);
    threshold = kTolClean * length_max;
  }

  for (int i = 0; i < polys_.size(); i++) {
    if (polys_[i].p.size() < 3) {
      continue;
    }
    for (int pass = 0; pass < 2; pass++) {
      // On the first pass delete only co-linear points that are closer to
      // their neighbors than the threshold. This is done first so that we have
      // the best chance of cleaning up the shape without modifying the
      // boundary. On the second pass delete all points that are closer to
      // their neighbors than the threshold.
      const int n = polys_[i].p.size();
      vector<bool> to_delete(n);
      if (pass == 0) {
        for (int j1 = 0; j1 < n; j1++) {
          int j2 = (j1 + 1) % n;
          int j3 = (j1 + 2) % n;
          JetPoint delta1 = polys_[i].p[j2].p - polys_[i].p[j1].p;
          JetPoint delta2 = polys_[i].p[j3].p - polys_[i].p[j2].p;
          JetNum length1 = delta1.norm();
          JetNum length2 = delta2.norm();
          JetNum sin_theta = Cross2(delta1, delta2) / (length1 * length2);
          if (fabs(sin_theta) < kTolColinear) {
            // j1-j2-j3 are colinear, delete j2 if it's too close to j1 or j2.
            to_delete[j2] = (length1 < threshold || length2 < threshold);
          }
        }
      } else {
        int j1 = n - 1;         // Index of previous undeleted point
        for (int j2 = 0; j2 < n; j2++) {
          JetNum length = (polys_[i].p[j2].p - polys_[i].p[j1].p).norm();
          if (length < threshold) {
            to_delete[j2] = true;
          } else {
            j1 = j2;
          }
        }
      }

      // Actually delete the points.
      int dest = 0;
      for (int src = 0; src < n; src++) {
        if (!to_delete[src]) {
          polys_[i].p[dest++] = polys_[i].p[src];
        }
      }
      polys_[i].p.resize(dest);
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

bool Shape::APointInside(double *x, double *y) {
  if (GeometryError(false)) {
    return false;       // AnyPointInPoly() needs good geometry
  }

  // Handle a nonempty single piece polygon of any orientation.
  JetPoint point;
  if (polys_.size() == 1 && polys_[0].p.size() >= 3) {
    AnyPointInPoly(polys_[0].p, -1, &point);
    *x = ToDouble(point[0]);
    *y = ToDouble(point[1]);
    return true;
  }

  // Handle a single positive area polygon with any number of negative area
  // holes.
  int posindex = -1;
  for (int i = 0; i < polys_.size(); i++) {
    if (Area(i) > 0) {
      if (posindex >= 0) {
        return false;           // Found more than one positive area piece
      }
      posindex = i;
    }
  }
  vector<RPoint> poly = polys_[posindex].p;
  for (int i = 0; i < polys_.size(); i++) {
    if (i != posindex) {
      poly.insert(poly.end(), polys_[i].p.begin(), polys_[i].p.end());
    }
  }
  AnyPointInPoly(poly, polys_[posindex].p.size(), &point);
  *x = ToDouble(point[0]);
  *y = ToDouble(point[1]);
  return true;
}

void Shape::FilletVertex(JetNum x, JetNum y, JetNum radius, JetNum limit) {
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

  // Compute center of fillet and other geometry.
  JetPoint v1 = p1 - pt;
  JetPoint v2 = p2 - pt;
  JetNum len1 = v1.norm();
  JetNum len2 = v2.norm();
  v1 /= len1;                   // Unit vector to prior vertex
  v2 /= len2;                   // Unit vector to next vertex
  JetPoint v = (v1 + v2) / 2;   // Vector to center
  if (v.norm() < 1e-9) {        //              @@@ THRESHOLD CORRECT?
    return;                     // Lines are (almost) colinear, nothing to do
  }
  v /= v.norm();                // Center is along pt + alpha*v
  JetNum alpha = abs(radius / (v1(1)*v(0) - v1(0)*v(1)));
  JetNum beta = alpha * (v1.dot(v));    // Start/end = beta*v1 or beta*v2
  if (beta >= len1 || beta >= len2) {
    // Fillet radius is too large, so limit it.
    JetNum minlen = std::min(len1, len2);
    JetNum ratio = minlen / beta;
    beta = minlen;
    alpha *= ratio;
    radius *= ratio;
  }
  JetPoint c = pt + alpha*v;            // Center point
  p1 = pt + beta*v1;                    // Start point
  p2 = pt + beta*v2;                    // End point

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
  JetNum steps = M_PI / acos(1.0 - limit / radius);     // In a circle
  steps = std::min(steps, kMaxStepsAllowed);
  int num_steps = ToInt64(ceil(abs(a2 - a1) * steps / (2.0 * M_PI))) + 1;
  if (num_steps <= 1) {
    return;
  }

  // Replace pt with the fillet arc points.
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

void Shape::ChamferVertex(JetNum x, JetNum y, JetNum predist, JetNum postdist) {
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
}

void Shape::SaveBoundaryAsDXF(const char *filename) {
  FILE *fout = fopen(filename, "wb");
  if (!fout) {
    Error("Can not write to '%s' (%s)", filename, strerror(errno));
    return;
  }
  fprintf(fout, "0\nSECTION\n2\nENTITIES\n");
  for (int i = 0; i < polys_.size(); i++) {
    for (int j1 = 0; j1 < polys_[i].p.size(); j1++) {
      int j2 = (j1 + 1) % polys_[i].p.size();
      double x1 = ToDouble(polys_[i].p[j1].p[0]);
      double y1 = ToDouble(polys_[i].p[j1].p[1]);
      double x2 = ToDouble(polys_[i].p[j2].p[0]);
      double y2 = ToDouble(polys_[i].p[j2].p[1]);
      fprintf(fout, "0\nLINE\n5\n0\n100\nAcDbEntity\n8\nCavity\n"
                    "100\nAcDbLine\n");
      fprintf(fout, "10\n%.10e\n20\n%.10e\n30\n0\n", x1, y1);
      fprintf(fout, "11\n%.10e\n21\n%.10e\n31\n0\n", x2, y2);
    }
  }
  fprintf(fout, "0\nENDSEC\n0\nEOF\n");
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
  polys_.clear();
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
  JetNum offset_x, offset_y, scale;
  if (!ClipperBounds(c1, c2, &offset_x, &offset_y, &scale)) {
    Clear();
    return;
  }
  RunClipper(c1, c2, clip_type, offset_x, offset_y, scale);
}

bool Shape::ClipperBounds(const Shape *c1, const Shape *c2, JetNum *offset_x,
                          JetNum *offset_y, JetNum *scale) const {
  // Compute bounds of c1, c2 or both.
  CHECK(c1 || c2);
  JetNum min_x = __DBL_MAX__, min_y = __DBL_MAX__;
  JetNum max_x = -__DBL_MAX__, max_y = -__DBL_MAX__;
  if ((c1 ? c1->UpdateBounds(&min_x, &min_y, &max_x, &max_y) : 0) +
      (c2 ? c2->UpdateBounds(&min_x, &min_y, &max_x, &max_y) : 0) == 0) {
    // No coordinates in either c1 or c2. The min/max wont be valid and the
    // result will have to be empty anyway.
    return false;
  }

  // Compute the scale factor from doubles to clipper integer coordinates.
  *offset_x = (max_x + min_x) / 2.0;
  *offset_y = (max_y + min_y) / 2.0;
  JetNum max_bound = std::max(max_x - min_x, max_y - min_y);
  *scale = kCoordScale / max_bound;
  return true;
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

  // Run the clipper. Use the pftPositive fill type so that a polygon can have
  // external invisible holes (the Paint() function can generate these).
  Clipper clipper;
  clipper.ZFillFunction(MyZFillCallback);
  if (c1) {
    clipper.AddPaths(p1, ptSubject, true);
  }
  if (c2) {
    clipper.AddPaths(p2, ptClip, true);
  }
  Paths result;
  clipper.Execute(clip_type, result, pftPositive, pftPositive);

  // Sometimes the clipper leaves vertices that are very close together or
  // duplicated. Clean these out because these shapes can not be meshed.
  Clean();

  // Convert the result back into JetPoint coordinates.
  FromPaths(scale, offset_x, offset_y, result);
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
    } else if (strcmp(s, "Paint") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<Shape, &Shape::LuaPaint>));
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
    lua_createtable(L, 0, 2);
    lua_pushnumber(L, polys_[0].p[i - 1].p[0]);
    LuaRawSetField(L, -2, "x");
    lua_pushnumber(L, polys_[0].p[i - 1].p[1]);
    LuaRawSetField(L, -2, "y");
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
  Expecting(L, 3, "AddPoint");
  AddPoint(luaL_checknumber(L, 2), luaL_checknumber(L, 3));
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
  Expecting(L, 2, "Clean");
  Clean(luaL_checknumber(L, 2));
  return 1;
}

int Shape::LuaContains(lua_State *L) {
  Expecting(L, 3, "Contains");
  lua_pushboolean(L,
      Contains(luaL_checknumber(L, 2), luaL_checknumber(L, 3)) != 0);
  return 1;
}

int Shape::LuaOffset(lua_State *L) {
  Expecting(L, 3, "Offset");
  Offset(luaL_checknumber(L, 2), luaL_checknumber(L, 3));
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaScale(lua_State *L) {
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
  Expecting(L, 2, "Rotate");
  Rotate(luaL_checknumber(L, 2));
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaMirrorX(lua_State *L) {
  Expecting(L, 2, "MirrorX");
  MirrorX(luaL_checknumber(L, 2));
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaMirrorY(lua_State *L) {
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
  // This is called with the arguments:
  //   1: {p=#, e=#} table (piece and edge), or an array of such tables
  //   2: port number
  // to mark the edge from vertex e to vertex e+1 as a port.
  Expecting(L, 3, "Port");
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
  vector<int> piece(1), edge(1);
  if (!GetPieceEdge(L, 2, &piece[0], &edge[0])) {
    if (!GetPieceEdgeArray(L, 2, &piece, &edge)) {
      LuaError(L, "Argument to Port() must be a {p=#, e=#} table, or an array "
                  "of such tables");
    }
  }
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
    if (!AssignPort(piece[i] - 1, edge[i] - 1, edge_kind)) {
      LuaError(L, "Piece %d edge %d already has a port number",
               piece[i], edge[i]);
    }
  }
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaABC(lua_State *L) {
  // Like LuaPort(), but use the special port number that designates an ABC.
  Expecting(L, 2, "ABC");
  lua_pushnumber(L, MAGIC_ABC_PORT);
  return LuaPort(L);
}

int Shape::LuaAPointInside(lua_State *L) {
  Expecting(L, 1, "APointInside");
  double x, y;
  if (!APointInside(&x, &y)) {
    LuaError(L, "APointInside() requires a nonempty single piece polygon of "
                "any orientation, or a single positive area polygon with any "
                "number of negative area holes.");
  }
  lua_pushnumber(L, x);
  lua_pushnumber(L, y);
  return 2;
}

int Shape::LuaFilletVertex(lua_State *L) {
  Expecting(L, 5, "FilletVertex");
  if (IsEmpty()) {
    LuaError(L, "FilletVertex() requires a nonempty shape");
  }
  FilletVertex(luaL_checknumber(L, 2), luaL_checknumber(L, 3),
               luaL_checknumber(L, 4), luaL_checknumber(L, 5));
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaChamferVertex(lua_State *L) {
  Expecting(L, 5, "ChamferVertex");
  if (IsEmpty()) {
    LuaError(L, "ChamferVertex() requires a nonempty shape");
  }
  ChamferVertex(luaL_checknumber(L, 2), luaL_checknumber(L, 3),
                luaL_checknumber(L, 4), luaL_checknumber(L, 5));
  lua_settop(L, 1);
  return 1;
}

int Shape::LuaPaint(lua_State *L) {
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
    mat.SetCallbackToRegistry(L);

    // Run the callback function once to verify it can work. This gives an
    // early indication for some (though not all) runtime errors.
    LuaVector *x = LuaUserClassCreateObj<LuaVector>(L);
    LuaVector *y = LuaUserClassCreateObj<LuaVector>(L);
    x->resize(10);
    y->resize(10);
    LuaVector *result[2];
    if (!Material::RunCallback(LuaGetObject(L), result)) {
      LuaError(L, "Callback function can not be run");
    }
  } else {
    int num_params = lua_gettop(L) - 3;
    if (num_params > Material::MaxParameters()) {
      LuaError(L, "Too many material parameters");
    }
    vector<JetNum> list;
    for (int i = 0; i < num_params; i++) {
      list.push_back(luaL_checknumber(L, i + 4));
    }
    mat.SetParameters(list);
  }
  Paint(*s2, mat);
  lua_settop(L, 1);
  return 1;
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ unconverted from matlab

/*
% Set this shape to the given "fat polyline". The coordinates of the center
% of the polyline are given by the x,y vectors and the widths of each
% segment are given by w. If w is a scalar then every segment has the same
% length. The cap1 and cap2 return values contain the [x1,y1,x2,y2]
% coordinates of the endcaps.
function [obj, cap1, cap2] = Polyline(obj, x, y, w)
  if length(w) == 1
    w = ones(length(x)-1, 1) * w;
  end
  assert(length(x) >= 2);
  assert(length(x) == length(y));
  assert(length(x) == length(w) + 1);

  % N = number of line segments (and num vertices) for the fat polyline.
  N = 2*length(x);
  lines = zeros(3, N);      % Lines in nx*x+ny*y=c form

  % Process each input line segment.
  for i = 1:length(x) - 1
    % Convert line segment into nx*x+ny*y=c form.
    len = sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2);
    nx =  (y(i+1) - y(i)) / len;
    ny = -(x(i+1) - x(i)) / len;
    c  = (x(i)*y(i+1) - x(i+1)*y(i)) / len;

    % Compute the lines for both sides of the fat polyline in
    % nx*x+ny*y=c form.
    lines(:, i    ) = [nx; ny; c + w(i) / 2];
    lines(:, N - i) = [nx; ny; c - w(i) / 2];
  end

  % Compute the lines for the end caps in nx*x+ny*y=c form.
  len = sqrt((x(2)-x(1))^2 + (y(2)-y(1))^2);
  lines(1, end) = (x(2)-x(1)) / len;
  lines(2, end) = (y(2)-y(1)) / len;
  lines(3, end) = (y(1)*y(2) - y(1)^2 + x(1)*x(2) - x(1)^2)/len;
  len = sqrt((x(end-1)-x(end))^2 + (y(end-1)-y(end))^2);
  lines(1, N/2) = (x(end-1)-x(end)) / len;
  lines(2, N/2) = (y(end-1)-y(end)) / len;
  lines(3, N/2) = (y(end)*y(end-1) - y(end)^2 + x(end)*x(end-1) - x(end)^2)/len;

  % Compute all shape vertices by intersecting the lines.
  obj.poly = zeros(2, N);
  for i = 1:N
    j = mod(i, N) + 1;
    denom = lines(1,j) * lines(2,i) - lines(1,i) * lines(2,j);
    obj.poly(1,i) =  (lines(3,j) * lines(2,i) - lines(3,i) * lines(2,j)) / denom;
    obj.poly(2,i) = -(lines(3,j) * lines(1,i) - lines(3,i) * lines(1,j)) / denom;
  end

  % Copy out endcap coordinates.
  cap1 = [obj.poly(1,N-1), obj.poly(2,N-1), obj.poly(1,N), obj.poly(2,N)];
  cap2 = [obj.poly(1,N/2-1), obj.poly(2,N/2-1), obj.poly(1,N/2), obj.poly(2,N/2)];
end

*/

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
      CHECK(s.APointInside(&px, &py));
      CHECK(s.Contains(px, py) == 1);
    }

    {
      // Create a shape with one positive area piece and no holes.
      Shape s;
      RandomShape2(&s, 0);

      // Test APointInside.
      double px, py;
      CHECK(s.APointInside(&px, &py));
      CHECK(s.Contains(px, py) == 1);

      // Test reverse orientation polys too.
      Shape s2;
      s2 = s;
      s2.Reverse();
      CHECK(s2.APointInside(&px, &py));
      CHECK(s.Contains(px, py) == 1);
    }

    {
      // Test triangles, make sure this simple case works too.
      Shape s;
      RandomShape1(&s, 3, true);
      double px, py;
      CHECK(s.APointInside(&px, &py));
      CHECK(s.Contains(px, py) == 1);
      Shape s2;
      s2 = s;
      s2.Reverse();
      CHECK(s2.APointInside(&px, &py));
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
