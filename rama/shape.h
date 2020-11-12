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

#ifndef __SHAPE_H__
#define __SHAPE_H__

#include "../toolkit/myvector"
#include "../toolkit/lua_util.h"
#include "common.h"
#include "clipper.h"
#include "edge_type.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "my_jet.h"
#include "../toolkit/lua_vector.h"

using std::vector;

// All 2D coordinates should be JetPoint, so that the derivative of geometry
// with respect to parameters can be tracked.
typedef Eigen::Matrix<JetNum, 2, 1> JetPoint;

// 2D transformations can use this matrix type.
typedef Eigen::Matrix<JetNum, 2, 2> JetMatrix2d;

// A 2D point in a polygon or in a mesh. RPoint means "rama point" and is
// not named Point to avoid conflicts with the same structure in MacTypes.h.
struct RPoint {
  JetPoint p;
  EdgeInfo e;

  // For mesh points on the boundary this links to the piece and edge of the
  // original shape. For all other points these are -1.
  int original_piece, original_edge;

  RPoint() : original_piece(-1), original_edge(-1) { p.setZero(); }
  RPoint(JetNum x, JetNum y) : p(x, y), original_piece(-1), original_edge(-1) {}

  bool operator==(const RPoint &q) const {
    return p == q.p && e == q.e;
  }

  // Less-than operator used to order points in map<> and similar containers.
  bool operator<(const RPoint &q) const {
    return p[0] < q.p[0] || (p[0] == q.p[0] && p[1] < q.p[1]);
  }
};

// Needed for MaterialParameters comparison below.
inline bool operator < (const JetComplex &a, const JetComplex &b) {
  return a.real() < b.real() || (a.real() == b.real() && a.imag() < b.imag());
}

// Parameters that will have a value at each point in a material.
struct MaterialParameters {
  enum { MAX_PARAMS = 5 };
  JetComplex epsilon;           // Multiplies k^2
  JetComplex sigma_xx;          // Sigma values for anisotropic Laplacians
  JetComplex sigma_yy;          // The isotropic Laplacian has xx=1, yy=1, xy=0
  JetComplex sigma_xy;
  JetComplex excitation;        // Or f(), the "right hand side"
  bool is_isotropic;            // True if sigma values are xx=1, yy=1, xy=0

  MaterialParameters() {
    SetDefault();
  }
  void SetSigmas(const JetComplex &xx, const JetComplex &yy,
                 const JetComplex &xy) {
    sigma_xx = xx;
    sigma_yy = yy;
    sigma_xy = xy;
    is_isotropic = (sigma_xx == JetComplex(1.0)) &&
      (sigma_yy == JetComplex(1.0)) && (sigma_xy == JetComplex(0.0));
  }

  bool operator==(const MaterialParameters &a) const {
    return epsilon == a.epsilon && sigma_xx == a.sigma_xx &&
           sigma_yy == a.sigma_yy && sigma_xy == a.sigma_xy &&
           excitation == a.excitation;
  }

  // Less-than operator used to order materials in map<> and similar containers.
  bool operator<(const MaterialParameters &a) const {
    if (epsilon < a.epsilon) return true;
    if (a.epsilon < epsilon) return false;
    if (sigma_xx < a.sigma_xx) return true;
    if (a.sigma_xx < sigma_xx) return false;
    if (sigma_yy < a.sigma_yy) return true;
    if (a.sigma_yy < sigma_yy) return false;
    if (sigma_xy < a.sigma_xy) return true;
    if (a.sigma_xy < sigma_xy) return false;
    return excitation < a.excitation;
  }

  // Mechanism to set parameters from a Lua function's argument list.
  void SetParameters(const vector<JetComplex> &list) {
    CHECK(list.size() == 1 || list.size() == 4 || list.size() == MAX_PARAMS);
    epsilon = list[0];
    if (list.size() >= 4) {
      sigma_xx = list[1];
      sigma_yy = list[2];
      sigma_xy = list[3];
    }
    if (list.size() >= 5) {
      excitation = list[4];
    }
    is_isotropic = (sigma_xx == JetComplex(1.0)) &&
      (sigma_yy == JetComplex(1.0)) && (sigma_xy == JetComplex(0.0));
  }

  // Set all values to their defaults.
  void SetDefault() {
    epsilon = 1;        // i.e. vacuum
    sigma_xx = 1;       // i.e. isotropic Laplacian
    sigma_yy = 1;
    sigma_xy = 0;
    excitation = 0;
    is_isotropic = true;
  }
};

// Polygon and triangle material properties.
struct Material : public MaterialParameters {
  // Flags that can be ORed with color to indicate some special properties of
  // the material.
  enum {
    FAR_FIELD = 0x1000000,  // Compute far field at border of this region
  };

  uint32 color;          // 0xrrggbb color (for drawing only, not simulation)
  LuaCallback callback;  // Callback function.
  // If 'callback' is Valid() then it is the callback function that makes
  // material parameters from (x,y) coordinates.

  Material() {
    color = 0xe0e0ff;
  }

  // Comparison. NOTE that callback comparison is probabilistic, see the
  // LuaCallback class.
  bool operator==(const Material &a) const {
    return MaterialParameters::operator==(a) &&
           color == a.color && callback == a.callback;
  }

  // Less-than operator used to order materials in map<> and similar containers.
  bool operator<(const Material &a) const {
    if (color < a.color) return true;
    if (color > a.color) return false;
    if (callback < a.callback) return true;
    if (a.callback < callback) return false;
    return MaterialParameters::operator<(a);
  }

  // Helper for running the callback function one time. After the callback and
  // x,y vectors are pushed on to the stack, call this to run the callback.
  // Returns true on success or false if there is a problem (in which case an
  // error message will have been generated). The callback function arguments
  // are removed from the stack. If 'within_lua' is true this is called from
  // within a Lua C function, so LuaError() will be used to generate errors.
  // Otherwise only Error() will be used.
  static bool RunCallback(Lua *lua, bool within_lua,
                          vector<MaterialParameters> *result) MUST_USE_RESULT;
};

struct Triangle {
  int index[3];         // Three point indexes define this triangle
  int material;         // Material index in containing Mesh object
  int neighbor[3];      // [i]=index of neighbor triangle for edge
};                      //   index[i]->index[(i+1)%3], or -1 if boundary edge

class Shape : public LuaUserClass {
 public:
  ~Shape();
  // Default copy constructor and assignment operator are ok.

  // Test for exact [in]equality of two shapes, i.e. not just the same outline
  // but the same order of vertices, the same edge kinds, and the same
  // materials etc. NOTE that callback comparison is probabilistic, see the
  // LuaCallback class.
  bool operator==(const Shape &s) const {
    return polys_ == s.polys_ && port_callbacks_ == s.port_callbacks_;
  }
  bool operator!=(const Shape &s) const { return !operator==(s); }

  // Return 0 if the shape geometry is well formed, otherwise return an error
  // message string. If enforce_positive_area is true then only positive area
  // shapes with negative area holes are allowed.
  const char *GeometryError(bool enforce_positive_area = true) const;

  // Hook up global functions like 'Rectangle' (etc) to lua.
  static void SetLuaGlobals(lua_State *L);

  // Swap two shapes (a fast way to exchange data).
  void Swap(Shape *s) {
    polys_.swap(s->polys_);
    port_callbacks_.swap(s->port_callbacks_);
  }

  // Set the empty shape.
  void Clear();

  // Dump shape data, for debugging.
  void Dump() const;

  // Draw shape to OpenGL.
  void DrawInterior(double alpha = 0) const;
  void DrawBoundary(const Eigen::Matrix4d &camera_transform,
                    bool show_lines, bool show_ports, bool show_vertices,
                    double boundary_derivatives_scale = 0) const;

  // Return true if this shape is completely empty.
  bool IsEmpty() const { return polys_.empty() || polys_[0].p.empty(); }

  // Return the number of separate pieces (polygons) in this shape. Disjoint
  // pieces count, interior holes count.
  int NumPieces() const { return polys_.size(); }

  // Return the polygon for the n'th piece.
  const vector<RPoint>& Piece(int n) const { return polys_[n].p; }

  // Return the material for the n'th piece.
  const Material& GetMaterial(int n) const { return polys_[n].material; }

  // Set this polygon to the n'th piece of 'p' (p can be this).
  void SetToPiece(int n, const Shape &p);

  // Set the given piece,edge to the given edge kind. Return true on success or
  // false if there is a conflict on this edge. If pmap is nonzero, use it to
  // ensure that coincident (duplicate) points all have consistent EdgeInfo.
  typedef std::map<std::pair<JetNum, JetNum>, vector<std::pair<int,int>>> PointMap;
  bool AssignPort(int piece, int edge, EdgeKind kind, PointMap *pmap = 0)
    MUST_USE_RESULT;

  // Return the orientation of the n'th piece. True is outer (anticlockwise)
  // orientation, false is inner (clockwise) orientation.
  bool Orientation(int n) const { return Area(n) >= 0; }

  // Return the area of the n'th piece. Positive areas are returned for outer
  // pieces, negative areas are returned for inner pieces.
  JetNum Area(int n) const;

  // Return the total area of all pieces.
  JetNum TotalArea() const;

  // Return true if any piece in this shape seems to intersect itself.
  bool SelfIntersection() const;

  // Return the sharpest convex angle in the shape, i.e. the angle that would
  // result in the mesher generating the most triangles. 0 is maximum
  // sharpness, pi is least sharp.
  JetNum SharpestAngle() const;

  // Return the largest and smallest side lengths.
  void ExtremeSideLengths(double *length_max, double *length_min) const;

  // Add a point to the last piece in the shape, optionally with EdgeInfo.
  void AddPoint(JetNum x, JetNum y, const EdgeInfo *e = 0);

  // Turn the last piece of this shape into a polyline by adding points
  // s[#s-1],...,s[2] to the polygon. This makes a zero area polygon to
  // represent the polyline.
  void MakePolyline();

  // Set this shape to the given rectangle.
  void SetRectangle(JetNum x1, JetNum y1, JetNum x2, JetNum y2);

  // Set this shape to the given circle.
  void SetCircle(JetNum x, JetNum y, JetNum radius, int npoints);

  // Set to the intersection, union (etc) of shapes c1 and c2. Either c1 or c2
  // can be this shape.
  void SetIntersect(const Shape &c1, const Shape &c2);
  void SetUnion(const Shape &c1, const Shape &c2);
  void SetDifference(const Shape &c1, const Shape &c2);
  void SetXOR(const Shape &c1, const Shape &c2);

  // Paint material properties into this shape at 's'. This potentially splits
  // the polygons into unmerged pieces with different material properties.
  void Paint(const Shape &s, const Material &mat);

  // Set this shape to 's', but merge together any adjacent pieces, erasing the
  // distinction between different materials. This undoes the effects of
  // Paint().
  void SetMerge(const Shape &s);

  // Get the boundary rectangle of this shape. It is a runtime error if the
  // shape is empty so the caller must check for that.
  void GetBounds(JetNum *min_x, JetNum *min_y,
                 JetNum *max_x, JetNum *max_y) const;

  // Return 0 if x,y is outside the shape, +1 if it is inside the shape, or -1
  // if x,y is exactly on boundary. Note that a point inside a polygon hole
  // will be considered to be outside.
  int Contains(JetNum x, JetNum y);

  // Transform the shape. All transformations except Reverse() preserve the
  // orientation.
  void Offset(JetNum dx, JetNum dy);
  void Scale(JetNum scalex, JetNum scaley);
  void Rotate(JetNum theta);            // theta in degrees
  void MirrorX(JetNum x_coord);         // Mirror about x == x_coord
  void MirrorY(JetNum y_coord);         // Mirror about y == y_coord
  void Reverse();                       // Reverse orientation

  // Grow or shrink the shape by 'delta'.
  //   - Only works with area >= 0 polygons.
  //   - For the miter style, 'limit' is the miter limit (e.g. 2).
  //   - For the round style, 'limit' is the maximum distance allowed between
  //     the polygon approximation of an arc and a true circle.
  enum CornerStyle { SQUARE, ROUND, MITER, BUTT };
  void Grow(JetNum delta, CornerStyle style, JetNum limit,
            CornerStyle endcap_style = BUTT);

  // Clean up the shape: remove vertices that are closer than 'threshold' to
  // other vertices. If threshold is zero then use a default that is a small
  // fraction of the largest side length.
  void Clean(JetNum threshold = 0);

  // Find shape polygons with zero-width necks and split those into multiple
  // pieces at the necks. This is needed because the clipper library is happy
  // to regard polygons with zero width necks as a single polygon, but the
  // triangle library regards the parts separated by the neck as distinct and
  // gets confused because AnyPointInPoly() returns a point only inside one of
  // them. Zero-area slivers (where two edges are coincident with the same
  // endpoints) are discarded and not created as separate pieces. Note that
  // this does nothing about necks that are formed by multiple polygons with
  // different materials that share a single point.
  void SplitPolygonsAtNecks();

  // Return the piece and edge that is closest to x,y. The edge index is
  // actually a vertex number N such that the edge is from vertex N to vertex
  // N+1. It is a runtime error if the shape is empty.
  void FindClosestEdge(JetNum x, JetNum y, int *piece, int *edge);

  // Return the piece and vertex index that is closest to x,y. It is a runtime
  // error if the shape is empty.
  void FindClosestVertex(JetNum x, JetNum y, int *piece, int *index);

  // Return a point that is guaranteed to be inside piece 'i' of the shape, but
  // not inside any of the other pieces that are inside that piece. For example
  // if the shape contains holes the returned point will not be inside any of
  // those holes. Or, if piece 'i' is itself a hole, the returned point will
  // not be inside any islands that are inside the hole. Return false if there
  // are any geometry errors. If i == -1 and there is just one piece then use
  // that, or use the single positive area piece, or return false if there is
  // not just one.
  bool APointInside(int i, double *x, double *y);

  // Add a fillet of the given radius to the vertex closest to x,y. The limit
  // is the maximum distance allowed between a polygon approximation of an arc
  // and a true circle. It is a runtime error if the shape is empty. Optionally
  // return the coordinates of the new arc start and end vertices, and the arc
  // center. If mutate is false, the pstart (etc) return values will be
  // computed but the shape will not be modified. Return true if an arc was
  // created (or does not need to be created because the vertex is colinear) or
  // false if no arc of this radius can fit.
  bool FilletVertex(JetNum x, JetNum y, JetNum radius, JetNum limit,
                    JetPoint *pstart=0, JetPoint *pend=0, JetPoint *center=0,
                    bool mutate=true) MUST_USE_RESULT;

  // Add a chamfer of the given pre- and post-vertex distances to the vertex
  // closest to x,y. It is a runtime error if the shape is empty. Optionally
  // return the coordinates of the new vertices that are created.
  void ChamferVertex(JetNum x, JetNum y, JetNum predist, JetNum postdist,
                     JetPoint *p1=0, JetPoint *p2=0);

  // Save the shape to various file formats.
  void SaveBoundaryAsDXF(const char *filename,
                         double arc_dist, double arc_angle);
  void SaveBoundaryAsXY(const char *filename);

  // Load a binary STL file and convert it to a Shape that traces the edges of
  // all the triangles in the Z=0 plane. If there is an error then this shape
  // will be empty, false will be returned, and an error will be generated.
  bool LoadSTL(const char *filename);

  // Access port callbacks.
  const std::map<int, LuaCallback> & PortCallbacks() const {
    return port_callbacks_;
  }

  //.........................................................................
  // Lua interface. All lua functions that don't return some other value will
  // return the userdata object, so that calls can be chained together like
  // Shape():foo():bar().

  int Index(lua_State *L);
  int FunctionCall(lua_State *L);
  int Length(lua_State *L);
  bool Operator(lua_State *L, int op, int pos);

  int LuaClone(lua_State *L);
  int LuaAddPoint(lua_State *L);
  int LuaMakePolyline(lua_State *L);
  int LuaContains(lua_State *L);
  int LuaOffset(lua_State *L);
  int LuaScale(lua_State *L);
  int LuaRotate(lua_State *L);
  int LuaMirrorX(lua_State *L);
  int LuaMirrorY(lua_State *L);
  int LuaReverse(lua_State *L);
  int LuaGrow(lua_State *L);
  int LuaSelect(lua_State *L);
  int LuaSelectAll(lua_State *L);
  int LuaPort(lua_State *L);
  int LuaABC(lua_State *L);
  int LuaAPointInside(lua_State *L);
  int LuaFilletVertex(lua_State *L);
  int LuaChamferVertex(lua_State *L);
  int LuaPaint(lua_State *L);
  int LuaHasPorts(lua_State *L);
  int LuaLoadSTL(lua_State *L);
  int LuaClean(lua_State *L);

 private:
  // The shape is a vector of pieces. Each piece is a vector of points that is
  // a closed polygon, along with some auxiliary information. Each polygon's
  // winding direction determines whether it is an outer boundary (counter
  // clockwise, positive area) or an inner hole (clockwise, negative area).
  // Each outer boundary is disjoint, though they may share common edges or
  // points, and they may have different material properties.
  struct Polygon {
    vector<RPoint> p;           // Points on polygon boundary
    Material material;          // Material of this polygon interior

    bool operator==(const Polygon &a) const {
      return p == a.p && material == a.material;
    }
    bool operator!=(const Polygon &a) const { return !operator==(a); }

    void Swap(Polygon &a) {
      p.swap(a.p);
      Material tmp = material;
      material = a.material;
      a.material = tmp;
    }
  };
  vector<Polygon> polys_;
  std::map<int, LuaCallback> port_callbacks_;  // port num -> callback func

  int UpdateBounds(JetNum *min_x, JetNum *min_y, JetNum *max_x, JetNum *max_y)
      const;
  void ToPaths(JetNum scale, JetNum offset_x, JetNum offset_y,
               ClipperLib::Paths *paths) const;
  void FromPaths(JetNum scale, JetNum offset_x, JetNum offset_y,
                 const ClipperLib::Paths &paths);
  void RunClipper(const Shape *c1, const Shape *c2,
                  ClipperLib::ClipType clip_type);
  bool ClipperBounds(const Shape *c1, const Shape *c2, JetNum *offset_x,
                     JetNum *offset_y, JetNum *scale) const;
  void RunClipper(const Shape *c1, const Shape *c2,
                  ClipperLib::ClipType clip_type,
                  JetNum offset_x, JetNum offset_y, JetNum scale);
  const Shape &LuaCheckShape(lua_State *L, int argument_index) const;
  bool CombinePortCallbacks(const Shape *c1, const Shape *c2);
};

// ********** Public geometry utility functions.

// Returns 0 if 'p' is not in the triangle defined by a,b,c, +1 if it is, or -1
// if pt is on the triangle boundary.
int PointInTriangle(const JetPoint &p, const JetPoint &a,
                    const JetPoint &b, const JetPoint &c);

// Return true if the triangle (p[0],p[1],p[2]) intersects the box.
bool TriangleIntersectsBox(const JetPoint *p[3], double xmin, double xmax,
                           double ymin, double ymax);

// Given a well formed polygon, return a point that is guaranteed to be inside
// it (regardless of its orientation). If poly_size is -1 then it is ignored.
// If the polygon has positive area and contains holes in which the returned
// point should not lie then poly_size is the number of initial points in
// 'poly' that contain the actual polygon, and the remaining points in 'poly'
// are from all of the holes. This scheme allows the algorithm to not have to
// treat hole polygon points any differently. It is a fatal error if the
// polygon is not well formed.
void AnyPointInPoly(const vector<RPoint> &poly, int poly_size,
                    JetPoint *point);

// Convert JetPoints to Vector2d.
inline Eigen::Vector2d ToVector2d(const JetPoint &p) {
  return Eigen::Vector2d(ToDouble(p[0]), ToDouble(p[1]));
}

#endif
