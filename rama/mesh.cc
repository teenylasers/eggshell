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
#include <setjmp.h>
#include <map>
#include "common.h"
#include "mesh.h"
#include "../toolkit/gl_utils.h"
#include "../toolkit/gl_font.h"
#include "../toolkit/colormaps.h"
#include "../toolkit/testing.h"
#include "../toolkit/shaders.h"
extern "C" {
  #include "triangle.h"
}

using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::Matrix4d;

static const bool kDebugMesh = false;           // Render debug stuff on mesh
static const double kSharpestAllowableAngle = 1e-4;

//***************************************************************************
// Triangle library support. We use nasty globals here because the triangle
// library does not support passing user data to the triunsuitable() callback.

static double square_of_longest_edge_permitted;

// Function called by the triangle library to see if a triangle is too big and
// needs refinement.
extern "C" int triunsuitable(double *v1, double *v2, double *v3, double area) {
  // Compute edge vectors.
  double dx1 = v1[0] - v3[0];
  double dy1 = v1[1] - v3[1];
  double dx2 = v2[0] - v3[0];
  double dy2 = v2[1] - v3[1];
  double dx3 = v1[0] - v2[0];
  double dy3 = v1[1] - v2[1];

  // Find the squares of the lengths of the triangle's three edges.
  double len1 = dx1 * dx1 + dy1 * dy1;
  double len2 = dx2 * dx2 + dy2 * dy2;
  double len3 = dx3 * dx3 + dy3 * dy3;

  // Find the square of the length of the longest edge.
  double maxlen = std::max(len1, std::max(len2, len3));

  return maxlen > square_of_longest_edge_permitted;
}

// Any printf() that the triangle library does probably represents some kind of
// complaint and ends up here.

extern "C" int triprintf(const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  GetErrorHandler()->HandleError(ErrorHandler::Message, msg, ap);
  return 0;
}

// This is called by the triangle library to indicate an error.

static jmp_buf triangle_jmp_buf;
extern "C" int triexit(int status) {
  Error("Triangulation failed");
  longjmp(triangle_jmp_buf, 1);
}

// Free heap-allocated data in a triangulateio structure from the triangle
// library. This comes in two forms, based on free() and delete[]. These two
// forms are mostly identical except that valgrind will complain if you mix
// them up.

static void FreeTriangulateIO(triangulateio *t) {
  free(t->pointlist);
  free(t->pointattributelist);
  free(t->pointmarkerlist);
  free(t->trianglelist);
  free(t->triangleattributelist);
  free(t->trianglearealist);
  free(t->neighborlist);
  free(t->segmentlist);
  free(t->segmentmarkerlist);
  free(t->holelist);
  free(t->regionlist);
  free(t->edgelist);
  free(t->edgemarkerlist);
  free(t->normlist);
}

static void DeleteTriangulateIO(triangulateio *t) {
  delete[] t->pointlist;
  delete[] t->pointattributelist;
  delete[] t->pointmarkerlist;
  delete[] t->trianglelist;
  delete[] t->triangleattributelist;
  delete[] t->trianglearealist;
  delete[] t->neighborlist;
  delete[] t->segmentlist;
  delete[] t->segmentmarkerlist;
  delete[] t->holelist;
  delete[] t->regionlist;
  delete[] t->edgelist;
  delete[] t->edgemarkerlist;
  delete[] t->normlist;
}

static void DumpTriangulateIO(const struct triangulateio &t, const char *msg)
  __attribute__((unused));
static void DumpTriangulateIO(const struct triangulateio &t, const char *msg) {
  printf("%s struct triangulateio contains:\n", msg);
  printf("\tPoints (numberofpoints=%d, numberofpointattributes=%d):\n",
         t.numberofpoints, t.numberofpointattributes);
  for (int i = 0; i < t.numberofpoints; i++) {
    printf("\t\t%d: xy=%g,%g | marker=%d\n", i,
           t.pointlist[2*i], t.pointlist[2*i+1], t.pointmarkerlist[i]);
    // Ignored, we don't use: t.pointattributelist
  }
  printf("\tTriangles (numberoftriangles=%d, numberofcorners=%d, "
         "numberoftriangleattributes=%d):\n",
         t.numberoftriangles, t.numberofcorners, t.numberoftriangleattributes);
  for (int i = 0; i < t.numberoftriangles; i++) {
    CHECK(t.numberofcorners == 3);
    printf("\t\t%d: %d %d %d | neighbors=%d %d %d | attr=", i,
           t.trianglelist[i*3], t.trianglelist[i*3+1], t.trianglelist[i*3+2],
           t.neighborlist[i*3], t.neighborlist[i*3+1], t.neighborlist[i*3+2]);
    for (int j = 0; j < t.numberoftriangleattributes; j++) {
      printf(" %g", t.triangleattributelist[i*t.numberoftriangleattributes+j]);
    }
    printf("\n");
    // Ignored, we don't use: trianglearealist
  }
  printf("\tSegments (numberofsegments=%d):\n", t.numberofsegments);
  for (int i = 0; i < t.numberofsegments; i++) {
    printf("\t\t%d: %d %d | marker=%d\n", i,
           t.segmentlist[i*2], t.segmentlist[i*2+1], t.segmentmarkerlist[i]);
  }
  printf("\tHoles (numberofholes=%d):\n", t.numberofholes);
  for (int i = 0; i < t.numberofholes; i++) {
    printf("\t\t%d: xy=%g,%g\n", i, t.holelist[i*2], t.holelist[i*2+1]);
  }
  printf("\tRegions (numberofregions=%d):\n", t.numberofregions);
  for (int i = 0; i < t.numberofregions; i++) {
    printf("\t\t%d: xy=%g,%g | attr=%g | area=%g\n", i, t.regionlist[i*4],
           t.regionlist[i*4+1], t.regionlist[i*4+2], t.regionlist[i*4+3]);
  }
  // Ignored, we don't use: t.numberofedges, edgelist, edgemarkerlist, normlist
};

//***************************************************************************
// Mesh.

Mesh::Mesh(const Shape &s_arg, double longest_edge_permitted, Lua *lua) {
  Trace trace(__func__);

  // Find shape polygons with zero-width necks and split those into multiple
  // pieces at the necks. This is needed because the clipper library is happy
  // to regard polygons with zero width necks as a single polygon, but the
  // triangle library regards the parts separated by the neck as distinct and
  // gets confused because AnyPointInPoly() returns a point only inside one of
  // them. We don't modify the shape argument, we make a local copy.
  Shape s(s_arg);
  s.SplitPolygonsAtNecks();

  // Check for mesh validity.
  valid_mesh_ = false;          // Default assumption
  {
    const char *geometry_error = s.GeometryError();
    if (geometry_error) {
      ERROR_ONCE("Can not create mesh: %s", geometry_error);
      return;
    }
    if (s.TotalArea() <= 0 || s.IsEmpty()) {
      ERROR_ONCE("Can not create mesh from empty or zero area polygon");
      return;
    }
  }
  frexp(longest_edge_permitted, &cell_size_);
  cell_size_ -= 2;

  // If we are asked to make a mesh from shapes with extremely short line
  // segments or extremely small interior angles then the mesher will consume a
  // huge amount of time and memory, mainly because of its desire to generate
  // triangles with low aspect ratios. The "short edge" case should be cleaned
  // up by the caller. Detect and warn about the other cases.
  // @@@ Should warn about this once per optimization run, not once per file
  //     reload.
  JetNum sharpest = s.SharpestAngle();
  if (sharpest < kSharpestAllowableAngle) {
    ERROR_ONCE("Can not create mesh because sharpest angle is %g (min is %g)",
               ToDouble(sharpest), kSharpestAllowableAngle);
    return;
  }

  // Save the shape width and height.
  {
    JetNum min_x, min_y, max_x, max_y;
    s.GetBounds(&min_x, &min_y, &max_x, &max_y);
    cd_width_ = ToDouble(max_x - min_x);
    cd_height_ = ToDouble(max_y - min_y);
  }

  // Identify negative area pieces that will become holes. For each hole pick
  // an x,y point that is guaranteed to be in the hole so that we can identify
  // it to the triangle library. This cumbersome way to identify holes (and
  // regions) is one of the main annoyances of the triangle library. If we have
  // split the model into unmergeable pieces with Paint() then polygon holes
  // may be enclosed by separate pieces but not actually be represented as
  // negative area polygons. We can also have holes that are precisely filled
  // by positive area polygons of different material. To properly identify the
  // actual holes to the triangle library we need to run clipper to merge
  // everything together and find any negative area polygons that result.
  vector<RPoint> hole_points;   // One point inside each hole
  {
    Shape hole_finder;
    hole_finder.SetMerge(s);
    for (int i = 0; i < hole_finder.NumPieces(); i++) {
      if (hole_finder.Area(i) < 0) {
        double x, y;
        CHECK(hole_finder.APointInside(i, &x, &y));
        hole_points.push_back(RPoint(x, y));
      }
    }
  }

  // Identify holes in this shape.
  int num_holes = 0;
  vector<bool> is_a_hole(s.NumPieces());
  for (int i = 0; i < s.NumPieces(); i++) {
    if (s.Area(i) < 0) {
      is_a_hole[i] = true;
      num_holes++;
    }
  }

  // Even though there can be duplicate points, we give all point coordinates
  // 'UPI's (unique point indexes). Duplicate points occur when unmerged
  // polygons with different material types come together. We insist that
  // duplicate points nevertheless have the same EdgeInfo, so that later when
  // we copy EdgeInfo from the shape into the mesh there is no ambiguity that
  // leads to errors in port formation.
  typedef std::map<std::pair<JetNum, JetNum>, int> PointMap;
  PointMap point_map;                           // x,y -> UPI
  vector<EdgeInfo> upi_edge_info;               // UPI --> edge info
  int num_unique_points = 0;
  for (int i = 0; i < s.NumPieces(); i++) {
    for (int j = 0; j < s.Piece(i).size(); j++) {
      JetPoint p = s.Piece(i)[j].p;
      auto it = point_map.find(std::make_pair(p[0], p[1]));
      if (it == point_map.end()) {
        // This is a new point.
        point_map[std::make_pair(p[0], p[1])] = num_unique_points;
        upi_edge_info.push_back(s.Piece(i)[j].e);
        num_unique_points++;
      } else {
        if (upi_edge_info[it->second] != s.Piece(i)[j].e) {
          // Even though we found a bug, we don't CHECK() and crash, because
          // ensuring this consistency is a little hard to get right and we
          // don't want to render the program unusable in some weird corner
          // case.
          ERROR_ONCE("Internal error: "
                     "Found duplicate points with inconsistent EdgeInfo");
        }
      }
    }
  }

  // Make a reverse point index that maps unique point indexes to Piece(i)[j].
  // In pass 0, for each Piece(i)[j] segment (i.e. from point j to j+1), figure
  // out how many times that segment is used overall. Segments that are used
  // just once are on the external boundary. For pass 1 and 2, make an
  // index_map mapping from UPIs (at the start of a segment) to boundary
  // Piece(i)[j] vertices (at the start of the same segment). Duplicate points
  // that map to the same UPI happen if there are unmerged polygons with
  // different material types, in which case a UPI could be mapped to multiple
  // Piece(i)[j] segments, but we always select the one on the external
  // boundary because the newly created mesh points will adopt the segment
  // markers which are taken from this mapping, and we want those segment
  // markers to indicate the correct boundary conditions.
  std::map<std::pair<int, int>, int> seg_count;         // (UPI,UPI) --> count
  vector<std::pair<int, int>> index_map(num_unique_points);  // UPI -> i,j
  for (int i = 0; i < num_unique_points; i++) {
    index_map[i] = std::make_pair(-1, -1);      // -1 means "unknown"
  }
  for (int pass = 0; pass < 3; pass++) {
    for (int i = 0; i < s.NumPieces(); i++) {
      for (int j = 0; j < s.Piece(i).size(); j++) {
        JetPoint p1 = s.Piece(i)[j].p;
        JetPoint p2 = s.Piece(i)[(j+1) % s.Piece(i).size()].p;
        int upi1 = point_map[std::make_pair(p1[0], p1[1])];
        int upi2 = point_map[std::make_pair(p2[0], p2[1])];
        if (pass == 0) {
          // Count segment use in pass 0.
          seg_count[std::make_pair(upi1, upi2)]++;
          seg_count[std::make_pair(upi2, upi1)]++;
        } else if (pass == 1) {
          int count = seg_count[std::make_pair(upi1, upi2)];
          if (count == 1) {
            // Give preference to boundary segments in pass 1.
            if (index_map[upi1].first != -1) {
              // In this (rare, usually degenerate) case a point is on many
              // external boundary segments, e.g. if different material's
              // polygons come together at a neck. This can not be represented
              // by index_map so we fail. @@@ Fix this?
              ERROR_ONCE("Internal error: "
                         "Can not create mesh, polygons are necked");
              return;
            }
            index_map[upi1] = std::make_pair(i, j);
          }
        } else {
          // For unassigned interior points choose arbitrary segments in pass 2.
          if (index_map[upi1].first == -1) {
            index_map[upi1] = std::make_pair(i, j);
          }
        }
      }
    }
  }
  for (int i = 0; i < num_unique_points; i++) {
    CHECK(index_map[i].first >= 0);     // Make sure all points assigned
  }

  // Setup data structure for 'Triangle' library. Since we are dealing with one
  // or more closed polygons, the number of vertices is equal to the number of
  // segments. Segment-bounded region attributes are set to the polygon indices
  // in polys_, so that triangle material types can be determined from the
  // output triangle attributes.
  //
  // Setting the marker values in the input is important for identifying
  // boundary edges in the output. Points are marked with their UPI (plus 2
  // since 0 and 1 have a reserved meaning in the triangle library). Segments
  // are marked with -1-(the UPI of the first point in the edge). New vertices
  // in the triangulation will pick up the segment marker values.
  int count = 0;                                // Count of all polys_ points
  for (int i = 0; i < s.NumPieces(); i++) {
    count += s.Piece(i).size();
  }
  triangulateio tin, tout;
  memset(&tin, 0, sizeof(tin));
  memset(&tout, 0, sizeof(tout));
  tin.pointlist = new double[num_unique_points * 2];  // x,y in UPI order
  tin.pointmarkerlist = new int[num_unique_points];   // marker for all points
  tin.numberofpoints = num_unique_points;
  tin.segmentlist = new int[count * 2];         // RPoint indices for segments
  tin.segmentmarkerlist = new int[count];       // Marker for all segments
  tin.numberofsegments = 0;                     // Set below, will be <= count
  tin.holelist = new double[hole_points.size() * 2];  // x,y inside all holes
  tin.numberofholes = hole_points.size();
  tin.regionlist = new double[s.NumPieces() * 4];     // x,y,attr,area for polys
  tin.numberofregions = s.NumPieces() - num_holes;
  // Copy all unique points.
  for (int upi = 0; upi < index_map.size(); upi++) {
    int i = index_map[upi].first;
    int j = index_map[upi].second;
    tin.pointlist[2*upi+0] = ToDouble(s.Piece(i)[j].p[0]);
    tin.pointlist[2*upi+1] = ToDouble(s.Piece(i)[j].p[1]);
    tin.pointmarkerlist[upi] = 2 + upi;
  }
  {
    // Copy all segments. Filter out redundant segments. The segment markers
    // are set to (negative) unique index of the first point in the segment,
    // minus one.
    typedef std::map<std::pair<int, int>, bool> SegmentMap;
    SegmentMap segment_map;
    int offset = 0;
    for (int i = 0; i < s.NumPieces(); i++) {
      for (int j = 0; j < s.Piece(i).size(); j++) {
        JetPoint p1 = s.Piece(i)[j].p;
        JetPoint p2 = s.Piece(i)[(j + 1) % s.Piece(i).size()].p;
        PointMap::iterator it1 = point_map.find(std::make_pair(p1[0], p1[1]));
        PointMap::iterator it2 = point_map.find(std::make_pair(p2[0], p2[1]));
        CHECK(it1 != point_map.end() && it2 != point_map.end());
        if (!segment_map[std::make_pair(it1->second, it2->second)]) {
          tin.segmentlist[2*offset+0] = it1->second;
          tin.segmentlist[2*offset+1] = it2->second;
          tin.segmentmarkerlist[offset] = -1 - it1->second;
          segment_map[std::make_pair(it1->second, it2->second)] = true;
          offset++;
        }
      }
    }
    tin.numberofsegments = offset;
  }
  // Set region coordinates and attributes for all non-hole polygons.
  {
    int offset = 0;
    for (int i = 0; i < s.NumPieces(); i++) {
      if (!is_a_hole[i]) {
        // Use AnyPointInPoly() to find a point inside this non-hole polygon.
        // If we didn't consider the holes we may find a point that is actually
        // in a hole, which would give this polygon the wrong region attribute.
        // Thus we must pass both polygon and hole points to AnyPointInPoly.
        vector<RPoint> poly = s.Piece(i);
        for (int j = 0; j < s.NumPieces(); j++) {
          if (is_a_hole[j]) {
            poly.insert(poly.end(), s.Piece(j).begin(), s.Piece(j).end());
          }
        }
        JetPoint point_in_poly;
        AnyPointInPoly(poly, s.Piece(i).size(), &point_in_poly);
        // offsets 0,1 are x,y coord in polygon region.
        tin.regionlist[offset*4 + 0] = ToDouble(point_in_poly[0]);
        tin.regionlist[offset*4 + 1] = ToDouble(point_in_poly[1]);
        tin.regionlist[offset*4 + 2] = i;    // Region attribute (polygon index)
        tin.regionlist[offset*4 + 3] = -1;   // Region max area (ignored)
        offset++;
      }
    }
    CHECK(offset == s.NumPieces() - num_holes);
  }
  // Set coordinates of all holes.
  for (int i = 0; i < hole_points.size(); i++) {
    tin.holelist[i*2 + 0] = ToDouble(hole_points[i].p[0]);
    tin.holelist[i*2 + 1] = ToDouble(hole_points[i].p[1]);
  }

  // Call 'Triangle' library. Use setjmp/longjmp based error handling to catch
  // if the library calls triexit. If this happens then we will leak some
  // memory (no telling what the triangle library was doing internally), but oh
  // well.
  if (setjmp(triangle_jmp_buf) != 0) {
    // triangulate() called triexit.
    return;
  }
  square_of_longest_edge_permitted = sqr(longest_edge_permitted);
  // Useful options to 'triangulate' are:
  //   * z: Index from zero
  //   * p: Triangulate a PSLG
  //   * A: Assign regional attribute to triangles
  //   * Q: Quiet
  //   * V: Verbose (for debugging)
  //   * q: Quality mesh generation by Delaunay refinement
  //   * u: Use triunsuitable function
  //   * n: Create a triangle neighbor list
  if (longest_edge_permitted > 0) {
    triangulate("zpAQqun", &tin, &tout, NULL);
  } else {
    triangulate("zpAQn", &tin, &tout, NULL);
  }

  // Feed output arrays.
  points_.resize(tout.numberofpoints);
  for (int i = 0; i < tout.numberofpoints; i++) {
    points_[i].p[0] = tout.pointlist[i*2 + 0];
    points_[i].p[1] = tout.pointlist[i*2 + 1];

    // Output points that are copied from input points (i.e. are the vertices
    // of boundaries) use the same EdgeInfo. Output points that are created on
    // input segments (i.e. boundary segments) are assigned an EdgeInfo that
    // contains the correct slot information for that edge. The EdgeInfo of
    // output points in the interior of the mesh will never be checked, so we
    // don't do anything regarding those points.
    EdgeInfo e;
    int marker = tout.pointmarkerlist[i];
    if (marker >= 2) {
      // Output point was copied from input point. Copy EdgeInfo of input. Here
      // we rely on the fact, checked above, that duplicate points have
      // consistent EdgeInfo, because we are copying the EdgeInfo from just one
      // of those points.
      int upi = marker - 2;
      CHECK(upi < index_map.size());
      int piece = index_map[upi].first;
      int piece_index = index_map[upi].second;
      e = s.Piece(piece)[piece_index].e;
      points_[i].original_piece = piece;
      points_[i].original_edge = piece_index;
    } else if (marker < 0) {
      // Output point was created on boundary segment. Set both slots of
      // EdgeInfo to the edge kind of the boundary segment.
      int upi = -marker - 1;
      CHECK(upi < index_map.size());
      int piece = index_map[upi].first;
      int piece_index1 = index_map[upi].second;
      int piece_index2 = (piece_index1 + 1) % s.Piece(piece).size();
      float d1, d2;
      const RPoint &p1 = s.Piece(piece)[piece_index1];
      const RPoint &p2 = s.Piece(piece)[piece_index2];
      EdgeKind s = p1.e.SharedKind(p2.e, &d1, &d2);
      e.kind[0] = s;
      // Linearly interpolate distance values.
      JetNum len1 = (points_[i].p - p1.p).squaredNorm();
      JetNum len2 = (p2.p - p1.p).squaredNorm();
      double alpha = ToDouble(sqrt(len1 / len2));
      e.dist[0] = alpha * (d2 - d1) + d1;
      points_[i].original_piece = piece;
      points_[i].original_edge = piece_index1;
    } else if (marker == 1) {
      // A marker value of 1 has a reserved meaning in the triangle library.
      // A marker value of 0 will be assigned to interior points.
      Panic("Internal error, marker==1 found");
    }
    points_[i].e = e;
  }
  CHECK(tout.numberofcorners == 3);
  CHECK(tout.numberoftriangleattributes == 1);  // 1 attr from input regions
  CHECK(tout.triangleattributelist);
  triangles_.resize(tout.numberoftriangles);
  for (int i = 0; i < tout.numberoftriangles; i++) {
    int polygon_index = tout.triangleattributelist[i];
    CHECK(polygon_index == tout.triangleattributelist[i]);       // Is integer?
    CHECK(polygon_index >= 0 && polygon_index < s.NumPieces());  // In range?
    triangles_[i].material = polygon_index;     // Index into materials_
    for (int j = 0; j < 3; j++) {
      triangles_[i].index[j] = tout.trianglelist[i*3 + j];

      // If this edge of the triangle does not have another triangle as a
      // neighbor then it is a boundary edge (indicated by -1 in neighborlist).
      // Note that is it not sufficient to identify boundary edges as ones
      // where both vertices are on the boundary. Interior segments will not
      // end up as accidental boundary edges, so we will not get interior ports
      // in the final mesh.
      triangles_[i].neighbor[j] =
          tout.neighborlist[i*3 + (j + 2) % 3];
    }
  }

  // Copy shape materials and port callbacks.
  materials_.resize(s.NumPieces());
  for (int i = 0; i < s.NumPieces(); i++) {
    materials_[i] = s.GetMaterial(i);
  }
  port_callbacks_ = s.PortCallbacks();

  // Free heap allocated data. Note that holelist and regionlist are copied
  // from tin to tout so make sure not to free them twice.
  DeleteTriangulateIO(&tin);
  tout.holelist = 0;
  tout.regionlist = 0;
  FreeTriangulateIO(&tout);

  valid_mesh_ = true;
  UpdateDerivatives(s);

  if (lua) {
    DeterminePointMaterial(lua, &mat_params_);
    DetermineBoundaryParameters(lua, &boundary_params_);
  }
}

void Mesh::DrawMesh(MeshDrawType draw_type, ColorMap::Function colormap,
                    int brightness, const Matrix4d &camera_transform) {
  if (draw_type == MESH_HIDE) {
    return;
  }

  if (draw_type == MESH_DIELECTRIC_REAL || draw_type == MESH_DIELECTRIC_IMAG ||
      draw_type == MESH_DIELECTRIC_ABS) {
    if (mat_params_.empty()) {
      return;                             // Nothing to show
    }
    // Create color map.
    const int kNumColors = 256;           // Should be even
    float rgb[kNumColors][3];
    for (int i = 0; i < kNumColors; i++) {
      colormap(float(i) / (kNumColors - 1), rgb[i]);
    }

    // Map the brightness to a scale used for min/max values.
    double scale = pow(10, -(brightness - 500.0) / 500.0);
    const double minval = -scale;
    const double maxval = +scale;

    gl::PushShader push_shader(gl::SmoothShader());
    vector<Vector3f> points, colors;
    for (int i = 0; i < triangles_.size(); i++) {
      for (int j = 0; j < 3; j++) {
        int k = triangles_[i].index[j];
        double value;
        if (draw_type == MESH_DIELECTRIC_REAL) {
          value = ToDouble(mat_params_[k].epsilon.real());
        } else if (draw_type == MESH_DIELECTRIC_IMAG) {
          value = ToDouble(mat_params_[k].epsilon.imag());
        } else {
          value = ToDouble(abs(mat_params_[k].epsilon));
        }
        int c = std::max(0, std::min(kNumColors - 1,
          int(round((value - minval) * (kNumColors / (maxval-minval))))));
        colors.push_back(Vector3f(rgb[c][0], rgb[c][1], rgb[c][2]));
        points.push_back(Vector3f(ToDouble(points_[k].p[0]),
                                  ToDouble(points_[k].p[1]), 0));
      }
    }
    gl::Draw(points, colors, GL_TRIANGLES);
    return;
  }

  // Drawing regular mesh, not dielectric.
  gl::SetUniform("color", 1, 0, 0);
  vector<Vector3f> points;
  for (int i = 0; i < triangles_.size(); i++) {
    for (int j = 0; j < 3; j++) {
      int k1 = triangles_[i].index[j];
      int k2 = triangles_[i].index[(j + 1) % 3];
      points.push_back(Vector3f(ToDouble(points_[k1].p[0]),
                                ToDouble(points_[k1].p[1]), 0));
      points.push_back(Vector3f(ToDouble(points_[k2].p[0]),
                                ToDouble(points_[k2].p[1]), 0));
    }
  }
  // @@@ Do GL_LINE_LOOP with element drawing and primitive restarting?
  gl::Draw(points, GL_LINES);

  // Print mesh statistics.
  char s[100];
  snprintf(s, sizeof(s), "%d triangles, %d points",
           (int) triangles_.size(), (int) points_.size());
  DrawString(s, 10, 10, &mesh_statistics_font, 0,0,0);

  // Debug rendering. Highlight special edge kinds (e.g. ports, ABCs).
  if (kDebugMesh) {
    gl::PushShader push_shader(gl::SmoothShader());
    vector<Vector3f> points, colors;
    for (BoundaryIterator it(this); !it.done(); ++it) {
      if (!it.kind().IsDefault()) {
        colors.push_back(Vector3f(0, it.dist1(), 1 - it.dist1()));
      } else {
        colors.push_back(Vector3f(it.dist1(), 0, 0));
      }
      points.push_back(Vector3f(ToDouble(points_[it.pindex1()].p[0]),
                                ToDouble(points_[it.pindex1()].p[1]), 0));
      if (!it.kind().IsDefault()) {
        colors.push_back(Vector3f(0, it.dist2(), 1 - it.dist2()));
      } else {
        colors.push_back(Vector3f(it.dist2(), 0, 0));
      }
      points.push_back(Vector3f(ToDouble(points_[it.pindex2()].p[0]),
                                ToDouble(points_[it.pindex2()].p[1]), 0));

      //@@@ Too much information?
      if (it.kind().PortNumber()) {
        char s[100];
        snprintf(s, sizeof(s), "%d %.3f-%.3f", it.kind().PortNumber(),
                 it.dist1(), it.dist2());
        DrawStringM(s,
                (ToDouble(points_[it.pindex1()].p[0]) +
                 ToDouble(points_[it.pindex2()].p[0])) / 2,
                (ToDouble(points_[it.pindex1()].p[1]) +
                 ToDouble(points_[it.pindex2()].p[1])) / 2, 0, camera_transform,
                &port_number_font, 0,0,0, TEXT_ALIGN_CENTER, TEXT_ALIGN_CENTER);
      }
    }
    gl::DrawThick(10, 10, false, [&]() {
      gl::Draw(points, colors, GL_LINES);
    });
  }
}

void Mesh::DrawPointDerivatives(double scale) {
  gl::SetUniform("color", 0, 0, 1);
  GL(PointSize)(5);
  vector<Vector3f> p;
  for (int i = 0; i < points_.size(); i++) {
    if (points_[i].original_piece >= 0) {
      p.push_back(Vector3f(ToDouble(points_[i].p[0]),
                           ToDouble(points_[i].p[1]), 0));
      p.push_back(Vector3f(
        ToDouble(points_[i].p[0]) + scale * points_[i].p[0].Derivative(),
        ToDouble(points_[i].p[1]) + scale * points_[i].p[1].Derivative(), 0));
    }
  }
  gl::Draw(p, GL_POINTS);
  gl::Draw(p, GL_LINES);
}

void Mesh::UpdateDerivatives(const Shape &s) {
  // Update point derivatives.
  for (int i = 0; i < points_.size(); i++) {
    int p = points_[i].original_piece;
    int e = points_[i].original_edge;
    if (p >= 0) {
      CHECK(p < s.NumPieces() && e >= 0 && e < s.Piece(p).size());
      const RPoint &p1 = s.Piece(p)[e];
      const RPoint &p2 = s.Piece(p)[(e + 1) % s.Piece(p).size()];
      double alpha = ToVector2d(points_[i].p - p1.p).norm() /
                     ToVector2d(p2.p - p1.p).norm();
      points_[i].p[0].Derivative() =
          (1 - alpha)*p1.p[0].Derivative() + alpha*p2.p[0].Derivative();
      points_[i].p[1].Derivative() =
          (1 - alpha)*p1.p[1].Derivative() + alpha*p2.p[1].Derivative();
    }
  }

  // Update material derivatives.
  CHECK(materials_.size() == s.NumPieces());
  for (int i = 0; i < materials_.size(); i++) {
    CHECK(materials_[i] == s.GetMaterial(i));   // Doesn't compare derivatives
    materials_[i] = s.GetMaterial(i);           // Updates derivatives
  }
}

static inline uint64 GridIndex(int32 ix, int32 iy) {
  return (uint64(uint32(ix)) << 32) | uint32(iy);
}

int Mesh::FindTriangle(double x, double y) {
  // Build the spatial index the first time through. We use a
  // "longest_edge_permitted" constraint in meshing, so the triangles we're
  // indexing will not have large aspect ratios and are all roughly the same
  // size. Therefore the spatial index is a grid of cells of side comparable to
  // "longest_edge_permitted" where we just record all triangles that intersect
  // each grid cell.
  const double cell = ldexp(1, cell_size_);
  if (spatial_index_.empty()) {
    for (int i = 0; i < triangles_.size(); i++) {
      // Compute the bounding box of the triangle.
      const JetPoint *tp[3];            // Triangle points
      for (int j = 0; j < 3; j++) {
        tp[j] = &points_[triangles_[i].index[j]].p;
      }
      double xmin = __DBL_MAX__, xmax = -__DBL_MAX__;
      double ymin = __DBL_MAX__, ymax = -__DBL_MAX__;
      for (int j = 0; j < 3; j++) {
        xmin = std::min(xmin, ToDouble((*tp[j])[0]));
        xmax = std::max(xmax, ToDouble((*tp[j])[0]));
        ymin = std::min(ymin, ToDouble((*tp[j])[1]));
        ymax = std::max(ymax, ToDouble((*tp[j])[1]));
      }
      // Determine the grid cells occupied by the triangle's bounding box. Add
      // all grid cells occupied by the triangle to the index.
      int ixmin = floor(xmin / cell);
      int ixmax = floor(xmax / cell);
      int iymin = floor(ymin / cell);
      int iymax = floor(ymax / cell);
      for (int ix = ixmin; ix <= ixmax; ix++) {
        for (int iy = iymin; iy <= iymax; iy++) {
          if (TriangleIntersectsBox(tp, ix * cell, (ix + 1) * cell,
                                    iy * cell, (iy + 1) * cell)) {
            spatial_index_[GridIndex(ix, iy)].push_back(i);
          }
        }
      }
    }
  }

  // Query the spatial index.
  int32 ix = floor(x / cell);
  int32 iy = floor(y / cell);
  SpatialIndex::const_iterator it = spatial_index_.find(GridIndex(ix, iy));
  if (it == spatial_index_.end()) {
    return -1;
  }
  JetPoint test_point;
  test_point[0] = x;
  test_point[1] = y;
  for (int i = 0; i < it->second.size(); i++) {
    const Triangle &tri = triangles_[it->second[i]];
    if (PointInTriangle(test_point, points_[tri.index[0]].p,
                        points_[tri.index[1]].p, points_[tri.index[2]].p)) {
      return it->second[i];
    }
  }
  // No intersecting triangle found.
  return -1;
}

void Mesh::DeterminePointMaterial(Lua *lua,
                                  vector<MaterialParameters> *mat_params) {
  Trace trace(__func__);
  mat_params->clear();
  bool have_callbacks = false;
  for (int i = 0; i < materials_.size(); i++) {
    have_callbacks = have_callbacks || materials_[i].callback.Valid();
  }
  if (!have_callbacks) {
    return;
  }
  mat_params->resize(0);
  mat_params->resize(points_.size());   // Sets material parameters to defaults
  for (int i = 0; i < materials_.size(); i++) {
    // Skip materials without parameter callback functions.
    if (!materials_[i].callback.Valid()) {
      continue;
    }
    // Mark all points that are touched by this material.
    vector<bool> mark(points_.size());
    for (int j = 0; j < triangles_.size(); j++) {
      if (triangles_[j].material == i) {
        for (int k = 0; k < 3; k++) {
          mark[triangles_[j].index[k]] = true;
        }
      }
    }
    // Count points that are touched by this material.
    int count = 0;
    for (int j = 0; j < mark.size(); j++) {
      count += mark[j];
    }
    if (count > 0) {
      // Push the callback function to the lua stack.
      materials_[i].callback.Push(lua->L());
      // Push vectors of x,y coordinates for marked points to the lua stack.
      LuaVector *x = LuaUserClassCreateObj<LuaVector>(lua->L());
      LuaVector *y = LuaUserClassCreateObj<LuaVector>(lua->L());
      x->resize(count);
      y->resize(count);
      count = 0;
      for (int j = 0; j < mark.size(); j++) {
        if (mark[j]) {
          (*x)[count] = points_[j].p[0];
          (*y)[count] = points_[j].p[1];
        }
        count += mark[j];
      }
      // Call the callback function. RunCallback() will pop the callback
      // function and arguments.
      vector<MaterialParameters> result;
      if (!materials_[i].RunCallback(lua, false, &result)) {
        // A lua error message will have been displayed at this point.
        mat_params->clear();
        return;
      }
      // Set point material properties from the callback function's results.
      int index = 0;
      for (int j = 0; j < mark.size(); j++) {
        if (mark[j]) {
          (*mat_params)[j] = result[index];
        }
        index += mark[j];
      }
      CHECK(index == count);
    }
  }
}

void Mesh::DetermineBoundaryParameters(Lua *lua,
                                std::map<RobinArg, RobinRet> *boundary_params) {
  Trace trace(__func__);
  boundary_params->clear();
  int original_top = lua_gettop(lua->L());

  // Scan the boundary and create lists of arguments that will be passed to
  // Robin(), indexed by port number.
  std::map<int, vector<RobinArg>> args;
  std::map<int, vector<double>> dist;   // Argument to callbacks
  std::map<int, vector<JetPoint>> xy;   // Argument to callbacks
  for (BoundaryIterator it(this); !it.done(); ++it) {
    int p = it.kind().PortNumber();
    if (p) {
      // First point of the boundary edge.
      args[p].push_back(RobinArg(it.tindex(), it.tside(), it.tside()));
      dist[p].push_back(it.dist1());
      xy[p].push_back(points_[it.pindex1()].p);

      // Second point of the boundary edge.
      args[p].push_back(RobinArg(it.tindex(), it.tside(), (it.tside() + 1) % 3));
      dist[p].push_back(it.dist2());
      xy[p].push_back(points_[it.pindex2()].p);
    }
  }

  // For all port numbers with callback functions, generate boundary
  // parameters.
  for (auto it : args) {
    int port_number = it.first;
    if (port_callbacks_.count(port_number) == 0) {
      continue;
    }
    vector<RobinArg> &arg = it.second;
    port_callbacks_[port_number].Push(lua->L());                // fn

    // Create the Lua vectors that will be passed to the callback function.
    LuaVector *d = LuaUserClassCreateObj<LuaVector>(lua->L());  // fn d
    LuaVector *x = LuaUserClassCreateObj<LuaVector>(lua->L());  // fn d x
    LuaVector *y = LuaUserClassCreateObj<LuaVector>(lua->L());  // fn d x y
    d->resize(arg.size());
    x->resize(arg.size());
    y->resize(arg.size());
    for (int i = 0; i < arg.size(); i++) {
      (*d)[i] = dist[port_number][i];
      (*x)[i] = xy[port_number][i][0];
      (*y)[i] = xy[port_number][i][1];
    }

    // Call the function, check the return values.
    // @@@ This duplicates code in Material::RunCallback, share the code?
    int n = lua_gettop(lua->L()) - 3;     // First return argument slot
    int code = lua->PCall(3, LUA_MULTRET);            // ret1 ret2
    if (code != LUA_OK) {
      return;
    }
    int nret = lua_gettop(lua->L()) - n + 1;
    if (nret != 2) {
      Error("Port callback should return 2 boundary parameters");
      return;
    }
    vector<vector<JetComplex>> vec(nret);
    for (int i = 0; i < nret; i++) {
      if (!ToJetComplexVector(lua->L(), n + i, &vec[i])) {
        Error("Invalid vectors (or complex vectors) returned by port "
              "callback");
        return;
      }
      if (vec[i].size() != arg.size()) {
        Error("Port callback should return vectors (or complex "
              "vectors) of the same size as the d,x,y arguments");
        return;
      }
      for (int j = 0; j < vec[i].size(); j++) {
        if (IsNaNValue(vec[i][j].real()) || IsNaNValue(vec[i][j].imag())) {
          Error("A NaN (not-a-number) was returned by a callback, "
                "in position %d of return value %d", j+1, i+1);
          return;
        } else if (isnan(vec[i][j].real()) || isnan(vec[i][j].imag())) {
          Warning("A NaN (not-a-number) derivative was returned by a callback, "
                  "in position %d of return value %d\n"
                  "This will prevent the optimizer from working.", j+1, i+1);
        }
      }
    }
    for (int i = 0; i < arg.size(); i++) {
      (*boundary_params)[arg[i]] = RobinRet(vec[0][i], vec[1][i]);
    }
    lua_pop(lua->L(), 2);                             // empty stack
  }
  CHECK(lua_gettop(lua->L()) == original_top);
}

//***************************************************************************
// Testing.

TEST_FUNCTION(SpatialIndex) {
  const double kGridSize = 0.1;
  Shape s;
  s.AddPoint(0, 0);
  s.AddPoint(1, 0);
  s.AddPoint(1, 1);
  s.AddPoint(0, 1);
  Mesh m(s, kGridSize, NULL);

  // Create spatial index and make sure each triangle in each grid cell
  // intersects the cell.
  m.FindTriangle(0, 0);
  const double kCell = ldexp(1, m.cell_size_);
  printf("Spatial index for %d points, %d triangles:\n",
         (int)m.points_.size(), (int)m.triangles_.size());
  for (int y = -2; y <= 34; y++) {
    for (int x = -2; x <= 34; x++) {
      uint64 index = GridIndex(x, y);
      Mesh::SpatialIndex::const_iterator it = m.spatial_index_.find(index);
      if (it == m.spatial_index_.end()) {
        printf(" .");
      } else {
        int n = it->second.size();
        printf("%2d", n);
        for (int i = 0; i < n; i++) {
          int index = it->second[i];
          const JetPoint *p[3];
          for (int k = 0; k < 3; k++) {
            p[k] = &m.points_[m.triangles_[index].index[k]].p;
          }
          CHECK(TriangleIntersectsBox(p, x*kCell, (x+1)*kCell,
                                         y*kCell, (y+1)*kCell));
        }
      }
    }
    printf("\n");
  }

  // Do a bunch of queries and make sure the returned triangles intersect the
  // query points.
  for (int i = 0; i < 10000; i++) {
    double x = RandDouble()*0.998 + 0.001;
    double y = RandDouble()*0.998 + 0.001;
    int t = m.FindTriangle(x, y);
    CHECK(t >= 0 && t < m.triangles_.size());
    JetPoint xy;
    xy[0] = x;
    xy[1] = y;
    CHECK(PointInTriangle(xy, m.points_[m.triangles_[t].index[0]].p,
                              m.points_[m.triangles_[t].index[1]].p,
                              m.points_[m.triangles_[t].index[2]].p) != 0);
  }

  // Do a bunch of queries outside the shape.
  for (int i = 0; i < 10000; i++) {
    double x = RandDouble();
    x += (x > 0.5) ? 0.6 : -0.6;
    double y = RandDouble();
    y += (y > 0.5) ? 0.6 : -0.6;
    int t = m.FindTriangle(x, y);
    CHECK(t == -1);
  }
}
