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

#ifndef __MESH_H__
#define __MESH_H__

#include "shape.h"
#include "../toolkit/lua_util.h"
#include "../toolkit/colormaps.h"
#include <map>

class Mesh {
 public:
  // Triangulate the shape to create a mesh. The shape coordinates are in
  // config units not meters. IsValid() will be false on failure (in which an
  // Error() will have been emitted). If longest_edge_permitted is > 0 then
  // triangles will be subdivided to satisfy this constraint, otherwise a
  // triangulation with a small number of triangles will be produced. If 'lua'
  // is provided the dielectric callback functions can be called.
  explicit Mesh(const Shape &s, double longest_edge_permitted, Lua *lua);

  // Did mesh creation succeed?
  bool IsValidMesh() const { return valid_mesh_; }

  // Get mesher results.
  const vector<RPoint> &points() { return points_; }
  const vector<Triangle> &triangles() { return triangles_; }
  const vector<Material> &materials() { return materials_; }

  // Draw the mesh to OpenGL. If the mesh is empty this does nothing.
  enum MeshDrawType {   // These enums match the 'mesh' choice box
    MESH_HIDE = 0,
    MESH_SHOW,
    MESH_DIELECTRIC_REAL,
    MESH_DIELECTRIC_IMAG,
    MESH_DIELECTRIC_ABS,
  };
  void DrawMesh(MeshDrawType draw_type, ColorMap::Function colormap,
                int brightness, const Eigen::Matrix4d &camera_transform);

  // For debugging draw the derivative vectors at boundary points.
  void DrawPointDerivatives(double scale);

  // Update the derivatives of mesh points and materials from the derivatives
  // of points and materials in 's'. The shape 's' must have exactly the same
  // structure as the original 's' given to the constructor, only differing in
  // the derivatives.
  void UpdateDerivatives(const Shape &s);

  // Return the triangle index that intersects (x,y), or return -1 if none.
  int FindTriangle(double x, double y);

  // Encapsulate arguments (i,j,k) and return values (alpha, beta) of the
  // solver's Robin() function. Return values are for Robin edge j of triangle
  // i, at point k.
  typedef std::tuple<int, int, int> RobinArg;
  typedef std::tuple<JetComplex, JetComplex> RobinRet;

 protected:
  bool valid_mesh_;
  vector<RPoint> points_;
  vector<Triangle> triangles_;
  vector<Material> materials_;          // Copies of shape piece materials
  std::map<int, Shape::CallbackInfo> port_callbacks_;  // Copied from shape
  double cd_width_=0, cd_height_=0;     // CD dimensions (in config units)
  friend class BoundaryIterator;
  // Optional, material parameters at each point (size = 0 or points_.size()).
  vector<MaterialParameters> mat_params_;
  // Optional boundary parameters. This maps Robin() arguments to alpha, beta.
  std::map<RobinArg, RobinRet> boundary_params_;
  // Spatial index that is built when FindTriangle() is called.
  int cell_size_;                       // Spatial index cell size is 2^this
  typedef std::map<uint64, std::vector<int> > SpatialIndex;
  SpatialIndex spatial_index_;

  // For drawing a 3D mesh:
 vector<Eigen::Vector3d> tri_normal1_, tri_normal2_;

  // If any materials have callback functions to determine their material
  // parameters, call them and populate the epsilon and (optionally) sigma
  // values in mat_params. vectors. Otherwise clear mat_params.
  void DeterminePointMaterial(Lua *lua, vector<MaterialParameters> *mat_params);

  // If any ports have callback functions to determine their boundary
  // parameters, call them and populate boundary_params_.
  void DetermineBoundaryParameters(Lua *lua,
                                 std::map<RobinArg, RobinRet> *boundary_params);

  // Setup tri_normal1_ etc so that triangle normals can be computed from
  //   Vector3d v(z1,z2,z3);    // Triangle vertex Z values
  //   normal = Vector3d(tri_normal1_[i].dot(v), tri_normal2_[i].dot(v), 1);
  //   normal.normalize();
  void SetupTriNormal();

  // For testing:
  friend void __RunTest_SpatialIndex();
};

// An iterator for mesh edges. If the color mask is zero, iterate over all
// boundary edges of all triangles in the mesh. If the color mask is nonzero,
// iterate over all edges that are boundaries between triangles without those
// color bits and triangles with those color bits. The latter is useful for
// traversing the boundary at which the far field should be computed, i.e.
// where the color mask is Material::FAR_FIELD.

class BoundaryIterator {
 public:
  explicit BoundaryIterator(Mesh *mesh, uint32_t color_mask = 0) {
    mesh_ = mesh;
    tindex_ = -1;
    tside_ = 2;
    color_mask_ = color_mask;
    operator++();
  }
  void operator++() {
    for (;;) {
      tside_++;
      if (tside_ >= 3) {
        tside_ = 0;
        tindex_++;
      }
      if (done()) break;
      auto &t = mesh_->triangles_[tindex_];   // Triangle we're currently on
      if (color_mask_ == 0) {
        if (t.neighbor[tside_] == -1) break;  // Select edges with no neighbors
      } else {
        if ((mesh_->materials_[t.material].color & color_mask_) == 0) {
          int neighbor = t.neighbor[tside_];  // Select edges with color-masked
          if (neighbor != -1 &&               //   neighbors
              (mesh_->materials_[mesh_->triangles_[neighbor].material].color &
               color_mask_)) break;
        }
      }
    }
    if (!done()) {
      pindex1_ = mesh_->triangles_[tindex_].index[tside_];
      pindex2_ = mesh_->triangles_[tindex_].index[(tside_ + 1) % 3];
      pindex3_ = mesh_->triangles_[tindex_].index[(tside_ + 2) % 3];
      kind_ = mesh_->points_[pindex1_].e.SharedKind(
              mesh_->points_[pindex2_].e, &dist1_, &dist2_);
    }
  }
  bool done() const { return tindex_ >= mesh_->triangles_.size(); }
  int tindex() const { return tindex_; }       // Triangle index
  int tside() const { return tside_; }         // Triangle side (0..2)
  int pindex1() const { return pindex1_; }     // Point 1 index, boundary edge
  int pindex2() const { return pindex2_; }     // Point 2 index, boundary edge
  int pindex3() const { return pindex3_; }     // 3rd index of triangle
  float dist1() const { return dist1_; }       // Point 1 dist
  float dist2() const { return dist2_; }       // Point 2 dist
  EdgeKind kind() const { return kind_; }      // Edge kind
 private:
  uint32_t color_mask_;
  int tindex_, tside_, pindex1_, pindex2_, pindex3_;
  EdgeKind kind_;
  float dist1_, dist2_;
  Mesh *mesh_;
};

#endif
