
#ifndef __MESH_H__
#define __MESH_H__

#include "shape.h"
#include "../toolkit/lua_util.h"
#include "../toolkit/colormaps.h"
#include <map>

class Mesh {
 public:
  // Triangulate the shape to create a mesh. IsValid() will be false on failure
  // (in which an Error() will have been emitted). If longest_edge_permitted is
  // > 0 then triangles will be subdivided to satisfy this constraint,
  // otherwise a triangulation with a small number of triangles will be
  // produced. If 'lua' is provided the dielectric callback functions can be
  // called.
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
  std::map<int, LuaCallback> port_callbacks_;  // Copied from shape
  friend class BoundaryIterator;
  // Optional, material parameters at each point (size = 0 or points_.size()).
  vector<MaterialParameters> mat_params_;
  // Optional boundary parameters. This maps Robin() arguments to alpha, beta.
  std::map<RobinArg, RobinRet> boundary_params_;
  // Spatial index that is built when FindTriangle() is called.
  int cell_size_;                       // Spatial index cell size is 2^this
  typedef std::map<uint64, std::vector<int> > SpatialIndex;
  SpatialIndex spatial_index_;

  // If any materials have callback functions to determine their material
  // parameters, call them and populate the epsilon and (optionally) sigma
  // values in mat_params. vectors. Otherwise clear mat_params.
  void DeterminePointMaterial(Lua *lua, vector<MaterialParameters> *mat_params);

  // If any ports have callback functions to determine their boundary
  // parameters, call them and populate boundary_params_.
  void DetermineBoundaryParameters(Lua *lua,
                                 std::map<RobinArg, RobinRet> *boundary_params);

  // For testing:
  friend void __RunTest_SpatialIndex();
};

// Iterate over all boundary edges of all triangles in a mesh.

class BoundaryIterator {
 public:
   explicit BoundaryIterator(Mesh *mesh) {
     mesh_ = mesh;
     tindex_ = -1;
     tside_ = 2;
     operator++();
   }
   void operator++() {
     do {
       tside_++;
       if (tside_ >= 3) {
         tside_ = 0;
         tindex_++;
       }
     } while (!done() && mesh_->triangles_[tindex_].neighbor[tside_] != -1);
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
  int tindex_, tside_, pindex1_, pindex2_, pindex3_;
  EdgeKind kind_;
  float dist1_, dist2_;
  Mesh *mesh_;
};

#endif
