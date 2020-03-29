
#ifndef __EDGE_TYPE_H__
#define __EDGE_TYPE_H__

// The different kinds of edges.
class EdgeKind {
 public:
  // Default edge kind is DEFAULT, which can have a solver-specific meaning
  // (e.g. a metal wall for an electromagnetic cavity).
  EdgeKind() : value_(DEFAULT) {}

  // Set and test various kinds of edges. Aside from these specific kinds there
  // are also port numbers.
  void SetDefault() { value_ = DEFAULT; }
  void SetDirichlet() { value_ = DIRICHLET; }   // Dirichlet boundary
  void SetNeumann() { value_ = NEUMANN; }       // Neumann boundary
  void SetABC() { value_ = ABC; }               // Absorbing boundary condition
  bool IsDefault() const { return value_ == DEFAULT; }
  bool IsDirichlet() const { return value_ == DIRICHLET; }
  bool IsNeumann() const { return value_ == NEUMANN; }
  bool IsABC() const { return value_ == ABC; }
  int IntegerForDebugging() const { return int(value_); }

  // Convert a port number p (>= 1) to an EdgeKind.
  explicit EdgeKind(int port_number) : value_(port_number - 1 + PORT1) {}
  static int MaxPort() { return 0xffff - PORT1 + 1; }

  // If this represents a numbered port then return the number (>= 1) otherwise
  // return 0 (in which case one of the other Is*() methods will return true).
  int PortNumber() const {
    return (value_ >= PORT1) ? (value_ - PORT1 + 1) : 0;
  }

  // Comparison.
  bool operator==(EdgeKind a) const { return value_ == a.value_; }

 private:
  enum {
    DEFAULT, DIRICHLET, NEUMANN, ABC,
    PORT1,                // First port number, other ports increment from here
  };

  uint16_t value_;
};

struct EdgeInfo {
  // Extra information about a point in a polygon. There are two slots, each
  // with a separate edge kind and port distance. This is used as clipper's
  // IntPoint::Z field. Both bounding vertices of an edge with kind 'K' have
  // either slot 0 or 1 with kind K. Port distances are in the range 0..1. Both
  // slots can not have the same kind, to avoid ambiguity about which distance
  // value to use. This scheme is designed to allow clipper's ZFillFunction to
  // propagate the edge information correctly when polygons are intersected,
  // and allows adjacent edges to have distinct edge kinds. Note that interior
  // points of a mesh will have an EdgeInfo also, but it is not used for
  // anything.
  //
  // This cumbersome scheme for assigning properties to edges is a consequence
  // of the clipper API dealing with vertices not edges in its callback. If we
  // designed our own polygon clipper then edge properties would be assigned to
  // edges and not distributed among the vertices.

  EdgeKind kind[2];      // Kind for slots 1 and 2
  float dist[2];         // Distance for slots 1 and 2

  EdgeInfo() {
    // The default edge kind does not need to worry about port distances
    // (unlike actual ports).
    dist[0] = dist[1] = 0;
  }

  bool operator==(const EdgeInfo &e) const {
    return kind[0] == e.kind[0] && kind[1] == e.kind[1] &&
           dist[0] == e.dist[0] && dist[1] == e.dist[1];
  }
  bool operator!=(const EdgeInfo &e) const { return !operator==(e); }

  // Return true if this object has its 'just initialized' state.
  bool IsDefault() const {
    return kind[0].IsDefault() && kind[1].IsDefault() &&
           dist[0] == 0 && dist[1] == 0;
  }

  // Return the non-default edge kind shared with 'e', or return the default
  // edge kind if there is none. Also return the two corresponding distances
  // for this object and e (if the float pointers are nonzero).
  EdgeKind SharedKind(EdgeInfo e, float *Dthis = 0, float *De = 0) const;

  // Set an unused (default) slot to (new_kind,new_dist) and return true, or
  // return false if no slot is unused. If there is already a slot for this
  // kind then reuse that.
  bool SetUnused(EdgeKind new_kind, float new_dist);
};

// This struct will be part of Clipper's IntPoint. Include the derivative of
// the *unscaled* point coordinates with respect to some parameter.
struct ClipperEdgeInfo : public EdgeInfo {
  double derivative_x, derivative_y;
  ClipperEdgeInfo() {
    derivative_x = derivative_y = 0;
  }
  ClipperEdgeInfo(const EdgeInfo &e, double dx, double dy) : EdgeInfo(e) {
    derivative_x = dx;
    derivative_y = dy;
  }
  bool IsDefault() const {
    return kind[0].IsDefault() && kind[1].IsDefault() && dist[0] == 0 &&
           dist[1] == 0 && derivative_x == 0 && derivative_y == 0;
  }
  void SetDefault() {
    kind[0].SetDefault();
    kind[1].SetDefault();
    dist[0] = 0;
    dist[1] = 0;
    derivative_x = derivative_y = 0;
  }
};

#endif
