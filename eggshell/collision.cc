
#include "collision.h"
#include "error.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using std::vector;

//***************************************************************************
// Utility.

struct Box {
  Vector3d center;       // Position of the box center
  Matrix3d R;            // Orientation of the box
  Vector3d halfside;     // Half side lengths
};

namespace {

template <class T> inline T sqr(T x) {
  return x * x;
}

template<class T> inline T Sign(T a) {
  return (a >= 0) ? 1 : -1;
}

// Print box information in matlab format, for debugging.

void PrintBox(const Box &A, const char *prefix) MAY_NOT_BE_USED;
void PrintBox(const Box &A, const char *prefix = "") {
  printf("%sR=[%.15e,%.15e,%.15e;%.15e,%.15e,%.15e;%.15e,%.15e,%.15e];",
         prefix, A.R(0,0), A.R(0,1), A.R(0,2), A.R(1,0), A.R(1,1), A.R(1,2),
         A.R(2,0), A.R(2,1), A.R(2,2));
  printf("%sc=[%.15e,%.15e,%.15e];", prefix,
         A.center[0], A.center[1], A.center[2]);
  printf("%ss=[%.15e,%.15e,%.15e];\n", prefix,
         A.halfside[0], A.halfside[1], A.halfside[2]);
}

// Given lines pa+alpha*ua and pb+beta*ub, compute the alpha and beta for the
// point of closest approach. The vectors ua and ub are assumed to be unit
// length.

void LineClosestApproach(const Vector3d &pa, const Vector3d &ua,
                         const Vector3d &pb, const Vector3d &ub,
                         double *alpha, double *beta) {
  Vector3d p = pb - pa;
  double uaub = ua.dot(ub);
  double q1 =  ua.dot(p);
  double q2 = -ub.dot(p);
  double d = 1 - uaub*uaub;
  if (d == 0) {
    *alpha = 0;
    *beta  = 0;
  } else {
    *alpha = (q1 + uaub*q2) / d;
    *beta  = (uaub*q1 + q2) / d;
  }
}

// Compute the intersection point between a line segment p1,p2 and a line
// defined by normal.P + d == 0. The normal does not need to be unit length.
// Return true if there is an intersection and return the intersection point in
// p. This does not consider end points p1 or p2 to be part of the line segment
// so will not return them if they are exactly on the line.

bool IntersectLineSegmentAndLine(const Vector2d &p1, const Vector2d &p2,
                                 const Vector2d &normal, double d,
                                 Vector2d *p) {
  double k1 = normal.dot(p1) + d;
  double k2 = normal.dot(p2) + d;
  if (k1 * k2 < 0) {
    *p = p1 - (k1 / (k2 - k1))*(p2 - p1);
    return true;
  }
  return false;
}

// Clip a 2D polygon with the halfspace defined by normal.P + d >= 0.

void ClipPolygonByHalfSpace(const vector<Vector2d> &poly,
                            const Vector2d &normal, double d,
                            vector<Vector2d> *newpoly) {
  newpoly->clear();
  for (int i = 0; i < poly.size(); i++) {
    if (normal.dot(poly[i]) + d >= 0)
      newpoly->push_back(poly[i]);
    // If the line segment from this point to the next crosses the half space
    // then add a new point.
    Vector2d newp;
    if (IntersectLineSegmentAndLine(poly[i], poly[(i + 1) % poly.size()],
                                    normal, d, &newp)) {
      newpoly->push_back(newp);
    }
  }
}

// Intersect box B with a rectangle R. Return the intersection polygon in the R
// coordinate system (where 0,0 is the center of R). If the returned polygon is
// empty then there was no intersection.

void IntersectBoxAndRectangle(const Box &B, const Box &R,
                              vector<Vector2d> *poly) {
  const double kTolerance = 1e-9;

  // To help limit numerical errors and simplify some math, we translate
  // everything so the rectangle center is the origin. The polygon result is
  // expressed in the rectangle coordinate system so this doesn't change that.
  Vector3d Bc = B.center - R.center;

  // Start with a 2D polygon for rectangle R in the R coordinate system.
  poly->clear();
  poly->push_back(Vector2d(-R.halfside(0), -R.halfside(1)));
  poly->push_back(Vector2d(-R.halfside(0),  R.halfside(1)));
  poly->push_back(Vector2d( R.halfside(0),  R.halfside(1)));
  poly->push_back(Vector2d( R.halfside(0), -R.halfside(1)));
  vector<Vector2d> newpoly;
  newpoly.reserve(poly->size());

  // Rectangle R lies in the plane: Rnormal.X == 0.
  Vector3d Rnormal = R.R.col(2);

  // The box is the intersection of 6 half spaces. Project the plane for each
  // half space to a line in the R coordinate system and use it to clip the
  // rectangle.
  for (int i = 0; i < 3; i++) {             // For each axis
    Vector3d Bnormal = B.R.col(i);          // Normal of the clip plane
    double BnBc = Bnormal.dot(Bc);
    double cross = Bnormal.cross(Rnormal).norm();
    for (int j = -1; j <= 1; j += 2) {      // For each box side for this axis
      // The clipping half space is: Bnormal.X + Bd <= 0.
      double Bd = -j*BnBc - B.halfside(i);

      // If the planes are nearly parallel (Bnormal ~= +/-Rnormal) then accept
      // or reject the entire polygon based on whether the rectangle plane
      // intersects the half space.
      if (cross < kTolerance) {
        if (Bd <= 0) {
          continue;             // Accept
        } else {
          poly->clear();        // Reject
          return;
        }
      }

      // Clip the current (convex) 2D polygon with the half space.
      Vector2d Hnormal(R.R.col(0).dot(Bnormal), R.R.col(1).dot(Bnormal));
      ClipPolygonByHalfSpace(*poly, -j*Hnormal, -Bd, &newpoly);
      newpoly.swap(*poly);

      // If there are no points left in the polygon there is no intersection.
      if (poly->empty())
        return;
    }
  }
}

}  // anonymous namespace

//***************************************************************************
// Box-box collision.

bool CollideBoxes(const Box &box1, const Box &box2,
                  CollisionInfo *info,
                  std::vector<ContactGeometry> *contacts) {
  // For the box separation test we use 15 separating axes: the 6 box face
  // normals and the 9 edge1 x edge2 axes (where edge1/edge2 is one of the
  // three box 1 or box 2 edges). We will generate contacts using one of the
  // following strategies (where R is the rotation matrix between box1 and
  // box2):
  //   * Face-face: Two or three of the columns in R are nearly axis aligned.
  //     Contacts are generated from a face-box intersection.
  //   * Face-edge: One of the columns in R is nearly axis aligned. Contacts
  //     are generated from a face-box intersection.
  //   * Face-corner: None of the columns in R is axis aligned but a box face
  //     normal is the minimum distance separating axis. Contacts are generated
  //     from a face-box intersection.
  //   * Edge-edge: None of the columns in R is axis aligned and an edge1 x
  //     edge2 vector is the minimum distance separating axis. Contacts are
  //     generated from an edge-edge intersection.
  // The best definition of "nearly axis aligned" depends on the rigid body
  // dynamics - we want to transition to multiple face-face contacts for
  // stacking stability when the faces are misaligned by an amount at least
  // similar to one timestep. The threshholds here are therefore set
  // heuristically.
  const double kAlignmentTolerance = 0.9962;    // cos(threshhold_angle)
  const double kTolerance = 1e-9;

  // To simplify the computations below we express box2 in box1's frame.
  Matrix3d R1 = box1.R.matrix();
  Matrix3d R2 = box2.R.matrix();
  Matrix3d R = R1.transpose() * R2;                         // box2 rotation
  Vector3d p = R1.transpose()*(box2.center - box1.center);  // box2 center
  Matrix3d Q = R.cwiseAbs();

  // Count the number of nearly axis aligned columns in R.
  int aacount = 0;
  for (int i = 0; i < 3; i++) {
    aacount += (R.col(i).cwiseAbs().maxCoeff() > kAlignmentTolerance);
  }

  // For each separating axis, if it separates the boxes then return. If not,
  // remember the smallest depth so far and its axis. Remember these separately
  // for face-normal and edge x edge separating axes.
  double min_depth_FN = -__DBL_MAX__;   // Negative of smallest depth so far
  Vector3d sepaxis_FN;                  // Separating axis for min_depth_FN
  int code_FN = 0;                      // Contact classification

  // Test separating axes which are the 3 axis unit vectors (faces of box1).
  // Then test separating axes which are the three columns of R (faces of
  // box2).
  #define SEP(expr1, expr2, normal, thecode) { \
    double e1 = (expr1); \
    double separation = fabs(e1) - (expr2); \
    if (separation > 0) return false; \
    if (separation > min_depth_FN) { \
      min_depth_FN = separation; \
      sepaxis_FN = Sign(e1) * normal; \
      code_FN = (thecode); \
    } \
  }
  const Vector3d &H1 = box1.halfside;
  const Vector3d &H2 = box2.halfside;
  SEP(p[0], H1[0] + H2.dot(Q.row(0)), R1.col(0), 1)
  SEP(p[1], H1[1] + H2.dot(Q.row(1)), R1.col(1), 2)
  SEP(p[2], H1[2] + H2.dot(Q.row(2)), R1.col(2), 3)
  SEP(R.col(0).dot(p), H1.dot(Q.col(0)) + H2[0], R2.col(0), 4)
  SEP(R.col(1).dot(p), H1.dot(Q.col(1)) + H2[1], R2.col(1), 5)
  SEP(R.col(2).dot(p), H1.dot(Q.col(2)) + H2[2], R2.col(2), 6)
  #undef SEP

  // Test separating axes which are the edge directions of box 1 crossed with
  // the edge directions of box 2. The normal (n0,n1,n2) is relative to box 1
  // and is not unit length, so the normal and separation need to be scaled. If
  // the unscaled normal is zero length (within a tolerance) then this means
  // box axes are nearly aligned and this test must be skipped.
  double min_depth_EE = -__DBL_MAX__;   // Negative of smallest depth so far
  Vector3d sepaxis_EE;                  // Separating axis for min_depth_EE
  int code_EE = 0;                      // Contact classification
  #define SEP(expr1, expr2, n0, n1, n2, thecode) { \
    Vector3d n(n0, n1, n2); \
    double len = n.norm(); \
    if (len > kTolerance) { \
      double e1 = (expr1); \
      double separation = fabs(e1) - (expr2); \
      if (separation > 0) return false; \
      separation /= len; \
      if (separation > min_depth_EE) { \
        min_depth_EE = separation; \
        sepaxis_EE = n / (Sign(e1) * len); \
        code_EE = (thecode); \
      } \
    } \
  }
  SEP(p[2]*R(1,0)-p[1]*R(2,0), (H1[1]*Q(2,0)+H1[2]*Q(1,0)+
                                H2[1]*Q(0,2)+H2[2]*Q(0,1)), 0,-R(2,0),R(1,0),7)
  SEP(p[2]*R(1,1)-p[1]*R(2,1), (H1[1]*Q(2,1)+H1[2]*Q(1,1)+
                                H2[0]*Q(0,2)+H2[2]*Q(0,0)), 0,-R(2,1),R(1,1),8)
  SEP(p[2]*R(1,2)-p[1]*R(2,2), (H1[1]*Q(2,2)+H1[2]*Q(1,2)+
                                H2[0]*Q(0,1)+H2[1]*Q(0,0)), 0,-R(2,2),R(1,2),9)
  SEP(p[0]*R(2,0)-p[2]*R(0,0), (H1[0]*Q(2,0)+H1[2]*Q(0,0)+
                                H2[1]*Q(1,2)+H2[2]*Q(1,1)), R(2,0),0,-R(0,0),10)
  SEP(p[0]*R(2,1)-p[2]*R(0,1), (H1[0]*Q(2,1)+H1[2]*Q(0,1)+
                                H2[0]*Q(1,2)+H2[2]*Q(1,0)), R(2,1),0,-R(0,1),11)
  SEP(p[0]*R(2,2)-p[2]*R(0,2), (H1[0]*Q(2,2)+H1[2]*Q(0,2)+
                                H2[0]*Q(1,1)+H2[1]*Q(1,0)), R(2,2),0,-R(0,2),12)
  SEP(p[1]*R(0,0)-p[0]*R(1,0), (H1[0]*Q(1,0)+H1[1]*Q(0,0)+
                                H2[1]*Q(2,2)+H2[2]*Q(2,1)), -R(1,0),R(0,0),0,13)
  SEP(p[1]*R(0,1)-p[0]*R(1,1), (H1[0]*Q(1,1)+H1[1]*Q(0,1)+
                                H2[0]*Q(2,2)+H2[2]*Q(2,0)), -R(1,1),R(0,1),0,14)
  SEP(p[1]*R(0,2)-p[0]*R(1,2), (H1[0]*Q(1,2)+H1[1]*Q(0,2)+
                                H2[0]*Q(2,1)+H2[1]*Q(2,0)), -R(1,2),R(0,2),0,15)
  #undef SEP

  // The boxes intersect. Return the separating axis and distance in global
  // coordinates.
  CHECK(code_FN != 0 && code_EE != 0);
  sepaxis_EE = R1 * sepaxis_EE;
  bool best_sepaxis_is_FN = (min_depth_FN > min_depth_EE);
  if (info) {
    if (best_sepaxis_is_FN) {
      info->depth = -min_depth_FN;
      info->separating_axis = sepaxis_FN;
    } else {
      info->depth = -min_depth_EE;
      info->separating_axis = sepaxis_EE;
    }
  }

  // Compute one or more contact points.

  if (aacount == 0 && !best_sepaxis_is_FN) {
    // An edge from box 1 touches an edge from box 2. Find points 'pa' and 'pb'
    // on the intersecting edges of box 1 and box 2. The contact normal is
    // sepaxis_EE.
    if (info) {
      info->code = code_EE;
    }
    Vector3d pa = box1.center, pb = box2.center;
    for (int j = 0; j < 3; j++) {
      pa += Sign(sepaxis_EE.dot(R1.col(j))) * H1[j] * R1.col(j);
      pb -= Sign(sepaxis_EE.dot(R2.col(j))) * H2[j] * R2.col(j);
    }
    // Get the edge directions and set the contact point between the points of
    // closest approach.
    Vector3d ua = R1.col((code_EE - 7) / 3);
    Vector3d ub = R2.col((code_EE - 7) % 3);
    double alpha, beta;
    LineClosestApproach(pa, ua, pb, ub, &alpha, &beta);
    contacts->resize(contacts->size() + 1);
    contacts->back().position = (pa + ua * alpha + pb + ub * beta) * 0.5;
    contacts->back().normal = sepaxis_EE;
    contacts->back().depth = -min_depth_EE;
    return true;
  }

  // We have a face-something intersection (because the separating axis is
  // perpendicular to a face or columns of R are axis aligned). Box 'A' will be
  // the one with the reference face (i.e. the separating axis is perpendicular
  // to this face) and box 'B' is the other ("incident") box. The contact
  // normal is sepaxis_FN.
  if (info) {
    info->code = code_FN;
  }
  const Box &A((code_FN <= 3) ? box1 : box2);
  Box B((code_FN <= 3) ? box2 : box1);
  Vector3d Aface_normal = sepaxis_FN * ((code_FN <= 3) ? 1 : -1);

  // Find the nearest face of the incident box (the "incident face"). We
  // collide the reference box with the incident face.
  Vector3d nf = B.R.transpose() * Aface_normal;
  int nf_index;
  nf.cwiseAbs().maxCoeff(&nf_index);
  Vector3d Bface_normal = -Sign(nf[nf_index]) * B.R.col(nf_index);

  // Adjust box B so that it is the incident face's rectangle rather than the
  // whole box. The axes must be permuted so that the rectangle axes are the
  // first two (for IntersectBoxAndRectangle).
  {
    B.center += Bface_normal * B.halfside[nf_index];
    Matrix3d BR;
    BR.col(0) = B.R.col((nf_index + 1) % 3);
    BR.col(1) = B.R.col((nf_index + 2) % 3);
    BR.col(2) = B.R.col(nf_index);
    B.R = BR;
    Vector3d Bhalfside(B.halfside[(nf_index + 1) % 3],
                       B.halfside[(nf_index + 2) % 3], 0);
    B.halfside = Bhalfside;
  }

  // The contact depth will be the depth below the reference face. Get the
  // reference face plane equation Aface_normal.P+Ad=0.
  Vector3d AfaceCenter = A.center + Aface_normal *
                                    A.halfside[(code_FN - 1) % 3];
  double Ad = -Aface_normal.dot(AfaceCenter);

  // Intersect A and B. Convert the resulting polygon points, which will be in
  // the B coordinate system, into contacts in the global frame.
  vector<Vector2d> poly;
  IntersectBoxAndRectangle(A, B, &poly);
  for (int i = 0; i < poly.size(); i++) {
    Vector3d pos = B.center + B.R.col(0) * poly[i][0] + B.R.col(1) * poly[i][1];
    double depth = -(Aface_normal.dot(pos) + Ad);
    // Heuristic for contact filtering: We only emit zero-depth contacts if
    // aacount >= 2.
    if (fabs(depth) > kTolerance || aacount >= 2) {
      contacts->push_back(ContactGeometry(pos, sepaxis_FN, depth));
    }
  }

  // If the incident face is entirely below the reference box then there will
  // be no contact points generated. This could happen for quickly moving
  // incident boxes that penetrate deeply into large reference boxes. It is not
  // clear what is a "correct" contact set in this case, so we will add a
  // single contact point at the incident box center, to hopefully resolve the
  // penetration.
  if (contacts->empty()) {
    contacts->push_back(ContactGeometry(box2.center, sepaxis_FN,
                                        -min_depth_FN));
    if (info) {
      info->code = 16;
    }
  }
  return true;
}

bool CollideBoxes(const Eigen::Vector3d &center1,
                  const Eigen::Matrix3d &rotation1,
                  const Eigen::Vector3d &side_lengths1,
                  const Eigen::Vector3d &center2,
                  const Eigen::Matrix3d &rotation2,
                  const Eigen::Vector3d &side_lengths2,
                  CollisionInfo *info,
                  std::vector<ContactGeometry> *contacts) {
  Box box1, box2;
  box1.center = center1;
  box1.R = rotation1;
  box1.halfside = side_lengths1 * 0.5;
  box2.center = center2;
  box2.R = rotation2;
  box2.halfside = side_lengths2 * 0.5;
  return CollideBoxes(box1, box2, info, contacts);
}

bool CollideBoxAndGround(const Eigen::Vector3d &center,
                         const Eigen::Matrix3d &rotation,
                         const Eigen::Vector3d &side_lengths,
                         std::vector<ContactGeometry> *contacts) {
  bool retval = false;
  for (int x = -1; x <= 1; x += 2) {
    for (int y = -1; y <= 1; y += 2) {
      for (int z = -1; z <= 1; z += 2) {
        Vector3d v = center + rotation.col(0) * side_lengths[0] * 0.5 * x +
                              rotation.col(1) * side_lengths[1] * 0.5 * y +
                              rotation.col(2) * side_lengths[2] * 0.5 * z;
        if (v[2] < 0) {
          ContactGeometry c;
          c.position = v;
          c.normal << 0, 0, 1;
          c.depth = -v[2];
          contacts->resize(contacts->size() + 1);
          contacts->back() = c;
          retval = true;
        }
      }
    }
  }
  return retval;
}

//***************************************************************************
// Testing.

#include <stdio.h>
#include "testing.h"

namespace {

// A slow-but-sure box separation test along the given axis.
bool BoxesSeparatedByAxis(const Box &box1, const Box &box2,
                          const Vector3d &axis) {
  double span1 = box1.halfside[0] * fabs(axis.dot(box1.R.col(0))) +
                 box1.halfside[1] * fabs(axis.dot(box1.R.col(1))) +
                 box1.halfside[2] * fabs(axis.dot(box1.R.col(2)));
  double span2 = box2.halfside[0] * fabs(axis.dot(box2.R.col(0))) +
                 box2.halfside[1] * fabs(axis.dot(box2.R.col(1))) +
                 box2.halfside[2] * fabs(axis.dot(box2.R.col(2)));
  return fabs(axis.dot(box1.center) - axis.dot(box2.center)) > (span1 + span2);
}

// A slow-but-sure test to see if two boxes are separated. The 15 possible
// separating axes are the 6 box face normals and the 9 edge1 x edge2 axes,
// where edge1/edge2 is one of the three box 1 or box 2 edges.
bool BoxesSeparated(const Box &box1, const Box &box2) {
  return BoxesSeparatedByAxis(box1, box2, box1.R.col(0)) ||
         BoxesSeparatedByAxis(box1, box2, box1.R.col(1)) ||
         BoxesSeparatedByAxis(box1, box2, box1.R.col(2)) ||
         BoxesSeparatedByAxis(box1, box2, box2.R.col(0)) ||
         BoxesSeparatedByAxis(box1, box2, box2.R.col(1)) ||
         BoxesSeparatedByAxis(box1, box2, box2.R.col(2)) ||
         BoxesSeparatedByAxis(box1, box2, box1.R.col(0).cross(box2.R.col(0))) ||
         BoxesSeparatedByAxis(box1, box2, box1.R.col(0).cross(box2.R.col(1))) ||
         BoxesSeparatedByAxis(box1, box2, box1.R.col(0).cross(box2.R.col(2))) ||
         BoxesSeparatedByAxis(box1, box2, box1.R.col(1).cross(box2.R.col(0))) ||
         BoxesSeparatedByAxis(box1, box2, box1.R.col(1).cross(box2.R.col(1))) ||
         BoxesSeparatedByAxis(box1, box2, box1.R.col(1).cross(box2.R.col(2))) ||
         BoxesSeparatedByAxis(box1, box2, box1.R.col(2).cross(box2.R.col(0))) ||
         BoxesSeparatedByAxis(box1, box2, box1.R.col(2).cross(box2.R.col(1))) ||
         BoxesSeparatedByAxis(box1, box2, box1.R.col(2).cross(box2.R.col(2)));
}

// A slow-but-sure polygon / half space intersection test. The half space is
// normal.P+d>=0.

bool PolygonIntersectsHalfspace(const vector<Vector2d> &poly, const Vector2d normal, double d) {
  for (int i = 0; i < poly.size(); i++) {
    double value = normal.dot(poly[i]) + d;
    if (value > 0)
      return true;  // At least part of the polygon intersects the half space
  }
  return false;
}

// Return <0 if p is inside A, 0 if p is on a face of A, >0 if p is outside A.

double FacePseudoDistance(const Box &A, const Vector3d &p) {
  Vector3d q = A.R.transpose() * (p - A.center);
  Vector3d r = q.cwiseAbs().cwiseQuotient(A.halfside);
  return r.maxCoeff() - 1;
}

// Random initializations.

double Rand() {
  return double(random()) / double(RAND_MAX);
}

void SetRandomBox(Box *A, bool use_this_axis1, const Vector3d &axis1_to_use) {
  A->center.setRandom();
  A->center *= 0.5;
  if (use_this_axis1) {
    A->R.col(0) = axis1_to_use;
  } else {
    A->R.col(0).setRandom();
  }
  A->R.col(0).normalize();
  A->R.col(1).setRandom();
  A->R.col(1) -= (A->R.col(0).dot(A->R.col(1))) * A->R.col(0);
  A->R.col(1).normalize();
  A->R.col(2) = A->R.col(0).cross(A->R.col(1));
  CHECK(fabs(A->R.col(0).norm() - 1) < 1e-9);
  CHECK(fabs(A->R.col(1).norm() - 1) < 1e-9);
  CHECK(fabs(A->R.col(2).norm() - 1) < 1e-9);
  CHECK(fabs(A->R.col(0).dot(A->R.col(1))) < 1e-9);
  CHECK(fabs(A->R.col(0).dot(A->R.col(2))) < 1e-9);
  CHECK(fabs(A->R.col(1).dot(A->R.col(2))) < 1e-9);
  for (int i = 0; i < 3; i++) {
    A->halfside[i] = Rand();
  }
}

// Test functions.

TEST_FUNCTION(LineClosestApproach) {
  for (int i = 0; i < 1000; i++) {
    Vector3d pa, ua, pb, ub;
    pa.setRandom();
    pb.setRandom();
    ua.setRandom();
    ub.setRandom();
    ua.normalize();
    ub.normalize();
    double alpha, beta;
    LineClosestApproach(pa, ua, pb, ub, &alpha, &beta);
    Vector3d p1 = pa + alpha * ua;
    Vector3d p2 = pb + beta * ub;
    Vector3d delta = p1 - p2;
    CHECK(fabs(ua.dot(delta)) < 1e-9);
    CHECK(fabs(ub.dot(delta)) < 1e-9);
    CHECK(((pa + (alpha*1.01) * ua) - (pb + beta * ub)).norm() > delta.norm());
    CHECK(((pa + (alpha*0.99) * ua) - (pb + beta * ub)).norm() > delta.norm());
    CHECK(((pa + alpha * ua) - (pb + (beta*1.01) * ub)).norm() > delta.norm());
    CHECK(((pa + alpha * ua) - (pb + (beta*0.99) * ub)).norm() > delta.norm());
  }
}

TEST_FUNCTION(IntersectLineSegmentAndLine) {
  for (int i = 0; i < 10000; i++) {
    // Create a random line and line segment.
    Vector2d p1, p2, normal, p;
    p1.setRandom();
    p2.setRandom();
    normal.setRandom();
    double d = Rand() - 0.5;
    bool i1 = IntersectLineSegmentAndLine(p1, p2, normal, d, &p);
    printf("%d", i1);
    // The half space is normal.P+d.
    double value1 = normal.dot(p1) + d;
    double value2 = normal.dot(p2) + d;
    if (i1) {
      // We have an intersection. Make sure the returned point is on the border
      // of the half space and on a line between p1 and p2.
      CHECK((value1 > 0 && value2 < 0) || (value1 < 0 && value2 > 0));
      CHECK(fabs(normal.dot(p) + d) < 1e-9);
      CHECK(((p - p1).normalized() + (p - p2).normalized()).norm() < 1e-6);
    } else {
      // No intersection.
      CHECK((value1 >= 0 && value2 >= 0) || (value1 <= 0 && value2 <= 0));
    }
  }
  printf("\n");
}

TEST_FUNCTION(ClipPolygonByHalfSpace) {
  for (int i = 0; i < 10000; i++) {
    Vector2d normal;
    vector<Vector2d> poly, newpoly;
    double d = 0;
    if (i <= 2) {
      // Check degenerate and near-degenerate situations (half space line right
      // on an edge).
      poly.push_back(Vector2d(0, 0));
      poly.push_back(Vector2d(0, 1));
      poly.push_back(Vector2d(1, 1));
      poly.push_back(Vector2d(1, 0));
      normal << 0, 1;
      d = -(i - 1) * 1e-12;
    } else {
      // Create random triangles or axis-aligned rectangles and half spaces.
      int numpts = random() % 5 + 3;
      for (int j = 0; j < numpts; j++) {
        poly.push_back(Vector2d(Rand() - 0.5, Rand() - 0.5));
      }
      normal.setRandom();
      d = Rand()*2 - 1;
    }
    ClipPolygonByHalfSpace(poly, normal, d, &newpoly);
    bool i1 = !newpoly.empty();
    bool i2 = PolygonIntersectsHalfspace(poly, normal, d);
    printf("%d%d ", int(i1), int(i2));
    CHECK(i1 == i2);

    if (i1) {
      CHECK(newpoly.size() >= 3 && newpoly.size() <= poly.size()*2);
      // Make sure all returned points are in the half space.
      for (int i = 0; i < newpoly.size(); i++) {
        CHECK(normal.dot(newpoly[i]) + d >= -1e-12);
      }
      // Make sure there are no duplicated points.
      for (int i = 0; i < newpoly.size(); i++) {
        int j = (i + 1) % newpoly.size();
        CHECK((newpoly[i] - newpoly[j]).norm() > 1e-12);
      }
    }

    // Check the special cases instantiated above.
    if (i <= 2) {
      CHECK(newpoly.size() == 4);
      if (i < 2) {
        CHECK(newpoly == poly);
      }
    }
  }
  printf("\n");
}

TEST_FUNCTION(IntersectBoxAndRectangle) {
  for (int i = 0; i < 100000; i++) {
    // Create a random box and rectangle. Sometimes axes are shared, to check
    // for degeneracies.
    Box B, R;
    SetRandomBox(&B, false, Vector3d());
    // Occasionally make the first or second axis of R the same as the first
    // axis of B.
    SetRandomBox(&R, (i & 1) == 0, B.R.col(0));
    if ((i & 2) == 0) {
      Matrix3d R2;
      R2.col(0) = R.R.col(1);
      R2.col(1) = R.R.col(0);
      R2.col(2) = -R.R.col(2);
      R.R = R2;
    }
    if ((i & 15) == 0) {
      // Occasionally copy two random axes of B.R into R.R, set the third from
      // the cross product.
      int index1 = random() % 3;
      int index2 = random() % 2;
      if (index2 == index1) index2++;
      R.R.col(0) = B.R.col(index1);
      R.R.col(1) = B.R.col(index2);
      R.R.col(2) = R.R.col(0).cross(R.R.col(1));
    }
    R.halfside[2] = 0;          // So BoxesSeparated() treats this as a rect
    vector<Vector2d> poly;
    IntersectBoxAndRectangle(B, R, &poly);

    bool sep1 = poly.empty();
    bool sep2 = BoxesSeparated(B, R);
    printf("%d%d ", int(sep1), int(sep2));
    CHECK(sep1 == sep2);

    CHECK(poly.size() == 0 ||poly.size() >= 3);
    for (int i = 0; i < poly.size(); i++) {
      // Polygon points are in the rectangle frame, convert them to 3D.
      Vector3d q1 = R.center + R.R*Vector3d(poly[i][0], poly[i][1],0);
      // Check that all polygon points are within the box.
      Vector3d q2 = B.R.transpose() * (q1 - B.center);
      bool on_box_boundary = false;
      for (int j = 0; j < 3; j++) {
        CHECK(fabs(q2[j]) < B.halfside[j] + 1e-9);
        on_box_boundary |= (fabs(q2[j]) > B.halfside[j] - 1e-9);
      }
      // Check that all polygon points are either on the box boundary or are
      // original rectangle points.
      if (!on_box_boundary) {
        // We expect that this point is a vertex of the rectangle.
        CHECK(fabs(fabs(poly[i][0]) - R.halfside[0]) < 1e-9 &&
              fabs(fabs(poly[i][1]) - R.halfside[1]) < 1e-9);
      }
    }
  }
  printf("\n");
}

TEST_FUNCTION(CollideBoxes) {
  for (int it = 0; it < 100000; it++) {
    // Make two random boxes.
    Box box1, box2;
    box1.center = Vector3d::Random();
    box2.center = Vector3d::Random();
    box1.halfside = Vector3d::Random().cwiseAbs();
    box2.halfside = Vector3d::Random().cwiseAbs();
    Quaterniond q1(Rand()-0.5, Rand()-0.5, Rand()-0.5, Rand()-0.5);
    Quaterniond q2(Rand()-0.5, Rand()-0.5, Rand()-0.5, Rand()-0.5);
    q1.normalize();
    q2.normalize();
    box1.R = q1.matrix();
    box2.R = q2.matrix();

    // Sometimes axes are shared, to check for degeneracies.
    bool axes_aligned = false;
    if ((it % 5) == 0) {
      int i = it / 5;
      SetRandomBox(&box2, true, box1.R.col(0));
      if ((i & 3) == 1) {
        Matrix3d R2;
        R2.col(0) = box2.R.col(1);
        R2.col(1) = box2.R.col(0);
        R2.col(2) = -box2.R.col(2);
        box2.R = R2;
      } else if ((i & 3) == 2) {
        // Occasionally copy two random axes of B.R into R.R, set the third from
        // the cross product.
        int index1 = random() % 3;
        int index2 = random() % 2;
        if (index2 == index1) index2++;
        box2.R.col(0) = box1.R.col(index1);
        box2.R.col(1) = box1.R.col(index2);
        box2.R.col(2) = box2.R.col(0).cross(box2.R.col(1));
        axes_aligned = true;
      }
    }

    // Collide the boxes.
    bool sep1 = BoxesSeparated(box1, box2);
    CollisionInfo info;
    vector<ContactGeometry> contacts;
    bool sep2 = !CollideBoxes(box1, box2, &info, &contacts);

    printf("%d%d ", int(sep1), int(sep2));
    CHECK(sep1 == sep2);                        // Agreement with slow method
    CHECK(sep2 == (info.code == 0));            // info.code indicates collision
    CHECK(sep2 == (contacts.size() == 0));      // having contacts == collision

    if (!sep2) {
      // Boxes collide. Check that the 'info' separating axis is unit length.
      CHECK(fabs(info.separating_axis.norm() - 1) < 1e-9);
      // Check depth positive (to within epsilon).
      CHECK(info.depth >= -1e-9);
      // Moving the boxes apart by 99% of the penetration depth along the
      // normal should leave them colliding. Moving them 101% should separate
      // them. This test also verifies the correct direction for the normal,
      // i.e. move box1 in the opposite of the normal direction to separate the
      // boxes.
      vector<ContactGeometry> dummy_contacts;
      Box box1check(box1);
      box1check.center -= 0.99 * info.depth * info.separating_axis;
      CHECK(CollideBoxes(box1check, box2, 0, &dummy_contacts));
      for (int i = 0; i < dummy_contacts.size(); i++) {
        // Make sure that contact normals point in (more or less) the same
        // direction as the separating_axis in this case
        CHECK(dummy_contacts[i].normal.dot(info.separating_axis) > 0);
      }
      box1check.center -= 0.02 * info.depth * info.separating_axis;
      CHECK(!CollideBoxes(box1check, box2, 0, &dummy_contacts));

      // Check the contact points that come back. A general test for contact
      // point correctness is tough because the best set is not well defined,
      // so we check the general features of the particular algorithm we use.
      for (int i = 0; i < contacts.size(); i++) {
        CHECK(contacts[i].depth >= -1e-9);
        CHECK(fabs(contacts[i].normal.norm() - 1) < 1e-9);
      }

      // Code-dependent checking of contacts.
      if (info.code >= 1 && info.code <= 3) {
        // Expecting one or more face-something contacts that lie on a face of
        // box 2. Projecting the contact position along the normal by the depth
        // should bring it to the surface of box1.
        for (int i = 0; i < contacts.size(); i++) {
          CHECK(fabs(FacePseudoDistance(box2, contacts[i].position)) < 1e-9);
          Vector3d q = contacts[i].position + contacts[i].normal *
                       contacts[i].depth;
          CHECK(fabs(FacePseudoDistance(box1, q)) < 1e-9);
        }
      } else if (info.code >= 4 && info.code <= 6) {
        // Expecting one or more face-something contacts that lie on a face of
        // box 1. Projecting the contact position along the normal by the depth
        // should bring it to the surface of box2.
        for (int i = 0; i < contacts.size(); i++) {
          CHECK(fabs(FacePseudoDistance(box1, contacts[i].position)) < 1e-9);
          Vector3d q = contacts[i].position - contacts[i].normal *
                       contacts[i].depth;
          CHECK(fabs(FacePseudoDistance(box2, q)) < 1e-9);
        }
      } else if (info.code >= 7 && info.code <= 15) {
        // Expecting a single contact between two edges.
        CHECK(contacts.size() == 1);
        // The contact normal is the separating axis in this case.
        CHECK(contacts[0].normal == info.separating_axis);
      } else if (info.code == 16) {
        // Expecting a single contact at box2's center.
        CHECK(contacts.size() == 1);
        CHECK(contacts[0].position == box2.center);
      } else {
        CHECK(info.code == 0);
      }

      if (axes_aligned && info.code >= 1 && info.code <= 6) {
        CHECK(contacts.size() == 4);    // Should be a 'contact rectangle'
      }
    }
  }

  printf("\n");
}

}  // anonymous namespace
