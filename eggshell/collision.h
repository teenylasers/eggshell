
#ifndef __COLLISION_H__
#define __COLLISION_H__

#include <vector>
#include <Eigen/Dense>

// A contact between an ordered pair of objects, A and B. The normal always
// points 'out' from object A. The depth is the distance that object B should
// be displaced along the normal to reduce the penetration to zero. The normal
// may or may not be the same as info.separating_axis.
struct ContactGeometry {
  Eigen::Vector3d position;     // Global coordinates of the contact point
  Eigen::Vector3d normal;       // Unit length contact normal
  double depth;                 // Penetration depth of the contact

  ContactGeometry() {
    position.setZero();
    normal.setZero();
    depth = 0;
  }
  ContactGeometry(const Eigen::Vector3d &_position,
                  const Eigen::Vector3d &_normal,
                  double _depth)
      : position(_position), normal(_normal), depth(_depth) {
  }
};

// General information about the intersection between an ordered pair of
// objects, A and B. The separating_axis always points 'out' from object A. The
// depth is the distance that object B should be displaced along the
// separating_axis to reduce the penetration to zero.
struct CollisionInfo {
  double depth;                         // Max penetration depth along sep axis
  Eigen::Vector3d separating_axis;      // Unit length separating axis
  // Classification of box-box collisions: 0 = no collision, 1-3 = normal is
  // face 1..3 of box A, 4-6 = normal is face 1..3 of box B, 7-15 = normal is a
  // cross product of an edge from each box, 16 = no good normal (e.g. too much
  // penetration) so heuristic contacts used.
  int code;

  CollisionInfo() {
    depth = 0;
    separating_axis.setZero();
    code = 0;
  }
};

// Collide two boxes with each other and add contact points to the 'contacts'
// vector as necessary. Return collision information in 'info' if that is not
// null. Return true if the boxes intersect (and contacts were added) or false
// otherwise. Each box has vertices: center +/- halfside(0)*R.col(0)
// +/- halfside(1)*R.col(1) +/- halfside(2)*R.col(2).
bool CollideBoxes(const Eigen::Vector3d &center1,
                  const Eigen::Matrix3d &rotation1,
                  const Eigen::Vector3d &side_lengths1,
                  const Eigen::Vector3d &center2,
                  const Eigen::Matrix3d &rotation2,
                  const Eigen::Vector3d &side_lengths2,
                  CollisionInfo *info,
                  std::vector<ContactGeometry> *contacts);

// Collide a box with the ground and add contact points to the 'contacts'
// vector as necessary. Return true if contacts were added or false otherwise.
bool CollideBoxAndGround(const Eigen::Vector3d &center,
                         const Eigen::Matrix3d &rotation,
                         const Eigen::Vector3d &side_lengths,
                         std::vector<ContactGeometry> *contacts);

#endif
