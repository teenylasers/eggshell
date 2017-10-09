
#ifndef __TOOLKIT_COLLISION_H__
#define __TOOLKIT_COLLISION_H__

#include "myvector"
#include <set>

namespace collision {

// An axis aligned bounding box of D dimensions.
template<int D> struct AABB {
  double min[D], max[D];

  bool Overlaps(const AABB &a) {
    for (int i = 0; i < D; i++) {
      if (min[i] > a.max[i] || max[i] < a.min[i]) {
        return false;
      }
    }
    return true;
  }
};

// Use the sweep-and-prune algorithm to find the indexes of all pairs of boxes
// in aabb that overlap. Return them as a set of pairs, with first < second in
// the pair.
template<int D> void SweepAndPrune(const std::vector<AABB<D> > &aabb,
                                   std::set<std::pair<int, int> > *overlaps);

}  // namespace collision

#endif
