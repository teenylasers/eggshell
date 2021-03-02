// Copyright (C) 2014-2020 Russell Smith.
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

#ifndef __TOOLKIT_COLLISION_H__
#define __TOOLKIT_COLLISION_H__

#include <vector>
#include <set>

namespace collision {

// An axis aligned bounding box of D dimensions.
template<int D> struct AABB {
  double min[D], max[D];

  // Return true if two AABBs overlap, or are only just touching with an
  // overlap distance of zero.
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
