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

#include "collision.h"
#include "error.h"
#include "testing.h"
#include "random.h"
#include <algorithm>

using std::vector;
using std::set;
using std::sort;
using std::pair;
using std::make_pair;

namespace collision {

template<class T> inline pair<T, T> make_sorted_pair(T a, T b) {
  if (a <= b) {
    return pair<T, T>(a, b);
  } else {
    return pair<T, T>(b, a);
  }
}

// A symmetric 2D matrix of boolean, indexed by AABB box IDs.
class IDPairSet {
 public:
  explicit IDPairSet(int num_ids) : n_(num_ids), s_(num_ids * num_ids) {}
  void Add(int a, int b) {
    CHECK(a >= 0 && a < n_ && b >= 0 && b < n_);
    s_[a*n_ + b] = true;
    s_[b*n_ + a] = true;
  }
  bool Contains(int a, int b) {
    CHECK(a >= 0 && a < n_ && b >= 0 && b < n_);
    return s_[a*n_ + b];
  }
  void swap(IDPairSet &q) {
    CHECK(n_ == q.n_);
    s_.swap(q.s_);
  }
 private:
  int n_;               // ID range is 0..n_-1
  vector<bool> s_;      // n_*n_ boolean matrix
};

template<int D>
void SweepAndPrune(const vector<AABB<D> > &aabb,
                   set<pair<int, int> > *overlaps) {
  IDPairSet last_candidates(aabb.size());
  for (int d = 0; d < D; d++) {
    // For each dimension create a sorted list of the starts and ends of box
    // edges.
    struct Edge {
      double coord;             // coordinate
      bool start;               // true if a box start, false if a box end
      int id;                   // index into aabb
      bool operator<(const Edge &e) const {
        return coord < e.coord || (coord == e.coord && start > e.start);
      }
    };
    vector<Edge> e(aabb.size() * 2);
    for (int i = 0; i < aabb.size(); i++) {
      e[i*2 + 0].coord = aabb[i].min[d];
      e[i*2 + 0].start = true;
      e[i*2 + 0].id = i;
      e[i*2 + 1].coord = aabb[i].max[d];
      e[i*2 + 1].start = false;
      e[i*2 + 1].id = i;
    }
    sort(e.begin(), e.end());

    // For all overlapping intervals set a pair of IDs in 'candidates'.
    IDPairSet candidates(d < D - 1 ? aabb.size() : 0);
    set<int> active_ids;                        // Started but not ended IDs
    for (int i = 0; i < e.size(); i++) {
      if (e[i].start) {
        int this_id = e[i].id;
        // This ID and all existing active pairs are candidates for overlap,
        // but only if they were candidates on the last iteration too.
        for (auto id : active_ids) {
          if (d == 0 || last_candidates.Contains(this_id, id)) {
            if (d == D - 1) {
              overlaps->insert(make_sorted_pair(this_id, id));
            } else {
              candidates.Add(this_id, id);
            }
          }
        }
        active_ids.insert(this_id);
      } else {
        active_ids.erase(e[i].id);
      }
    }
    if (d < D - 1) {
      candidates.swap(last_candidates);
    }
  }
}

}  // namespace collision

//***************************************************************************
// Testing

#include <stdio.h>
#include <stdlib.h>

template<int D> void SweepAndPruneTester() {
  const int n = 1000;
  vector<collision::AABB<D> > aabb(n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < D; j++) {
      aabb[i].min[j] = Random();
      aabb[i].max[j] = aabb[i].min[j] + 0.1*Random();
    }
  }

  set<pair<int, int> > overlaps;
  collision::SweepAndPrune(aabb, &overlaps);

  int count = 0;
  for (int i = 0; i < n; i++) {
    for (int j = i + 1; j < n; j++) {
      int o = overlaps.count(make_pair(i, j));
      CHECK(o == aabb[i].Overlaps(aabb[j]));
      if (o) {
        count++;
      }
    }
  }
  printf("Checked %d overlaps for %dD\n", count, D);
}

TEST_FUNCTION(SweepAndPrune_1D) {
  SweepAndPruneTester<1>();
}

TEST_FUNCTION(SweepAndPrune_2D) {
  SweepAndPruneTester<2>();
}

TEST_FUNCTION(SweepAndPrune_3D) {
  SweepAndPruneTester<3>();
}
