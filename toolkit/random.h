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

// Random numbers. Unfortunately the GNU C library random() is not available on
// Windows, rand() and std::rand() is available everywhere but generates low
// quality random numbers on some platforms, and the Eigen Random() is hard to
// remember the syntax for (and actually maps to std::rand() under the hood).
// Here we provide an easy-to-remember interface, that is based on the C++11
// random number implementation.

#ifndef __TOOLKIT_RANDOM_H__
#define __TOOLKIT_RANDOM_H__

#include <random>

// Return a random double from 0 to 1 (if no argument is given) or seed the
// random number generator (if an integer >= 0 is given).
inline double Random(int seed = -1) {
  static std::mt19937_64 g;
  static std::uniform_real_distribution<> d(0, 1);
  if (seed >= 0) {
    g.seed(seed);
    return 0;
  } else {
    return d(g);
  }
}

// Return a random integer from 0 to n-1 (if no argument is given) or seed the
// random number generator (if an integer >= 0 is given).
inline int RandomInt(int n, int seed = -1) {
  static std::mt19937_64 g;
  std::uniform_int_distribution<> d(0, n - 1);
  if (seed >= 0) {
    g.seed(seed);
    return 0;
  } else {
    return d(g);
  }
}

#endif
