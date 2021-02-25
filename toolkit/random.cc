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

#include "random.h"
#include "testing.h"
#include "error.h"

TEST_FUNCTION(Random) {
  // Check the random is correct.
  double min = __DBL_MAX__;
  double max = -__DBL_MAX__;
  for (int i = 0; i < 10000; i++) {
    double r = Random();
    min = std::min(min, r);
    max = std::max(max, r);
  }
  printf("Random number range = %f ... %f\n", min, max);
  CHECK(min >= 0 && min <= 0.01);
  CHECK(max >= 0.99 && max < 1);

  // Check repeatability with a constant seed.
  Random(123);
  const int N = 1000;
  double v[N];
  for (int i = 0; i < N; i++) {
    v[i] = Random();
  }
  Random(123);
  for (int i = 0; i < N; i++) {
    CHECK(v[i] == Random());
  }
}

TEST_FUNCTION(RandomInt) {
  // Check the random is correct.
  int min = 1000000;
  int max = -1000000;
  for (int i = 0; i < 10000; i++) {
    int r = RandomInt(100);
    min = std::min(min, r);
    max = std::max(max, r);
  }
  printf("Random number range = %d ... %d\n", min, max);
  CHECK(min >= 0 && min <= 5);
  CHECK(max >= 95 && max < 100);

  // Check repeatability with a constant seed.
  RandomInt(0, 123);
  const int N = 1000;
  int v[N];
  for (int i = 0; i < N; i++) {
    v[i] = RandomInt(1000);
  }
  RandomInt(0, 123);
  for (int i = 0; i < N; i++) {
    CHECK(v[i] == RandomInt(1000));
  }
}
