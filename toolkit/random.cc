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
// Windows, rand() and std::rand() is available everywhere but generate low
// quality random numbers on some platforms, the Eigen Random() is hard to
// remember the syntax for (and actually maps to std::rand() under the hood),
// and the C++11 random number generators are kind of complicated. Here we
// provide an easy-to-remember interface, that is based on the C++11 features.

#include "random.h"
#include "testing.h"
#include "error.h"
#include "thread.h"
#include <random>

static std::mutex mutex;           // Protects the generator
static std::mt19937_64 generator;  // Random number generator

static struct NondeterministicSeeder {
  NondeterministicSeeder() {
    MutexLock lock(&mutex);
    std::random_device rd;
    generator.seed(rd());
  }
} seeder;

void RandomSeed(int seed) {
  MutexLock lock(&mutex);
  generator.seed(seed);
}

double RandomDouble() {
  MutexLock lock(&mutex);
  std::uniform_real_distribution<> d(0, 1);
  return d(generator);
}

int RandomInt(int n) {
  MutexLock lock(&mutex);
  std::uniform_int_distribution<> d(0, n - 1);
  return d(generator);
}

TEST_FUNCTION(RandomDouble) {
  // Check the random is correct.
  double min = __DBL_MAX__;
  double max = -__DBL_MAX__;
  for (int i = 0; i < 10000; i++) {
    double r = RandomDouble();
    min = std::min(min, r);
    max = std::max(max, r);
  }
  printf("Random number range = %f ... %f\n", min, max);
  CHECK(min >= 0 && min <= 0.01);
  CHECK(max >= 0.99 && max < 1);

  // Check repeatability with a constant seed.
  RandomSeed(123);
  const int N = 1000;
  double v[N];
  for (int i = 0; i < N; i++) {
    v[i] = RandomDouble();
  }
  RandomSeed(123);
  for (int i = 0; i < N; i++) {
    CHECK(v[i] == RandomDouble());
  }
}

TEST_FUNCTION(RandomInt) {
  // Check the random is correct.
  int min = 1000000;
  int max = -1000000;
  bool got99 = false;
  for (int i = 0; i < 10000; i++) {
    int r = RandomInt(100);
    min = std::min(min, r);
    max = std::max(max, r);
    got99 = got99 || (r == 99);
  }
  printf("Random number range = %d ... %d\n", min, max);
  CHECK(got99);
  CHECK(min >= 0 && min <= 5);
  CHECK(max >= 95 && max < 100);

  // Check repeatability with a constant seed.
  RandomSeed(123);
  const int N = 1000;
  int v[N];
  for (int i = 0; i < N; i++) {
    v[i] = RandomInt(1000);
  }
  RandomSeed(123);
  for (int i = 0; i < N; i++) {
    CHECK(v[i] == RandomInt(1000));
  }
}
