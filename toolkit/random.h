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
// random number implementation. All these functions are thread safe.

#ifndef __TOOLKIT_RANDOM_H__
#define __TOOLKIT_RANDOM_H__

// Seed the random number generator. The default seed is nondeterministic.
void RandomSeed(int seed);

// Return a random double from 0 to 1.
double RandomDouble();

// Return a random integer from 0 to n-1.
int RandomInt(int n);

#endif
