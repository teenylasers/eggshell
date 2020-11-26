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

// Random numbers. Unfortunately the GNU C library random() is not available
// on Windows, rand() is available everywhere but generates low quality random
// numbers on some platforms, and the Eigen Random() is hard to remember the
// syntax for. Here we provide an easy-to-remember interface.

#ifndef __TOOLKIT_RANDOM_H__
#define __TOOLKIT_RANDOM_H__

#include "Eigen/Dense"

// Return a random double from 0 to 1.
inline double Random() {
  return 0.5 + 0.5*Eigen::Matrix<double,1,1>::Random()[0];
}

#endif
