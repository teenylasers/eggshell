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

#ifndef __TOOLKIT_MATLAB_ENGINE_H__
#define __TOOLKIT_MATLAB_ENGINE_H__

#include "Eigen/Sparse"

namespace MEngine {

  void Eval(const char *command_line);
  void PutVector(const char *name, int n, const double *data);
  void PutVector(const char *name, int n, const std::complex<double> *data);
  void PutMatrix(const char *name, const Eigen::MatrixXd &A);
  void PutMatrix(const char *name,
                 const Eigen::SparseMatrix<std::complex<double> > &A);

  template <class T>
  void PutScalar(const char *name, T data) {
    PutVector(name, 1, &data);
  }

  // The templated functions below work with both std::vector and Eigen
  // vectors.

  template <class T>
  inline void PutVector(const char *name, const T &a) {
    PutVector(name, a.size(), a.data());
  }

  template <class T>
  inline void Plot(const T &ydata) {
    PutVector("y_", ydata);
    Eval("plot(y_)");
  }

  template <class T>
  inline void Plot(const T &xdata, const T &ydata) {
    PutVector("x_", xdata);
    PutVector("y_", ydata);
    Eval("plot(x_,y_)");
  }

}

#endif
