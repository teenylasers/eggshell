
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
