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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "engine.h"     // Matlab include
#include "matrix.h"     // Matlab include
#include "matlab_engine.h"
#include "error.h"

namespace MEngine {

static Engine *engine = 0;

static void Start() {
  if (!engine) {
    engine = engOpen(NULL);
    CHECK(engine);
    engSetVisible(engine, 1);
  }
}

void Eval(const char *command_line) {
  Start();
  engEvalString(engine, command_line);
}

void PutVector(const char *name, int n, const double *data) {
  Start();
  mxArray *m = mxCreateDoubleMatrix(n, 1, mxREAL);
  double *p = mxGetPr(m);
  for (int i = 0; i < n; i++) {
    p[i] = data[i];
  }
  engPutVariable(engine, name, m);
  mxDestroyArray(m);
}

void PutVector(const char *name, int n, const std::complex<double> *data) {
  Start();
  mxArray *m = mxCreateDoubleMatrix(n, 1, mxCOMPLEX);
  double *pr = mxGetPr(m);
  double *pi = mxGetPi(m);
  for (int i = 0; i < n; i++) {
    pr[i] = data[i].real();
    pi[i] = data[i].imag();
  }
  engPutVariable(engine, name, m);
  mxDestroyArray(m);
}

void PutMatrix(const char *name, const Eigen::MatrixXd &A) {
  Start();
  mxArray *m = mxCreateDoubleMatrix(A.rows(), A.cols(), mxREAL);
  double *p = mxGetPr(m);
  const double *data = A.data();
  int n = A.rows() * A.cols();
  for (int i = 0; i < n; i++) {
    p[i] = data[i];
  }
  engPutVariable(engine, name, m);
  mxDestroyArray(m);
}

void PutMatrix(const char *name,
               const Eigen::SparseMatrix<std::complex<double> > &A) {
  CHECK(A.isCompressed());
  Start();
  int nnz = A.nonZeros();
  const std::complex<double> *data = A.valuePtr();
  mxArray *m = mxCreateSparse(A.rows(), A.cols(), nnz, mxCOMPLEX);
  double *pr = mxGetPr(m);
  for (int i = 0; i < nnz; i++) {
    pr[i] = data[i].real();
  }
  double *pi = mxGetPi(m);
  for (int i = 0; i < nnz; i++) {
    pi[i] = data[i].imag();
  }
  const int *eir = A.innerIndexPtr();   // Eigen column indexes
  const int *ejc = A.outerIndexPtr();   // Eigen row offsets
  size_t *mir = mxGetIr(m);            // Matlab column indexes
  size_t *mjc = mxGetJc(m);            // Matlab row offsets
  for (int i = 0; i < nnz; i++) {
    mir[i] = eir[i];
  }
  for (int i = 0; i < A.cols(); i++) {
    mjc[i] = ejc[i];
  }
  mjc[A.cols()] = nnz;
  engPutVariable(engine, name, m);
  mxDestroyArray(m);
}

}  // namespace MEngine
