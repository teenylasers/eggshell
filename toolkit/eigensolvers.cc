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
#include <vector>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "eigensolvers.h"
#include "error.h"
#include "mat_file.h"
#include "testing.h"
#include "random.h"

using namespace Eigen;
using std::vector;

typedef SparseMatrix<double> SMatrix;

// Compute the M smallest eigenvalues and eigenvectors of A*x=lambda*B*x, using
// shift-and-invert mode with sigma shift. Return the eigenvalues.
VectorXd TestEigenSolver(const SMatrix &A, const SMatrix *B, double sigma,
                         int M) {
  printf("Now running the eigen solver\n");
  eigensolvers::LaplacianEigenSolver eigensolver(A, B, M, sigma);
  printf("num converged = %d\n", eigensolver.GetNumConvergedEigenvalues());
  printf("iterations = %d\n", eigensolver.GetNumIterations());
  CHECK(eigensolver.Status() == Success);
  printf("Done\n");

  // Check the number of returned eigenvalues.
  const VectorXd &val = eigensolver.GetEigenValues();
  CHECK(val.size() == M);
  for (int i = 0; i < val.size(); i++) {
    printf("Eigenvalue %d = %f\n", i, val[i]);
  }

  // Check the eigenvectors.
  const MatrixXd &vec = eigensolver.GetEigenVectors();
  printf("Eigenvector matrix = %d x %d\n", vec.rows(), vec.cols());
  CHECK(vec.rows() == A.cols());
  CHECK(vec.cols() == M);
  for (int i = 0; i < M; i++) {
    double error = 1e9;
    if (B) {
      error = (A * vec.col(i) - val[i] * (*B) * vec.col(i)).norm();
    } else {
      error = (A * vec.col(i) - val[i] * vec.col(i)).norm();
    }
    CHECK(error < 1e-9);
  }

  return eigensolver.GetEigenValues();
}

TEST_FUNCTION(LaplacianEigenSolver) {
  Random(123);

  // Create a laplacian sparse matrix for an N*N 2D grid. This will have
  // dirichlet boundary conditions and therefore be nonsingular.
  const int N = 100;
  vector<Triplet<double> > Ad_trips;
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      int index = i*N + j;
      Ad_trips.push_back(Triplet<double>(index, index, 4));
      if (i > 0) Ad_trips.push_back(Triplet<double>(index, index-N, -1));
      if (i < N-1) Ad_trips.push_back(Triplet<double>(index, index+N, -1));
      if (j > 0) Ad_trips.push_back(Triplet<double>(index, index-1, -1));
      if (j < N-1) Ad_trips.push_back(Triplet<double>(index, index+1, -1));
    }
  }
  SMatrix Ad(N*N, N*N);
  Ad.setFromTriplets(Ad_trips.begin(), Ad_trips.end());

  // Create a laplacian sparse matrix for an N*N 2D grid. This will have
  // neumann boundary conditions and therefore be singular.
  vector<Triplet<double> > An_trips;
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      int index = i*N + j;
      int sz = An_trips.size();
      if (i > 0) An_trips.push_back(Triplet<double>(index, index-N, -1));
      if (i < N-1) An_trips.push_back(Triplet<double>(index, index+N, -1));
      if (j > 0) An_trips.push_back(Triplet<double>(index, index-1, -1));
      if (j < N-1) An_trips.push_back(Triplet<double>(index, index+1, -1));
      An_trips.push_back(Triplet<double>(index, index, An_trips.size() - sz));
    }
  }
  SMatrix An(N*N, N*N);
  An.setFromTriplets(An_trips.begin(), An_trips.end());

  // Create a random symmetric tridiagonal matrix B that is strictly diagonally
  // dominant and therefore positive definite.
  vector<Triplet<double> > Btrips;
  for (int i = 0; i < N*N; i++) {
    Btrips.push_back(Triplet<double>(i, i, 0.015));
    if (i > 0) Btrips.push_back(Triplet<double>(i, i-1, 0.01 * Random()));
  }
  SMatrix B(N*N, N*N);
  B.setFromTriplets(Btrips.begin(), Btrips.end());
  B = B + SMatrix(B.transpose());

  // Write matrices to a file for later comparison with matlab.
  if (false) {
    MatFile matfile("/tmp/arpacktest.mat");
    matfile.WriteSparseMatrix("Ad", Ad);
    matfile.WriteSparseMatrix("An", An);
    matfile.WriteSparseMatrix("B", B);
  }

  // Test the eigen problem: Ad*x=lambda*x
  const int M = 5;
  VectorXd val = TestEigenSolver(Ad, 0, 0, M);
  CHECK(fabs(val[0] - 0.001935) < 1e-6);
  CHECK(fabs(val[1] - 0.004836) < 1e-6);
  CHECK(fabs(val[2] - 0.004836) < 1e-6);
  CHECK(fabs(val[3] - 0.007738) < 1e-6);
  CHECK(fabs(val[4] - 0.009669) < 1e-6);

  // Test the eigen problem: Ad*x=lambda*B*x
  // Compare with matlab: eigs(Ad,B,5,'smallestabs')
  val = TestEigenSolver(Ad, &B, 0, M);
  CHECK(fabs(val[0] - 0.048375157555676) < 1e-6);
  CHECK(fabs(val[1] - 0.121018479281537) < 1e-6);
  CHECK(fabs(val[2] - 0.121202245787936) < 1e-6);
  CHECK(fabs(val[3] - 0.194301702140338) < 1e-6);
  CHECK(fabs(val[4] - 0.241711622046461) < 1e-6);

  // Test the eigen problem: An*x=lambda*x
  val = TestEigenSolver(An, 0, 0.0004, M);
  CHECK(fabs(val[0]) < 1e-6);
  CHECK(fabs(val[1] - 0.000987) < 1e-6);
  CHECK(fabs(val[2] - 0.000987) < 1e-6);
  CHECK(fabs(val[3] - 0.001974) < 1e-6);
  CHECK(fabs(val[4] - 0.003947) < 1e-6);

  // Test the eigen problem: An*x=lambda*B*x
  // Compare with matlab: eigs(An,B,5,'smallestabs')
  val = TestEigenSolver(An, &B, 0.01, M);
  CHECK(fabs(val[0]) < 1e-6);
  CHECK(fabs(val[1] - 0.0246476654125647) < 1e-6);
  CHECK(fabs(val[2] - 0.0249103281320441) < 1e-6);
  CHECK(fabs(val[3] - 0.0497573731273677) < 1e-6);
  CHECK(fabs(val[4] - 0.0984064949089944) < 1e-6);
}
