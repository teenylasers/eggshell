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

// Testing for FEMSolver.

#include "femsolver.h"
#include <stdio.h>
#include "testing.h"

namespace FEM {

TEST_FUNCTION(VerifyExampleFEMProblem) {
  FEMSolver<ExampleFEMProblem> solver;
  const char *error = solver.Verify();
  if (error) {
    Panic("%s", error);
  }
  for (int i = 0; i < solver.NumTriangles(); i++) {
    printf("Triangle %d has points", i);
    for (int j = 0; j < 3; j++) {
      printf(" %d", solver.Triangle(i, j));
    }
    printf(" and edge types");
    for (int j = 0; j < 3; j++) {
      printf(" %d", solver.EdgeType(i, j));
    }
    printf("\n");
  }
}

TEST_FUNCTION(CreateIndexMaps) {
  FEMSolver<ExampleFEMProblem> solver;
  solver.CreateIndexMaps();
  for (int i = 0; i < solver.index_map.size(); i++) {
    printf("index_map[%d] = %d\n", i, solver.index_map[i]);
    CHECK(solver.index_map[i] >= -1 &&
          solver.index_map[i] < int(solver.reverse_index_map.size()));
    if (solver.index_map[i] >= 0) {
      CHECK(solver.reverse_index_map[solver.index_map[i]] == i);
    }
  }
  for (int i = 0; i < solver.reverse_index_map.size(); i++) {
    printf("reverse_index_map[%d] = %d\n", i, solver.reverse_index_map[i]);
    CHECK(solver.reverse_index_map[i] >= 0 &&
          solver.reverse_index_map[i] < int(solver.index_map.size()));
    CHECK(solver.index_map[solver.reverse_index_map[i]] == i);
  }
}

TEST_FUNCTION(PadSolution) {
  FEMSolver<ExampleFEMProblem> solver;
  solver.CreateIndexMaps();
  CHECK(solver.reverse_index_map.size() < solver.index_map.size());
  ExampleFEMProblem::MNumberVector solution;
  solution.resize(solver.reverse_index_map.size());
  for (int i = 0; i < solver.reverse_index_map.size(); i++) {
    solution[i] = i + 10;
  }
  solver.PadSolution(&solution);
  CHECK(solution.size() == solver.index_map.size());
  int count = 10;
  for (int i = 0; i < solver.index_map.size(); i++) {
    if (solver.index_map[i] == -1) {
      CHECK(solution[i] == 0);
    } else {
      CHECK(solution[i] == count);
      count++;
    }
  }
}

TEST_FUNCTION(CreateAndSolveSystem) {
  FEMSolver<ExampleFEMProblem> solver;
  solver.CreateIndexMaps();
  solver.CreateSystem();
  bool status = solver.SolveSystem();
  CHECK(status);
  CHECK(solver.solution.size() == solver.NumPoints());

  // Check the solution u against the Galerkin equation:
  //
  //   + Integral_B[b Grad[u] . normal]        (call this term1)
  //   - Integral_E[Grad[b] . Grad[u]]         (call this term2)
  //   + Integral_E[b g u]                     (call this term3)
  //   - Integral_E[b f]                       (call this term4)
  //   == 0
  //
  // where b is each basis function, Integral_E is the integral over the
  // element, Integral_B is an integral over the border of the element.

  // Calculate each of the above terms for each triangle. These were all
  // calculated by hand in mathematica.
  const int nt = solver.NumTriangles();
  double term1[nt][3];          // term[i][j] means the terms for triangle i
  double term2[nt][3];          // for the b function that is 1 at index j of
  double term3[nt][3];          // the triangle and 0 elsewhere.
  double term4[nt][3];
  for (int i = 0; i < nt; i++) {
    double u[3], g[3], f[3];
    for (int j = 0; j < 3; j++) {
      u[j] = solver.solution[solver.Triangle(i, j)];
      g[j] = solver.PointG(i, j).value;
      f[j] = solver.PointF(i, j).value;
    }
    term1[i][0] = term1[i][1] = term1[i][2] = 0;
    if (i==25 || i==27 || i==29 || i==31) {
      // Horizontal edge Robin boundary. Alpha is constant over the edge, beta
      // is linearly interpolated.
      ExampleFEMProblem::Number alpha0, alpha1, beta0, beta1;
      solver.Robin(i, 0, 0, solver.PointG(i, 0), &alpha0, &beta0);
      solver.Robin(i, 0, 1, solver.PointG(i, 1), &alpha1, &beta1);
      term1[i][0] += ((-u[1])*(alpha0.value + alpha1.value) -
                      u[0]*(3*alpha0.value + alpha1.value) +
                      2*(2*beta0.value + beta1.value))/12.0;
      term1[i][1] += ((-u[0])*(alpha0.value + alpha1.value) -
                      u[1]*(alpha0.value + 3*alpha1.value) +
                      2*(beta0.value + 2*beta1.value))/12.0;
      term1[i][2] += 0;
    }
    if (i==7 || i==15 || i==23 || i==31) {
      // Vertical Robin boundary. Alpha is constant over the edge, beta is
      // linearly interpolated.
      ExampleFEMProblem::Number alpha1, alpha2, beta1, beta2;
      solver.Robin(i, 1, 1, solver.PointG(i, 1), &alpha1, &beta1);
      solver.Robin(i, 1, 2, solver.PointG(i, 2), &alpha2, &beta2);
      term1[i][0] += 0;
      term1[i][1] += ((-u[2])*(alpha1.value + alpha2.value) -
                     u[1]*(3*alpha1.value + alpha2.value) +
                     2*(2*beta1.value + beta2.value))/12.0;
      term1[i][2] += ((-u[1])*(alpha1.value + alpha2.value) -
                     u[2]*(alpha1.value + 3*alpha2.value) +
                     2*(beta1.value + 2*beta2.value))/12.0;
    }
    double s = (i & 1);                 // Triangle type (0=down, 1=up)
    term2[i][0] = s ? (u[0] - u[1])/2 : u[0] - (u[1] + u[2])/2;
    term2[i][1] = s ? u[1] - (u[0] + u[2])/2 : (u[1] - u[0])/2;
    term2[i][2] = s ? (u[2] - u[1])/2 : (u[2] - u[0])/2;
    term3[i][0] = (2*g[0]*(3*u[0] + u[1] + u[2]) +
        g[1]*(2*(u[0] + u[1]) + u[2]) + g[2]*(2*u[0] + u[1] + 2*u[2]))/120.0;
    term3[i][1] = (2*g[1]*(u[0] + 3*u[1] + u[2]) +
        g[0]*(2*(u[0] + u[1]) + u[2]) + g[2]*(u[0] + 2*(u[1] + u[2])))/120.0;
    term3[i][2] = (g[0]*(2*u[0] + u[1] + 2*u[2]) +
        2*g[2]*(u[0] + u[1] + 3*u[2]) + g[1]*(u[0] + 2*(u[1] + u[2])))/120.0;
    term4[i][0] = (2*f[0] + f[1] + f[2]) / 24.0;
    term4[i][1] = (f[0] + 2*f[1] + f[2]) / 24.0;
    term4[i][2] = (f[0] + f[1] + 2*f[2]) / 24.0;
  }

  // Identify all Dirichlet points.
  vector<bool> dpoints(solver.NumPoints());
  for (int i = 0; i < nt; i++) {                // For each triangle
    for (int j = 0; j < 3; j++) {               // For each point
      int pt = solver.Triangle(i, j);
      if (solver.EdgeType(i, j) == solver.DIRICHLET ||
          solver.EdgeType(i, (j + 2) % 3) == solver.DIRICHLET) {
        dpoints[pt] = true;
      }
    }
  }

  // For each point compute the sum of all the terms for that basis function.
  // The strategy: for each point on each triangle, that triangle is part of
  // the basis function for that point, so add in the appropriate terms (not
  // for Dirichlet points though).
  vector<double> termsum(solver.NumPoints());
  for (int i = 0; i < nt; i++) {                // For each triangle
    for (int j = 0; j < 3; j++) {               // For each point
      int pt = solver.Triangle(i, j);
      if (!dpoints[pt]) {
        termsum[pt] += term1[i][j];
        termsum[pt] -= term2[i][j];
        termsum[pt] += term3[i][j];
        termsum[pt] -= term4[i][j];
      }
    }
  }

  // Check that all terms sum to zero for all basis functions.
  for (int i = 0; i < solver.NumPoints(); i++) {
    printf("Point %2d, u=%10f, termsum=%e\n", i, solver.solution[i],
           termsum[i]);
  }
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 5; x++) {
      printf(" %s", fabs(termsum[y*5+x]) > 1e-9 ? "!" : ".");
    }
    printf("\n");
  }
  for (int i = 0; i < termsum.size(); i++) {
    CHECK(fabs(termsum[i]) <= 1e-9);
  }
}

TEST_FUNCTION(RobinBoundary) {
  // Check the sign of beta. The boundary constraint below is du/dnormal=1,
  // with normal pointing out of the boundary. This should ensure the entire
  // solution is positive.
  FEMSolver<ExampleFEMProblem> solver;
  for (int i = 0; i < solver.NumTriangles() * 3; i++) {
    solver.test_f[i] = 0;
    solver.test_g[i] = 0;
    solver.test_a[i] = 0;
    solver.test_b[i] = 1;
  }
  CHECK(solver.SolveSystem());
  CHECK(solver.solution.size() == solver.NumPoints());
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      printf("%10.4f", solver.solution[i*5+j]);
      CHECK(solver.solution[i*5+j] >= 0);
    }
    printf("\n");
  }
}

TEST_FUNCTION(ComputeSolutionDerivative) {
  FEMSolver<ExampleFEMProblem> solver;
  CHECK(solver.SolveSystem());

  // Set random derivatives in triplets and the rhs.
  vector<ExampleFEMProblem::Triplet> new_triplets;
  CHECK(solver.solution.size() == 25);
  const int m = solver.SystemSize();
  CHECK(m == 16);
  Eigen::MatrixXd dAdp(m, m);
  Eigen::VectorXd dbdp(m);
  dAdp.setZero();
  dbdp.setZero();
  for (int i = 0; i < solver.triplets.size(); i++) {
    ExampleFEMProblem::Number value = solver.triplets[i].value();
    double deriv = RandomDouble() * 2 - 1;
    dAdp(solver.triplets[i].row(), solver.triplets[i].col()) += deriv;
    value.derivative = deriv;
    new_triplets.push_back(ExampleFEMProblem::Triplet(
          solver.triplets[i].row(), solver.triplets[i].col(), value));
  }
  solver.triplets.swap(new_triplets);
  for (int i = 0; i < solver.rhs.size(); i++) {
    double deriv = RandomDouble() * 2 - 1;
    solver.rhs[i].derivative = deriv;
    dbdp[i] = deriv;
  }

  // Compute what the solution derivative should be.
  Eigen::VectorXd soln(m);
  for (int i = 0; i < soln.size(); i++) {
    soln[i] = solver.solution[solver.reverse_index_map[i]];
  }
  Eigen::VectorXd target = solver.factorizer->solve(-dAdp * soln + dbdp);
  solver.PadSolution(&target);

  // Compare.
  ExampleFEMProblem::MNumberVector solution_derivative;
  CHECK(solver.ComputeSolutionDerivative(&solution_derivative));
  CHECK(solution_derivative.size() == 25);
  for (int i = 0; i < solution_derivative.size(); i++) {
    CHECK(fabs(solution_derivative[i] - target[i]) < 1e-9);
  }
}

TEST_FUNCTION(EigenSystem) {
  FEMSolver<ExampleFEMProblem> solver;
  for (int i = 0; i < solver.NumTriangles() * 3; i++) {
    solver.test_g[i] = -1;
    solver.test_a[i] = 0;
  }
  CHECK(solver.EigenSystem(5, 0));
  Eigen::VectorXd ev;
  for (int i = 0; i < 5; i++) {
    printf("Eigenvalue %d = %f\n", i, solver.GetEigenvalue(i));
  }
  CHECK(fabs(solver.GetEigenvalue(0) - 0.323741) < 1e-6);
  CHECK(fabs(solver.GetEigenvalue(1) - 1.741238) < 1e-6);
  CHECK(fabs(solver.GetEigenvalue(2) - 1.865839) < 1e-6);
  CHECK(fabs(solver.GetEigenvalue(3) - 3.737993) < 1e-6);
  CHECK(fabs(solver.GetEigenvalue(4) - 5.467924) < 1e-6);

  // Make sure A*x = lambda*B*x for all x,lambda.
  Eigen::SparseMatrix<double> A, B;
  solver.GetSystemMatrix(solver.triplets, &A);
  solver.GetSystemMatrix(solver.Ctriplets, &B);
  for (int i = 0; i < 5; i++) {
    const Eigen::VectorXd vec = solver.GetRawEigenvector(i);
    Eigen::VectorXd error = A*vec - solver.GetEigenvalue(i)*B*vec;
    CHECK(error.norm() < 1e-10);
  }
}

}  // namespace FEM
