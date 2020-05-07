
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include <stdio.h>

struct Block {
  int hello;

  Block() {}
  explicit Block(double value) {}
  void operator=(double value) {}
  void operator+=(const Block &a) {}
  void operator-=(const Block &a) {}

  bool operator==(const Block &a) { return false; }
  bool operator<=(const Block &a) { return false; }      //@@@ problematic

  // For SparseLU only:
  Block operator*=(const Block &a) { return Block(); }
  void operator/=(const Block &a) {}
  bool operator!=(const Block &a) { return false; }
  //bool operator>(const Block &a) { return false; }      //@@@ problematic
  bool operator<(const Block &a) { return false; }      //@@@ problematic
  bool operator>=(const Block &a) { return false; }      //@@@ problematic
};

Block operator+(const Block &a, const Block &b) { return Block(); }
Block operator*(const Block &a, const Block &b) { return Block(); }
Block operator/(const Block &a, const Block &b) { return Block(); }

// For SparseLU only:
Block operator-(const Block &a, const Block &b) { return Block(); }
Block abs(const Block &a) { return Block(); }           //@@@ problematic

// For SimplicialLDLT only:
//Block sqrt(const Block &a) { return Block(); }           //@@@ hmm

int main(int argc, char **argv) {

  Eigen::SparseMatrix<Block> a(5,5), b(5,5);
  Eigen::SparseLU<Eigen::SparseMatrix<Block>> solver;
  //Eigen::SimplicialLDLT<Eigen::SparseMatrix<Block>, Eigen::Lower, Eigen::NaturalOrdering<int>> solver;
  solver.analyzePattern(a);
  solver.factorize(a);

  printf("%ld,%ld\n", a.rows(), a.cols());


  return 0;
}
