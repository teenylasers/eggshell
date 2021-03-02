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

// Write matrix data in Matlab v7 file format.

#ifndef __TOOLKIT_MAT_FILE_H__
#define __TOOLKIT_MAT_FILE_H__

#include <stdio.h>
#include "error.h"
#include "Eigen/Sparse"

class MatFile {
 public:
  // Create the given matlab file for writing.
  explicit MatFile(const char *filename);

  // Close the file (this will not generate an error).
  ~MatFile();

  // Return true if the file has been created and written successfully so far.
  // This can be called at any point, and should probably be called just before
  // the destructor. If false is returned an Error() will have been emitted.
  bool Valid() const MUST_USE_RESULT { return valid_; }

  // Write a generic matrix. 'dims' is an array of 'ndims' elements that gives
  // the size of each matrix dimension. mx_class is an mxXXX constant that
  // specifies the type of the data. If imag_data is nonzero then this is a
  // complex matrix and imag_data points to the imaginary part.
  void WriteMatrix(const char *name, int ndims, int dims[], int mx_class,
                   const void *real_data, const void *imag_data);

  // Specialize WriteMatrix() to some specific matrix types.
  void WriteScalar(const char *name, double value);

  // Write a generic sparse matrix.
  void WriteSparseMatrix(const char *name, int rows, int cols,
                         int mx_class, int nonzeros,
                         const int *row_indexes, const int *col_indexes,
                         const void *real_data, const void *imag_data);

  // Write a sparse Eigen matrix.
  void WriteSparseMatrix(const char *name,
                         const Eigen::SparseMatrix<double> &A);

  // Array types passed to WriteMatrix().
  enum {
    mxCELL_CLASS   = 1,
    mxSTRUCT_CLASS = 2,
    mxOBJECT_CLASS = 3,
    mxCHAR_CLASS   = 4,
    mxSPARSE_CLASS = 5,
    mxDOUBLE_CLASS = 6,
    mxSINGLE_CLASS = 7,
    mxINT8_CLASS   = 8,
    mxUINT8_CLASS  = 9,
    mxINT16_CLASS  = 10,
    mxUINT16_CLASS = 11,
    mxINT32_CLASS  = 12,
    mxUINT32_CLASS = 13,
    mxINT64_CLASS  = 14,
    mxUINT64_CLASS = 15,
  };

 private:
  FILE *fout_;
  bool valid_;

  DISALLOW_COPY_AND_ASSIGN(MatFile);
};

#endif
