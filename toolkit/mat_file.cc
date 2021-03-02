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
// See https://www.mathworks.com/help/pdf_doc/matlab/matfile_format.pdf
//
// If this is compiled with __TOOLKIT_MAT_FILE_USE_ZLIB__ then the zlib library
// is assumed to be available and compressed data will be written. Otherwise,
// only uncompressed data will be written.
//
// Matlab file features not yet supported:
//   * Small data element format.
//   * "Automatic compression of numeric data".
//   * Sparse arrays.
//   * Cell arrays.
//   * Structs.
//   * Big endian architectures.
//   * Files > 2G.

#define _FILE_OFFSET_BITS 64    // Needed for large files on linux

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <vector>
#include "mat_file.h"
#include "error.h"
#ifdef __TOOLKIT_MAT_FILE_USE_ZLIB__
#include "zlib.h"
#endif

using std::vector;

//***************************************************************************
// Matlab v7 file format.

// 128 byte file header.
struct FileHeader {
  char description[116];        // First 4 bytes can not be zero
  uint64_t subsys_offset;       // Subsystem-specific data offset (0=none)
  int8_t version[2];            // Set to 0x0100 in big endian format
  char endian[2];               // 'I', 'M' for little endian format
} __attribute__((packed));

// 8 byte data element header.
struct ElementHeader {
  uint32_t type;                // A TYPE_xxx value
  uint32_t size;                // Element size not including this header
} __attribute__((packed));

// Element types.
enum {
  TYPE_INT8        = 1,
  TYPE_UINT8       = 2,
  TYPE_INT16       = 3,
  TYPE_UINT16      = 4,         // Also for non-unicode character data
  TYPE_INT32       = 5,
  TYPE_UINT32      = 6,
  TYPE_SINGLE      = 7,
  TYPE_DOUBLE      = 9,
  TYPE_INT64       = 12,
  TYPE_UINT64      = 13,
  TYPE_MATRIX      = 14,
  TYPE_COMPRESSED  = 15,
  TYPE_UTF8        = 16,
  TYPE_UTF16       = 17,
  TYPE_UTF32       = 18,
};

static size_t BytesForClassType(int mx_class) {
  switch (mx_class) {
    case MatFile::mxDOUBLE_CLASS : return 8;
    case MatFile::mxSINGLE_CLASS : return 4;
    case MatFile::mxINT8_CLASS   : return 1;
    case MatFile::mxUINT8_CLASS  : return 1;
    case MatFile::mxINT16_CLASS  : return 2;
    case MatFile::mxUINT16_CLASS : return 2;
    case MatFile::mxINT32_CLASS  : return 4;
    case MatFile::mxUINT32_CLASS : return 4;
    case MatFile::mxINT64_CLASS  : return 8;
    case MatFile::mxUINT64_CLASS : return 8;
    default: Panic("Unsupported class type");
  }
}

static int ElementTypeForClassType(int mx_class) {
  switch (mx_class) {
    case MatFile::mxDOUBLE_CLASS : return TYPE_DOUBLE;
    case MatFile::mxSINGLE_CLASS : return TYPE_SINGLE;
    case MatFile::mxINT8_CLASS   : return TYPE_INT8;
    case MatFile::mxUINT8_CLASS  : return TYPE_UINT8;
    case MatFile::mxINT16_CLASS  : return TYPE_INT16;
    case MatFile::mxUINT16_CLASS : return TYPE_UINT16;
    case MatFile::mxINT32_CLASS  : return TYPE_INT32;
    case MatFile::mxUINT32_CLASS : return TYPE_UINT32;
    case MatFile::mxINT64_CLASS  : return TYPE_INT64;
    case MatFile::mxUINT64_CLASS : return TYPE_UINT64;
    default: Panic("Unsupported class type");
  }
}

//***************************************************************************
// Compression.

namespace {

bool FWrite(bool &valid, FILE *fout, const void *ptr, size_t sz) {
  if (valid && sz > 0 && fwrite(ptr, sz, 1, fout) != 1) {
    Error("Matlab file write failed (%s)", strerror(errno));
    valid = false;
  }
  return valid;
}

class CompressedFile {
 public:
  CompressedFile(FILE *fout, bool compress);
  ~CompressedFile();

  // Compress the given memory buffer to the file.
  void Write(const void *data, size_t size);

  // No more data will be written to the file so finalize the compressed
  // stream. Return Valid(). This must be called before the destructor.
  bool Finalize() MUST_USE_RESULT;

  // Return true if all file write and compression operations so far succeeded.
  bool Valid() const MUST_USE_RESULT { return valid_; }

 private:
  bool valid_;          // true if no file write or compression errors so far
  FILE *fout_;
  bool compress_, finalized_;
  #ifdef __TOOLKIT_MAT_FILE_USE_ZLIB__
  z_stream_s strm_;
  #endif

  DISALLOW_COPY_AND_ASSIGN(CompressedFile);
};

CompressedFile::CompressedFile(FILE *fout, bool compress) {
  valid_ = true;
  fout_ = fout;
  compress_ = compress;
  finalized_ = false;
  #ifdef __TOOLKIT_MAT_FILE_USE_ZLIB__
    memset(&strm_, 0, sizeof(strm_));
    if (compress_) {
      CHECK(deflateInit (&strm_, Z_DEFAULT_COMPRESSION) == Z_OK);
    }
  #else
    if (compress) {
      Panic("mat_file.cc was not compiled with compressed file ability.");
    }
  #endif
}

CompressedFile::~CompressedFile() {
  CHECK(finalized_);
}

bool CompressedFile::Finalize() {
  #ifdef __TOOLKIT_MAT_FILE_USE_ZLIB__
  if (compress_) {
    if (valid_ && !finalized_) {
      vector<uint8_t> buffer(100000);
      strm_.next_in = 0;
      strm_.avail_in = 0;
      for (;;) {
        strm_.next_out = buffer.data();
        strm_.avail_out = buffer.size();
        int status = deflate(&strm_, Z_FINISH);
        CHECK(status == Z_OK || status == Z_STREAM_END);
        size_t sz = strm_.next_out - buffer.data();
        if (!FWrite(valid_, fout_, buffer.data(), sz) ||
            status == Z_STREAM_END) {
          break;
        }
      }
    }
    CHECK(deflateEnd(&strm_) == Z_OK);
  }
  memset(&strm_, 0, sizeof(strm_));
  #endif
  finalized_ = true;
  return valid_;
}

void CompressedFile::Write(const void *data, size_t size) {
  if (!valid_ || finalized_) {
    return;
  }
  if (compress_) {
    #ifdef __TOOLKIT_MAT_FILE_USE_ZLIB__
    strm_.next_in = (Bytef*) data;
    strm_.avail_in = size;
    vector<uint8_t> buffer(100000);
    while (strm_.avail_in > 0) {
      strm_.next_out = buffer.data();
      strm_.avail_out = buffer.size();
      CHECK(deflate(&strm_, Z_NO_FLUSH) == Z_OK);
      size_t sz = strm_.next_out - buffer.data();
      if (!FWrite(valid_, fout_, buffer.data(), sz)) {
        return;
      }
    }
    #endif
  } else {
    FWrite(valid_, fout_, data, size);
  }
}

}  // anonymous namespace

//***************************************************************************
// MatFile.

MatFile::MatFile(const char *filename) {
  static_assert(sizeof(off_t) == 8, "off_t not 64 bits");
  static_assert(sizeof(size_t) == 8, "size_t not 64 bits");
  static_assert(sizeof(FileHeader) == 128, "File Header struct not packed. "
    "On g++ x64 you might want to compile with -mno-ms-bitfields");
  static_assert(sizeof(ElementHeader) == 8, "ElementHeader struct not packed.");

  fout_ = fopen(filename, "wb");
  valid_ = (fout_ != 0);
  if (!valid_) {
    Error("Can not write to '%s' (%s)", filename, strerror(errno));
    return;
  }

  FileHeader hdr;
  memset(&hdr, 0, sizeof(hdr));
  memset(hdr.description, ' ', sizeof(hdr.description));
  hdr.version[1] = 1;
  hdr.endian[0] = 'I';
  hdr.endian[1] = 'M';
  FWrite(valid_, fout_, &hdr, sizeof(hdr));
}

MatFile::~MatFile() {
  if (fout_) {
    fclose(fout_);
  }
}

void MatFile::WriteMatrix(const char *name, int ndims, int dims[],
                          int mx_class, const void *real_data,
                          const void *imag_data) {
  if (!valid_) {
    return;
  }

  // Optionally turn off compression for debugging:
  const bool kCompress = true;

  // Assumed below when 'dims' is written directly to the file.
  static_assert(sizeof(int) == sizeof(int32_t), "int not 32 bits");

  // Make sure data portion is not too large for this file format.
  size_t num_elements = 1;
  for (int i = 0; i < ndims; i++) {
    num_elements *= size_t(dims[i]);
  }
  size_t data_size = num_elements * BytesForClassType(mx_class);
  if (data_size >> 31) {
    Panic("Matrix data larger than 2Gb can not be written");
  }

  // Sizes for the various elements must be 8-byte aligned. Compute padding
  // sizes here.
  int namelen = strlen(name);
  size_t pad1 = 4 * (ndims & 1);        // For dimensions
  size_t pad2 = (8 - namelen) & 7;      // For name
  size_t pad3 = (8 - data_size) & 7;    // For data
  uint64_t padding = 0;

  // Write header for compressed matrix. We don't know the actual size yet.
  off_t offset1 = ftello(fout_);
  CHECK(offset1 != -1);         // See if ftello() failed
  ElementHeader hdr;
  if (kCompress) {
    hdr.type = TYPE_COMPRESSED;
    hdr.size = 0;
    FWrite(valid_, fout_, &hdr, sizeof(hdr));
  }

  {
    // Write the header. For TYPE_MATRIX the size must include the padding
    // bytes.
    CompressedFile f(fout_, kCompress);
    int imag = (imag_data != 0);
    hdr.type = TYPE_MATRIX;
    hdr.size = sizeof(ElementHeader) + 8 +                        // Flags
               sizeof(ElementHeader) + ndims * 4 + pad1 +         // Dimensions
               sizeof(ElementHeader) + namelen + pad2 +           // Name
               sizeof(ElementHeader) + data_size + pad3 +         // Real data
               imag * (sizeof(ElementHeader) + data_size + pad3); // Imag data
    f.Write(&hdr, sizeof(hdr));

    // Write flags.
    hdr.type = TYPE_UINT32;
    hdr.size = 8;
    uint8_t flags[8];
    memset(flags, 0, sizeof(flags));
    flags[0] = mx_class;
    flags[1] = 0x08 * imag;
    f.Write(&hdr, sizeof(hdr));
    f.Write(flags, sizeof(flags));

    // Write dimensions.
    hdr.type = TYPE_INT32;
    hdr.size = ndims * sizeof(int32_t);
    f.Write(&hdr, sizeof(hdr));
    f.Write(dims, hdr.size);
    if (pad1 > 0) {
      f.Write(&padding, pad1);
    }

    // Write name.
    hdr.type = TYPE_INT8;
    hdr.size = namelen;
    f.Write(&hdr, sizeof(hdr));
    f.Write(name, namelen);
    if (pad2 > 0) {
      f.Write(&padding, pad2);
    }

    // Write real part.
    hdr.type = ElementTypeForClassType(mx_class);
    hdr.size = data_size;
    f.Write(&hdr, sizeof(hdr));
    f.Write(real_data, data_size);
    if (pad3 > 0) {
      f.Write(&padding, pad3);
    }

    // Write optional imaginary part.
    if (imag_data) {
      f.Write(&hdr, sizeof(hdr));
      f.Write(imag_data, data_size);
      if (pad3 > 0) {
        f.Write(&padding, pad3);
      }
    }

    // Finalize the compressed data.
    valid_ &= f.Finalize();
  }

  // Write the proper size to the compressed element header. The size must
  // include the padding bytes at the end.
  if (kCompress) {
    off_t offset2 = ftello(fout_);
    CHECK(offset2 != -1);               // See if ftello() failed
    int pad4 = (-offset2) & 7;
    off_t size = offset2 - offset1 - sizeof(ElementHeader) + pad4;
    hdr.type = TYPE_COMPRESSED;
    hdr.size = size;
    CHECK(hdr.size == size);            // Check for elements too big to write
    valid_ &= valid_ && (fseeko(fout_, offset1, SEEK_SET) == 0);
    FWrite(valid_, fout_, &hdr, sizeof(hdr));
    valid_ &= valid_ && (fseeko(fout_, offset2, SEEK_SET) == 0);

    // Padding for compressed element.
    if (pad4 > 0) {
      FWrite(valid_, fout_, &padding, pad4);
    }
  }
}

void MatFile::WriteScalar(const char *name, double value) {
  int32_t dims[2];
  dims[0] = 1;
  dims[1] = 1;
  WriteMatrix(name, 2, dims, mxDOUBLE_CLASS, &value, 0);
}

void MatFile::WriteSparseMatrix(const char *name, int rows, int cols,
                                int mx_class, int nonzeros,
                                const int *row_indexes, const int *col_indexes,
                                const void *real_data, const void *imag_data) {
  if (!valid_) {
    return;
  }

  // Sizes for the various elements must be 8-byte aligned. Compute padding
  // sizes here.
  size_t value_size = nonzeros * BytesForClassType(mx_class);
  if (value_size >> 31) {
    Panic("Matrix data larger than 2Gb can not be written");
  }
  int namelen = strlen(name);
  size_t name_pad = (8 - namelen) & 7;
  size_t ir_size = nonzeros * sizeof(int32_t);
  size_t ir_pad = (8 - ir_size) & 7;
  size_t jc_size = (cols + 1) * sizeof(int32_t);
  size_t jc_pad = (8 - jc_size) & 7;
  size_t value_pad = (8 - value_size) & 7;
  uint64_t padding = 0;

  // We are not currently using compressed data storage here. We could.
  CompressedFile f(fout_, false);

  // Write header.
  int imag = (imag_data != 0);
  ElementHeader hdr;
  hdr.type = TYPE_MATRIX;
  hdr.size = sizeof(ElementHeader) + 8 +                        // Flags
             sizeof(ElementHeader) + 2 * 4 +                    // Dimensions
             sizeof(ElementHeader) + namelen + name_pad +       // Name
             sizeof(ElementHeader) + ir_size + ir_pad +         // IR
             sizeof(ElementHeader) + jc_size + jc_pad +         // JC
             (imag + 1)*
             (sizeof(ElementHeader) + value_size + value_pad);  // PR,PI
  f.Write(&hdr, sizeof(hdr));

  // Write flags.
  hdr.type = TYPE_UINT32;
  hdr.size = 8;
  f.Write(&hdr, sizeof(hdr));
  uint8_t *hdr_as_bytes = (uint8_t *) &hdr;
  hdr.type = mxSPARSE_CLASS;
  hdr_as_bytes[1] = 0x08 * imag;
  hdr.size = nonzeros;
  f.Write(&hdr, sizeof(hdr));

  // Write dimensions.
  hdr.type = TYPE_INT32;
  hdr.size = 2 * sizeof(int32_t);
  f.Write(&hdr, sizeof(hdr));
  int32_t dims[2];
  dims[0] = rows;
  dims[1] = cols;
  f.Write(dims, hdr.size);

  // Write name.
  hdr.type = TYPE_INT8;
  hdr.size = namelen;
  f.Write(&hdr, sizeof(hdr));
  f.Write(name, namelen);
  f.Write(&padding, name_pad);

  // Write IR (row indices of the nonzero elements, one for each nonzero).
  hdr.type = TYPE_INT32;
  hdr.size = ir_size;
  f.Write(&hdr, sizeof(hdr));
  f.Write(row_indexes, hdr.size);
  f.Write(&padding, ir_pad);

  // Write JC (column offsets, one for each column, plus one more for total).
  hdr.type = TYPE_INT32;
  hdr.size = jc_size;
  f.Write(&hdr, sizeof(hdr));
  f.Write(col_indexes, hdr.size);
  f.Write(&padding, jc_pad);

  // Write PR, PI.
  hdr.type = TYPE_DOUBLE;
  hdr.size = value_size;
  f.Write(&hdr, sizeof(hdr));
  f.Write(real_data, hdr.size);
  f.Write(&padding, value_pad);
  if (imag_data) {
    f.Write(&hdr, sizeof(hdr));
    f.Write(imag_data, hdr.size);
    f.Write(&padding, value_pad);
  }

  // Finalize.
  valid_ &= f.Finalize();
}

void MatFile::WriteSparseMatrix(const char *name,
                                const Eigen::SparseMatrix<double> &A_arg) {
  if (!valid_) {
    return;
  }

  // For fast writes we want "compressed matrix storage", which uses a
  // matlab-compatible data structure. Note that this means "row,column,value"
  // format, not z-lib compression.
  Eigen::SparseMatrix<double> *Astore = 0;
  auto *A = &A_arg;
  if (!A->isCompressed()) {
    Astore = new Eigen::SparseMatrix<double>(A_arg);
    Astore->makeCompressed();
    A = Astore;
  }

  WriteSparseMatrix(name, A->rows(), A->cols(), mxDOUBLE_CLASS, A->nonZeros(),
                    A->innerIndexPtr(), A->outerIndexPtr(),
                    A->valuePtr(), 0);

  delete Astore;
}
