
#ifndef __COMMON_H__
#define __COMMON_H__

#include <complex>
#include "../toolkit/error.h"
#include "../toolkit/trace.h"
#include "../toolkit/platform.h"
#include "../toolkit/wxgl_font.h"

// Types
typedef signed char int8;
typedef signed short int16;
typedef signed int int32;
typedef signed long long int int64;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long int uint64;
typedef std::complex<double> Complex;

//***************************************************************************
// Convenience functions.

template <class T> inline T sqr(T x) {
  return x * x;
}

inline double RandDouble() {
  return double(rand()) / double(RAND_MAX);
}

template <class T> inline T NormalizeAngle(T a) {
  while (a > M_PI) {
    a -= 2 * M_PI;
  }
  while (a < -M_PI) {
    a += 2 * M_PI;
  }
  return a;
}

//***************************************************************************
// Common fonts.

// Some useful fonts, to be passed as arguments to DrawString().
extern Font port_number_font;
extern Font s_parameter_font;
extern Font mesh_statistics_font;
extern Font debug_string_font;

void CreateStandardFonts(double content_scale_factor);

#endif
