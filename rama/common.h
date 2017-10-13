
#ifndef __COMMON_H__
#define __COMMON_H__

#include <complex>
#include "error.h"
#include "trace.h"
#include "platform.h"

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
class wxFont;
extern const wxFont *port_number_font, *s_parameter_font, *mesh_statistics_font;

void CreateStandardFonts(double content_scale_factor);

#endif
