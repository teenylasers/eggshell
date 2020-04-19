
#ifndef __COMMON_H__
#define __COMMON_H__

#include <myvector>
#include <complex>
#include "../toolkit/error.h"
#include "../toolkit/trace.h"
#include "../toolkit/platform.h"
#include "../toolkit/gl_font.h"
#include "../toolkit/lua_util.h"

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
// Lua utility.

// Pop a function from the lua stack, return its hash and store the function in
// the registry, using the hash as the registry key.
std::string PutCallbackInRegistry(lua_State *L);

// Push a function to the lua stack from the registry, using the hash as the
// registry key.
void GetCallbackFromRegistry(lua_State *L, std::string hash);

// Set 'value' to the real or complex lua value at position 'index' on the
// stack. Return true on success or false if the stack value can not be
// interpreted as a complex value. Leave the stack unchanged on exit.
bool ToJetComplex(lua_State *L, int index, JetComplex *value);

// Set 'value' to the real or complex lua vector at position 'index' on the
// stack. Return true on success or false if the stack value can not be
// interpreted as a complex vector. Leave the stack unchanged on exit.
bool ToJetComplexVector(lua_State *L, int index,
                        std::vector<JetComplex> *value);

//***************************************************************************
// Common fonts.

// Some useful fonts, to be passed as arguments to DrawString().
extern Font port_number_font;
extern Font s_parameter_font;
extern Font mesh_statistics_font;
extern Font debug_string_font;

void CreateStandardFonts(double content_scale_factor);

#endif
