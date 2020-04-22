
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

// Pop a function from the lua stack, return a unique index for this function,
// and store the function in the registry. The inque index is always > 0, so
// that the index 0 can be regarded as a special value meaning "no callback".
int64_t PutCallbackInRegistry(lua_State *L);

// Push the function with the given unique ID to the lua stack.
void GetCallbackFromRegistry(lua_State *L, int64_t unique_id);

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
