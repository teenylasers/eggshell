// Rama Simulator, Copyright (C) 2014-2020 Russell Smith.
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

#ifndef __COMMON_H__
#define __COMMON_H__

#include <vector>
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

// LuaCallback is a reference to a Lua callback function, stored in e.g.
// material callbacks. The important properties of references are (1)
// probabilistic comparison and (2) references can be used to look up the
// same function in a different lua_State that has been initialized with the
// same script.

class LuaCallback {
 public:
  // Create an empty callback, so Valid() returns false.
  LuaCallback() {}

  // Construct a new callback by popping a function from the Lua stack and
  // saving it in the registry.
  explicit LuaCallback(lua_State *L);

  // Return true if this is an actual callback function.
  bool Valid() const { return index_ != 0; }

  // Probabilistic comparison. If two LuaCallback objects don't equal each
  // other, they probably reference different functions. If they do equal each
  // other, they *might* reference the same functions (but hard to say, since
  // function upvalues are not considered, so the functions might return
  // different results for the same arguments).
  bool operator==(const LuaCallback &f) const {
    return hash_ == f.hash_ && index_ == f.index_;
  }

  // For ordering callbacks in maps:
  bool operator<(const LuaCallback &f) const {
    if (index_ < f.index_) return true;
    if (index_ > f.index_) return false;
    return hash_ < f.hash_;
  }

  // Push this function to the Lua stack. Valid() must be true.
  void Push(lua_State *L) const;

 private:
  std::string hash_;            // Hash of function dump
  int64_t index_ = 0;           // Index into Registry "rama" table
};

// Set 'value' to the real or complex lua value at position 'index' on the
// stack. Return true on success or false if the stack value can not be
// interpreted as a complex value. Leave the stack unchanged on exit.
bool ToJetComplex(lua_State *L, int index, JetComplex *value);

// Set 'value' to the real or complex lua vector at position 'index' on the
// stack. Return true on success or false if the stack value can not be
// interpreted as a complex vector. Leave the stack unchanged on exit.
bool ToJetComplexVector(lua_State *L, int index,
                        std::vector<JetComplex> *value);

// Check that any numbers in a Lua function argument list are not NaNs or
// infinity. These likely indicate a problem building the model, and will make
// a mess of our algorithms if stored in shapes or materials because they
// violate various assumptions, e.g. (for NaNs) that if A is copied to B then
// A==B. Basic Lua numbers are checked, alse Vector and Complex arguments are
// checked.
void LuaErrorIfNaNOrInfs(lua_State *L);

//***************************************************************************
// Common fonts.

// Some useful fonts, to be passed as arguments to DrawString().
extern Font port_number_font;
extern Font s_parameter_font;
extern Font mesh_statistics_font;
extern Font debug_string_font;

void CreateStandardFonts(double content_scale_factor);

#endif
