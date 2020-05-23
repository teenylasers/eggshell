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

#define GL_FONT_IMPLEMENTATION

#include "common.h"
#include "../toolkit/lua_vector.h"

//***************************************************************************
// Lua utility.

LuaCallback::LuaCallback(lua_State *L) {
  // Pop a function from the lua stack and store it in the "rama" table in the
  // registry. The index is always > 0, so that the index 0 can be regarded as
  // a special value meaning "no callback".
  int top = lua_gettop(L);
  CHECK(lua_type(L, -1) == LUA_TFUNCTION);      // fn
  LuaHash(L, &hash_, true);

  // To avoid key collisions in the registry we store every Rama-specific thing
  // in the "rama" table. Create this if it doesn't exist.
  lua_pushliteral(L, "rama");                   // fn "rama"
  lua_rawget(L, LUA_REGISTRYINDEX);             // fn T
  if (lua_type(L, -1) != LUA_TTABLE) {
    lua_pop(L, 1);                              // fn
    lua_newtable(L);                            // fn T
    lua_pushliteral(L, "rama");                 // fn T "rama"
    lua_pushvalue(L, -2);                       // fn T "rama" T
    lua_rawset(L, LUA_REGISTRYINDEX);           // fn T
  }
  index_ = lua_rawlen(L, -1) + 1;               // fn T
  lua_pushvalue(L, -2);                         // fn T fn
  lua_rawseti(L, -2, index_);                   // fn T
  lua_pop(L, 2);
  CHECK(lua_gettop(L) == top - 1);              // Should have popped argument
}

void LuaCallback::Push(lua_State *L) const {
  // Push the function with the given index to the lua stack.
  CHECK(Valid());
  int top = lua_gettop(L);
  lua_pushliteral(L, "rama");                   // "rama"
  lua_rawget(L, LUA_REGISTRYINDEX);             // T
  CHECK(lua_type(L, -1) == LUA_TTABLE);
  lua_rawgeti(L, -1, index_);                   // T fn
  lua_remove(L, top + 1);                       // fn
  CHECK(lua_type(L, -1) == LUA_TFUNCTION);
  CHECK(lua_gettop(L) == top + 1);              // One return value
}

bool ToJetComplex(lua_State *L, int index, JetComplex *value) {
  int top = lua_gettop(L);
  if (lua_type(L, index) == LUA_TNUMBER) {
    *value = JetComplex(lua_tonumber(L, index));
    return true;
  }
  if (lua_getmetatable(L, index)) {             // Stack: T
    lua_getglobal(L, "__complex_metatable__");  // Stack: T T
    if (lua_rawequal(L, -1, -2)) {
      // This looks like a complex table, make sure the real and imaginary
      // components are scalars.
      lua_pop(L, 2);                            // Stack:
      lua_rawgeti(L, index, 1);                 // Stack: re
      if (lua_type(L, -1) == LUA_TNUMBER) {
        JetNum real_part = lua_tonumber(L, -1);
        lua_pop(L, 1);                          // Stack:
        lua_rawgeti(L, index, 2);               // Stack: im
        if (lua_type(L, -1) == LUA_TNUMBER) {
          JetNum imag_part = lua_tonumber(L, -1);
          *value = JetComplex(real_part, imag_part);
          lua_settop(L, top);
          return true;
        }
      }
    }
  }
  lua_settop(L, top);
  return false;
}

bool ToJetComplexVector(lua_State *L, int index,
                        std::vector<JetComplex> *value) {
  int top = lua_gettop(L);
  LuaVector *result_real = LuaCastTo<LuaVector>(L, index);
  if (result_real) {
    value->resize(result_real->size());
    for (int i = 0; i < result_real->size(); i++) {
      (*value)[i] = (*result_real)[i];
    }
    return true;
  }
  if (lua_getmetatable(L, index)) {             // Stack: T
    lua_getglobal(L, "__complex_metatable__");  // Stack: T T
    if (lua_rawequal(L, -1, -2)) {
      // This looks like a complex table, make sure the real and imaginary
      // components are vectors of the same size.
      lua_pop(L, 2);                            // Stack:
      lua_rawgeti(L, index, 1);                 // Stack: re
      result_real = LuaCastTo<LuaVector>(L, -1);
      if (result_real) {
        lua_pop(L, 1);                          // Stack:
        lua_rawgeti(L, index, 2);               // Stack: im
        LuaVector *result_imag = LuaCastTo<LuaVector>(L, -1);
        if (result_imag && result_imag->size() == result_real->size()) {
          value->resize(result_real->size());
          for (int i = 0; i < result_real->size(); i++) {
            (*value)[i] = JetComplex((*result_real)[i], (*result_imag)[i]);
          }
          lua_settop(L, top);
          return true;
        }
      }
    }
  }
  lua_settop(L, top);
  return false;
}

void LuaErrorIfNaNs(lua_State *L) {
  int n = lua_gettop(L);
  for (int i = 1; i <= n; i++) {
    JetComplex value;
    std::vector<JetComplex> vvalue;
    if (ToJetComplex(L, i, &value)) {
      if (IsNaNValue(value.real()) || IsNaNValue(value.imag())) {
        LuaError(L, "A NaN (not-a-number) was passed to a function.");
      }
    } else if (ToJetComplexVector(L, i, &vvalue)) {
      for (int j = 0; j < vvalue.size(); j++) {
        if (IsNaNValue(vvalue[j].real()) || IsNaNValue(vvalue[j].imag())) {
          LuaError(L, "A NaN (not-a-number) was passed to a function "
                      "in a vector.");
        }
      }
    }
  }
}

//***************************************************************************
// Common fonts.

Font port_number_font;
Font s_parameter_font;
Font mesh_statistics_font;
Font debug_string_font;

#ifdef QT_CORE_LIB

#include <QFont>
#include <QFontMetrics>

void CreateStandardFonts(double content_scale_factor) {
  const double scale = FONT_SCALE;
  if (!port_number_font.font) {
    port_number_font.font = new QFont("Courier", 10 * scale, QFont::Normal);
    port_number_font.font->setStyleHint(QFont::TypeWriter);
    port_number_font.fm = new QFontMetrics(*port_number_font.font);

    s_parameter_font.font = new QFont("Helvetica", 10 * scale, QFont::Normal);
    s_parameter_font.font->setStyleHint(QFont::QFont::SansSerif);
    s_parameter_font.fm = new QFontMetrics(*s_parameter_font.font);

    mesh_statistics_font.font = new QFont("Helvetica", 10 * scale,
                                          QFont::Normal, true);
    mesh_statistics_font.font->setStyleHint(QFont::QFont::SansSerif);
    mesh_statistics_font.fm = new QFontMetrics(*mesh_statistics_font.font);

    debug_string_font.font = new QFont("Helvetica", 10 * scale, QFont::Normal);
    debug_string_font.font->setStyleHint(QFont::QFont::SansSerif);
    debug_string_font.fm = new QFontMetrics(*debug_string_font.font);
  }
}

#endif
