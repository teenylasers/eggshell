
#define GL_FONT_IMPLEMENTATION

#include "common.h"
#include "../toolkit/lua_vector.h"

//***************************************************************************
// Lua utility.

std::string PutCallbackInRegistry(lua_State *L) {
  CHECK(lua_type(L, -1) == LUA_TFUNCTION);
  std::string hash;
  LuaGetObject(L)->Hash(&hash, true);           // Does not pop function
  lua_pushlstring (L, hash.data(), hash.size());
  lua_rotate(L, -2, 1);                         // Swap top 2 elements
  lua_rawset(L, LUA_REGISTRYINDEX);
  return hash;
}

void GetCallbackFromRegistry(lua_State *L, std::string hash) {
  CHECK(hash.size() == 16);                     // Make sure it's an MD5 hash
  lua_pushlstring (L, hash.data(), hash.size());
  lua_rawget(L, LUA_REGISTRYINDEX);
  CHECK(lua_type(L, -1) == LUA_TFUNCTION);
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

//***************************************************************************
// Common fonts.

Font port_number_font;
Font s_parameter_font;
Font mesh_statistics_font;
Font debug_string_font;

#ifdef __TOOLKIT_WXWINDOWS__

#include "stdwx.h"

void CreateStandardFonts(double content_scale_factor) {
  if (!port_number_font.font) {
    double scale = content_scale_factor * FONT_SCALE;
    port_number_font.font = new wxFont(10 * scale, wxFONTFAMILY_TELETYPE,
                                       wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD);
    s_parameter_font.font = new wxFont(10 * scale, wxFONTFAMILY_SWISS,
                                       wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    mesh_statistics_font.font = new wxFont(10 * scale, wxFONTFAMILY_SWISS,
                                       wxFONTSTYLE_ITALIC, wxFONTWEIGHT_NORMAL);
    debug_string_font.font = new wxFont(10 * scale, wxFONTFAMILY_SWISS,
                                       wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
  }
}

#endif  // __TOOLKIT_WXWINDOWS__

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
