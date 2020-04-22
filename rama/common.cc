
#define GL_FONT_IMPLEMENTATION

#include "common.h"
#include "../toolkit/lua_vector.h"

//***************************************************************************
// Lua utility.

int64_t PutCallbackInRegistry(lua_State *L) {
  int top = lua_gettop(L);
  CHECK(lua_type(L, -1) == LUA_TFUNCTION);      // fn

  // Create a new unique callback ID.
  static int64_t unique_id = 0;
  unique_id++;                                  // FYI, not thread safe

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
  lua_pushinteger(L, unique_id);                // fn T id
  lua_pushvalue(L, -3);                         // fn T id fn
  lua_rawset(L, -3);                            // fn T
  lua_pop(L, 2);
  CHECK(lua_gettop(L) == top - 1);              // Should have popped argument
  return unique_id;
}

void GetCallbackFromRegistry(lua_State *L, int64_t unique_id) {
  int top = lua_gettop(L);
  lua_pushliteral(L, "rama");                   // "rama"
  lua_rawget(L, LUA_REGISTRYINDEX);             // T
  CHECK(lua_type(L, -1) == LUA_TTABLE);
  lua_pushinteger(L, unique_id);                // T id
  lua_rawget(L, -2);                            // T fn
  lua_remove(L, top);                           // fn
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
