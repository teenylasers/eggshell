// Lua vectors that behave much like matlab vectors, i.e. all math operations
// are elementwise over the vector. These are necessary in some mesh callback
// functions so those functions don't have to be called for every mesh point
// (which can be very slow).

#ifndef __TOOLKIT_LUA_VECTOR_H__
#define __TOOLKIT_LUA_VECTOR_H__

#include "lua_util.h"
#include "my_jet.h"

class LuaVector : public LuaUserClass {
 public:
  int size() const { return v_.size(); }
  void resize(int vector_size);
  JetNum & operator[](int i) { return v_[i]; }

  int Index(lua_State *L);
  int NewIndex(lua_State *L);
  int Length(lua_State *L);
  bool Operator(lua_State *L, int op, int pos);

  int LuaResize(lua_State *L);

  // Setup the global 'vec' table that contains vector math functions.
  static void SetLuaGlobals(lua_State *L);

 private:
  VectorJetNum v_;
};

#endif
