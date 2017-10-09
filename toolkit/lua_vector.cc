#include "lua_vector.h"

#define COMMA ,         // Used to pass commas as macro arguments

// Given arguments to a binary operator at positions 1 and 2 on the stack,
// return the corresponding LuaVector objects and the scalar to use when either
// of the object pointers is 0. Return the result vector to populate. Generate
// an error for bad arguments.
static inline LuaVector *BinaryOperatorArguments(
       lua_State *L, LuaVector **op1_ret, LuaVector **op2_ret, JetNum *scalar) {
  LuaVector *op1 = LuaCastTo<LuaVector>(L, 1);
  LuaVector *op2 = LuaCastTo<LuaVector>(L, 2);
  int n = 0;
  *scalar = 0;
  if (op1 && op2 && op1->size() == op2->size()) {
    n = op1->size();
  } else if (op1 && lua_type(L, 2) == LUA_TNUMBER) {
    n = op1->size();
    *scalar = lua_tonumber(L, 2);
  } else if (op2 && lua_type(L, 1) == LUA_TNUMBER) {
    n = op2->size();
    *scalar = lua_tonumber(L, 1);
  } else {
    LuaError(L, "Operands must be two vectors of the same size or a vector "
                "and a scalar");
  }
  LuaVector *result = LuaUserClassCreateObj<LuaVector>(L);
  result->resize(n);
  *op1_ret = op1;
  *op2_ret = op2;
  return result;
}

void LuaVector::resize(int vector_size) {
  v_.resize(vector_size);
  v_.setZero();
}

int LuaVector::Index(lua_State *L) {
  CHECK(lua_gettop(L) >= 1);
  if (lua_type(L, -1) == LUA_TSTRING) {
    const char *s = lua_tostring(L, -1);
    if (strcmp(s, "Resize") == 0) {
      lua_pushcfunction(L, (LuaUserClassStub<LuaVector,
                                             &LuaVector::LuaResize>));
    } else {
      LuaError(L, "Unknown index to vector: '%s'", s);
    }
  } else if (lua_type(L, -1) == LUA_TNUMBER) {
    JetNum jindex = lua_tonumber(L, -1);
    int iindex = ToInt64(jindex);
    if (iindex != jindex || iindex < 1 || iindex > v_.size()) {
      LuaError(L, "Vector index out of range or not an integer "
               "(is %g, expecting 1..%d)", ToDouble(jindex), int(v_.size()));
    }
    lua_pushnumber(L, v_[iindex - 1]);
  } else {
    LuaError(L, "The vector must be indexed with an integer or function name, "
             "not %s", luaL_typename(L, -1));
  }
  return 1;
}

int LuaVector::NewIndex(lua_State *L) {
  CHECK(lua_gettop(L) >= 2);
  JetNum jindex = luaL_checknumber(L, -2);
  int iindex = ToInt64(jindex);
  if (iindex != jindex || iindex < 1 || iindex > v_.size()) {
    LuaError(L, "Vector index out of range (is %g, expecting 1..%d)",
             ToDouble(jindex), int(v_.size()));
  }
  v_[iindex - 1] = luaL_checknumber(L, -1);
  return 0;
}

int LuaVector::Length(lua_State *L) {
  lua_pushnumber(L, v_.size());
  return 1;
}

bool LuaVector::Operator(lua_State *L, int op, int pos) {
  // Handle unary operations.
  if (op == LUA_OPUNM) {
    LuaVector *op = LuaCastTo<LuaVector>(L, 1);
    CHECK(op);
    LuaVector *result = LuaUserClassCreateObj<LuaVector>(L);
    result->v_.resize(op->v_.size());
    result->v_ = -op->v_;
    return true;
  }

  // Handle binary operators with vector or scalar arguments. We would like the
  // ==,<,<= operators to return vectors of 0 or 1 but the result of those
  // operators is always converted to a boolean, so we have separete functions
  // in the 'vec' table for them instead.
  LuaVector *op1, *op2;
  JetNum scalar;
  LuaVector *result = BinaryOperatorArguments(L, &op1, &op2, &scalar);
  int n = result->v_.size();
  switch (op) {
    #define BINOP(op,fn) \
      if (!op1) { \
        for (int i = 0; i < n; i++) result->v_[i] = fn(scalar op op2->v_[i]); \
      } else if (!op2) { \
        for (int i = 0; i < n; i++) result->v_[i] = fn(op1->v_[i] op scalar); \
      } else { \
        for (int i = 0; i < n; i++) result->v_[i]=fn(op1->v_[i] op op2->v_[i]);\
      } \
      return true;
    case LUA_OPADD: BINOP(+, )
    case LUA_OPSUB: BINOP(-, )
    case LUA_OPMUL: BINOP(*, )
    case LUA_OPDIV: BINOP(/, )
    case LUA_OPPOW: BINOP(COMMA, pow)
    #undef BINOP
    default:
      // For unhandled operators the result vector on the stack will eventually
      // be garbage collected.
      return false;
  }
}

int LuaVector::LuaResize(lua_State *L) {
  if (lua_gettop(L) != 2) {
    LuaError(L, "vector:Resize() expecting 1 argument");
  }
  JetNum n = luaL_checknumber(L, 2);
  int nn = ToInt64(n);
  if (nn != n || nn < 0) {
    LuaError(L, "Argument to resize must be an integer >= 0");
  }
  resize(nn);
  return 0;
}

void LuaVector::SetLuaGlobals(lua_State *L) {
  lua_newtable(L);
  #define MATHFUNC1(name) \
    lua_pushcfunction(L, [](lua_State *L) -> int { \
      if (lua_gettop(L) != 1) { \
        LuaError(L, #name "() expecting 1 argument"); \
      } \
      LuaVector *arg = LuaCastTo<LuaVector>(L, 1); \
      if (!arg) { \
        LuaError(L, #name "() expecting 1 vector argument"); \
      } \
      LuaVector *result = LuaUserClassCreateObj<LuaVector>(L); \
      int n = arg->v_.size(); \
      result->v_.resize(n); \
      for (int i = 0; i < n; i++) { \
        result->v_[i] = name(arg->v_[i]); \
      } \
      return 1; \
    }); \
    LuaRawSetField(L, -2, #name);

  MATHFUNC1(abs)
  MATHFUNC1(acos)
  MATHFUNC1(asin)
  MATHFUNC1(atan)
  MATHFUNC1(ceil)
  MATHFUNC1(cos)
  MATHFUNC1(exp)
  MATHFUNC1(floor)
  MATHFUNC1(log)
  MATHFUNC1(log10)
  MATHFUNC1(sin)
  MATHFUNC1(sqrt)
  MATHFUNC1(tan)

  #define MATHFUNC2(name, fn, op) \
    lua_pushcclosure(L, [](lua_State *L) -> int { \
      if (lua_gettop(L) != 2) { \
        LuaError(L, #name "() expecting 2 arguments"); \
      } \
      LuaVector *op1, *op2; \
      JetNum scalar; \
      LuaVector *result = BinaryOperatorArguments(L, &op1, &op2, &scalar); \
      int n = result->v_.size(); \
      if (!op1) { \
        for (int i = 0; i < n; i++) result->v_[i] = fn(scalar op op2->v_[i]); \
      } else if (!op2) { \
        for (int i = 0; i < n; i++) result->v_[i] = fn(op1->v_[i] op scalar); \
      } else { \
        for (int i = 0; i < n; i++) \
          result->v_[i] = fn(op1->v_[i] op op2->v_[i]); \
      } \
      return 1; \
    }, 0); \
    LuaRawSetField(L, -2, #name);

  MATHFUNC2(fmod, fmod, COMMA)
  MATHFUNC2(atan2, atan2, COMMA);
  MATHFUNC2(eq, , ==);
  MATHFUNC2(ne, , !=);
  MATHFUNC2(lt, , <);
  MATHFUNC2(gt, , >);
  MATHFUNC2(le, , <=);
  MATHFUNC2(ge, , >=);

  lua_setglobal(L, "vec");
}
