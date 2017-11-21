
#include "lua_util.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "md5.h"

// The addresses of these object are unique registry keys.
char lua_registry_metatable_key;     // LuaUserClass shared metatable

// Handle panics from within the lua interpreter.
static int LuaPanicFunction(lua_State *L) {
  LuaPanic(lua_tostring(L, -1));
}

// Convert a lua error code into a string.
static const char *LuaStrerror(int err) {
  switch (err) {
    case LUA_OK: return "No error";
    case LUA_ERRRUN: return "Script could not run";
    case LUA_ERRSYNTAX: return "Script syntax error";
    case LUA_ERRMEM: return "Memory allocation error";
    case LUA_ERRGCMM: return "Script error while running a __gc metamethod";
    case LUA_ERRERR: return "Script error while running the message handler";
    case LUA_ERRFILE: return "Script file can not be read";
    default: return "Unknown error while running Lua";
  }
}

static int LuaLog10(lua_State *L) {
  lua_pushnumber(L, log10(luaL_checknumber(L, 1)));
  return 1;
}

int LuaRawGetGlobal(lua_State *L, const char *name) {
  lua_pushinteger(L, LUA_RIDX_GLOBALS);                   // index
  CHECK(lua_rawget(L, LUA_REGISTRYINDEX) == LUA_TTABLE);  // globals
  lua_pushstring(L, name);                                // globals name
  int type = lua_rawget(L, -2);                           // globals value
  lua_remove(L, -2);                                      // value
  return type;
}

int LuaError(lua_State *L, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);

  #if defined(__WXMSW__) || defined(Q_OS_WIN)
    // Windows specific:
    int count = _vscprintf(fmt, ap) + 1;
    char *buffer = new char[count];
    _vsnprintf(buffer, count, fmt, ap);
    lua_pushstring(L, buffer);
    delete[] buffer;
  #else
    // Linux:
    char *strp = 0;
    CHECK(vasprintf(&strp, fmt, ap) != -1);
    lua_pushstring(L, strp);
    free(strp);
  #endif

  // A lua version that does not support all the printf() specifiers:
  //   lua_pushvfstring(L, fmt, ap);

  va_end(ap);
  lua_error(L);
  // lua_error doesn't return, this is just to satisfy the noreturn attribute:
  abort();
}

void LuaRawSetField(lua_State *L, int index, const char *k) {
  lua_pushstring(L, k);
  lua_rotate(L, -2, 1);         // Swap top 2 elements
  lua_rawset(L, (index >= 0) ? index : (index - 1));
}

const char *LuaToString(lua_State *L, int index) {
  switch (lua_type(L, index)) {
    case LUA_TNIL:
      return "nil";
    case LUA_TBOOLEAN:
      return lua_toboolean(L, index) ? "true" : "false";
    case LUA_TNUMBER:
    case LUA_TSTRING:
      return lua_tostring(L, index);
    default:
      return lua_typename(L, lua_type(L, index));
  }
}

Lua *LuaGetObject(lua_State *L) {
  Lua *p = *(Lua**) lua_getextraspace(L);
  CHECK(p);
  return p;
}

// A lua error function for lua_pcall() that generates a stack backtrace.

static int LuaErrorFunction(lua_State *L) {
  Lua *lua = LuaGetObject(L);
  lua_Debug dbg;

  for (int level = 1; lua_getstack(L, level, &dbg); level++) {
    CHECK(lua_getinfo(L, "nSl", &dbg) != 0);
    char buffer[1000];
    if (dbg.name) {
      snprintf(buffer, sizeof(buffer), "In %s:%d (%s %s)",
               dbg.short_src, dbg.currentline, dbg.namewhat, dbg.name);
    } else {
      snprintf(buffer, sizeof(buffer), "In main script line %d",
               dbg.currentline);
    }
    lua->StackBacktrace(buffer);
  }

  // Return error message argument given to this function.
  return 1;
}

// Called when an undefined global is referenced. Arguments are the globals
// table and the name of the key that is not present.

static int LuaUndefinedGlobal(lua_State *L) {
  CHECK(lua_gettop(L) == 2);
  LuaError(L, "Unknown global variable '%s'", lua_tostring(L, 2));
  return 0;
}

// Replacement for the standard print() function that calls Lua::Print().

static int LuaPrint(lua_State *L) {
  return LuaGetObject(L)->Print();
}

//***************************************************************************
// LuaUserClass and support functions.

LuaUserClass::~LuaUserClass() {
}

int LuaUserClass::Index(lua_State *L) {
  LuaError(L, "object[index] operation not supported");
}

int LuaUserClass::NewIndex(lua_State *L) {
  LuaError(L, "object[index]=value operation not supported");
}

int LuaUserClass::FunctionCall(lua_State *L) {
  LuaError(L, "object(arguments) function call operation not supported");
}

int LuaUserClass::Length(lua_State *L) {
  LuaError(L, "#object operation not supported");
}

bool LuaUserClass::Operator(lua_State *L, int op, int pos) {
  return false;
}

// These functions are called from various lua metamethods for userdata
// objects.

static int LuaUserClassIndex(lua_State *L) {
  CHECK(lua_gettop(L) == 2);
  CHECK(lua_type(L, 1) == LUA_TUSERDATA);
  return ((LuaUserClass *) lua_topointer(L, 1))->Index(L);
}

static int LuaUserClassNewIndex(lua_State *L) {
  CHECK(lua_gettop(L) == 3);
  CHECK(lua_type(L, 1) == LUA_TUSERDATA);
  return ((LuaUserClass *) lua_topointer(L, 1))->NewIndex(L);
}

static int LuaUserClassFunctionCall(lua_State *L) {
  CHECK(lua_gettop(L) >= 1);
  CHECK(lua_type(L, 1) == LUA_TUSERDATA);
  return ((LuaUserClass *) lua_topointer(L, 1))->FunctionCall(L);
}

static int LuaUserClassLength(lua_State *L) {
  // Both arguments are the same userdata in lua 5.3.0.
  CHECK(lua_gettop(L) == 2);
  CHECK(lua_type(L, 1) == LUA_TUSERDATA);
  return ((LuaUserClass *) lua_topointer(L, 1))->Length(L);
}

// This is called from lua overloaded operators. The arguments are one or two
// userdatas for the LuaUserClass object.
template <int op> static int LuaUserClassOperator(lua_State *L) {
  LuaUserClass *obj[2] = {0, 0};
  // Lua 5.3 will pass a dummy second operand for unary operators (that is
  // equal to the first operand), but the Lua manual says this internal feature
  // may be removed in future versions. If that's ever the case the following
  // check will fail and we should add the second operand manually, as
  // LuaUserClass::Operator() expects it.
  CHECK(lua_gettop(L) == 2);
  for (int i = 0; i < 2; i++) {
    if (lua_type(L, i + 1) == LUA_TUSERDATA) {
      obj[i] = (LuaUserClass *) lua_topointer(L, i + 1);
    }
  }
  if (obj[0] && obj[0]->Operator(L, op, 1)) {
    return 1;
  } else if (obj[1] && obj[1] != obj[0] && obj[1]->Operator(L, op, 2)) {
    return 1;
  } else {
    LuaError(L, "This operator is not supported");
    return 0;
  }
}

// This is called by the lua garbage collector. The arguments are:
//   * A userdata for the LuaUserClass object.
// This calls the destructor of the LuaUserClass object, but does not free the
// memory as lua will do that.

static int LuaUserClassGarbageCollector(lua_State *L) {
  CHECK(lua_gettop(L) == 1);
  CHECK(lua_type(L, 1) == LUA_TUSERDATA);
  LuaUserClass *obj = (LuaUserClass *) lua_topointer(L, 1);
  obj->~LuaUserClass();
  return 0;
}

//***************************************************************************
// Lua.

Lua::Lua() {
  // Stock lua defines LUA_OP{EQ,LT,LE} to be the same as LUA_OP{ADD,SUB,MUL},
  // which is a problem below because we would like to use all the operator
  // constants in a single interface. Check here that whatever lua library we
  // are using has redefined these constants appropriately.
  static_assert(LUA_OPEQ != LUA_OPADD, "LUA_OPEQ value should be changed");
  static_assert(LUA_OPLT != LUA_OPSUB, "LUA_OPLT value should be changed");
  static_assert(LUA_OPLE != LUA_OPMUL, "LUA_OPLE value should be changed");
  static_assert(LUA_OPEQ == LUA_OPBNOT+1, "LUA_OPEQ value should be changed");
  static_assert(LUA_OPEQ == 14, "LUA_OPEQ value should be changed");
  static_assert(LUA_OPLT == 15, "LUA_OPLT value should be changed");
  static_assert(LUA_OPLE == 16, "LUA_OPLE value should be changed");

  L_ = luaL_newstate();
  there_were_errors_ = false;
  lua_atpanic(L_, LuaPanicFunction);
  CHECK(lua_gettop(L_) == 0);

  // Set a pointer back to this object in the Lua state.
  *(Lua**) lua_getextraspace(L_) = this;

  // Create the shared metatable used by all LuaUserClass-based objects. This
  // provides table-like indexing, garbage collection and operator overloading.
  lua_newtable(L_);                                       // T
  #define MAKEOP(meta_name, fn) \
    lua_pushstring(L_, meta_name); \
    lua_pushcfunction(L_, fn); \
    lua_rawset(L_, -3);
  MAKEOP("__index", LuaUserClassIndex)
  MAKEOP("__newindex", LuaUserClassNewIndex)
  MAKEOP("__call", LuaUserClassFunctionCall)
  MAKEOP("__len", LuaUserClassLength)
  MAKEOP("__gc", LuaUserClassGarbageCollector)
  MAKEOP("__add",  LuaUserClassOperator<LUA_OPADD>)
  MAKEOP("__sub",  LuaUserClassOperator<LUA_OPSUB>)
  MAKEOP("__mul",  LuaUserClassOperator<LUA_OPMUL>)
  MAKEOP("__div",  LuaUserClassOperator<LUA_OPDIV>)
  MAKEOP("__idiv", LuaUserClassOperator<LUA_OPIDIV>)
  MAKEOP("__mod",  LuaUserClassOperator<LUA_OPMOD>)
  MAKEOP("__pow",  LuaUserClassOperator<LUA_OPPOW>)
  MAKEOP("__unm",  LuaUserClassOperator<LUA_OPUNM>)
  MAKEOP("__bnot", LuaUserClassOperator<LUA_OPBNOT>)
  MAKEOP("__band", LuaUserClassOperator<LUA_OPBAND>)
  MAKEOP("__bor",  LuaUserClassOperator<LUA_OPBOR>)
  MAKEOP("__bxor", LuaUserClassOperator<LUA_OPBXOR>)
  MAKEOP("__shl",  LuaUserClassOperator<LUA_OPSHL>)
  MAKEOP("__shr",  LuaUserClassOperator<LUA_OPSHR>)
  MAKEOP("__eq",   LuaUserClassOperator<LUA_OPEQ>)
  MAKEOP("__lt",   LuaUserClassOperator<LUA_OPLT>)
  MAKEOP("__le",   LuaUserClassOperator<LUA_OPLE>)
  #ifdef LUA_OPCONCAT           // LUA_OPCONCAT may have been added to lua.h
    MAKEOP("__concat", LuaUserClassOperator<LUA_OPCONCAT>)
  #endif
  #undef MAKEOP

  lua_rawsetp(L_, LUA_REGISTRYINDEX, &lua_registry_metatable_key);

  // Install the __index function in a new global metatable to cause an error
  // when undefined globals are referenced.
  lua_pushinteger(L_, LUA_RIDX_GLOBALS);                   // index
  CHECK(lua_rawget(L_, LUA_REGISTRYINDEX) == LUA_TTABLE);  // globals
  CHECK(lua_getmetatable(L_, -1) == 0);
  lua_newtable(L_);                            // globals T
  lua_pushstring(L_, "__index");               // globals T __index
  lua_pushcfunction(L_, LuaUndefinedGlobal);   // globals T __index fn
  lua_rawset(L_, -3);                          // globals T
  lua_setmetatable(L_, -2);                    // globals

  // Pop everything off the stack.
  lua_settop(L_, 0);
}

Lua::~Lua() {
  // Delete the lua state and also (via __gc) destruct all userdata objects
  // that are based on LuaUserClass.
  lua_close(L_);
}

void Lua::UseStandardLibraries(bool safe) {
  int top = lua_gettop(L_);
  if (safe) {
    luaopen_base(L_);   lua_setglobal(L_, "base");
    luaopen_table(L_);  lua_setglobal(L_, "table");
    luaopen_string(L_); lua_setglobal(L_, "string");
    luaopen_utf8(L_);   lua_setglobal(L_, "utf8");
    luaopen_math(L_);   lua_setglobal(L_, "math");
  } else {
    luaL_openlibs(L_);
  }

  lua_register(L_, "print", LuaPrint);

  LuaRawGetGlobal(L_, "math");
  lua_pushcfunction(L_, LuaLog10);
  LuaRawSetField(L_, -2, "log10");
  lua_settop(L_, top);
}

bool Lua::RunString(const std::string &script, bool run_it) {
  return Run(script, false, run_it);
}

bool Lua::RunFile(const std::string &filename, bool run_it) {
  return Run(filename, true, run_it);
}

int Lua::Print() {
  int n = lua_gettop(L_);
  for (int i = 1; i <= n; i++) {
    fputs(LuaToString(L_, i), stdout);
    if (i < n) {
      fputc(' ', stdout);
    }
  }
  fputc('\n', stdout);
  return 0;
}

static int StringWriter(lua_State *L, const void *p, size_t sz, void *ud) {
  std::string *buffer = (std::string *) ud;
  buffer->append((const char *) p, sz);
  return 0;
}

void Lua::Dump(std::string *s, bool strip) {
  CHECK(lua_gettop(L_) >= 1);
  CHECK(lua_type(L_, -1) == LUA_TFUNCTION);
  s->clear();
  lua_dump(L_, StringWriter, s, strip);
}

void Lua::Hash(std::string *hash, bool strip) {
  std::string dump;
  Dump(&dump, strip);
  md5_state_t ms;
  md5_init(&ms);
  md5_append(&ms, (const md5_byte_t*) dump.data(), dump.size());
  md5_byte_t digest[16];
  md5_finish(&ms, digest);
  hash->assign((char*) digest, sizeof(digest));
}

bool Lua::Run(const std::string &s, bool s_is_filename, bool run_it) {
  there_were_errors_ = false;
  int top = lua_gettop(L_);

  // Load and compile the script.
  int err = LUA_OK;
  if (s_is_filename) {
    err = luaL_loadfile(L_, s.c_str());
  } else {
    err = luaL_loadbuffer(L_, s.data(), s.size(), NULL);
  }
  if (err != LUA_OK) {
    char buffer[1000];
    snprintf(buffer, sizeof(buffer), "%s:\n%s",
             LuaStrerror(err), lua_tostring(L_, -1));
    Error(buffer);
    return false;
  }

  // Now the compiled chunk is at the top of the stack. Optionally run it.
  if (run_it) {
    err = PCall(0, LUA_MULTRET);
    lua_settop(L_, top);
  }
  return err == LUA_OK;
}

int Lua::PCall(int nargs, int nresults) {
  // The caller pushed a function and 'nargs' stack elements. We want to place
  // the message handler function before them.
  int n = lua_gettop(L_) - nargs;
  CHECK(n >= 1);
  CHECK(lua_type(L_, n) == LUA_TFUNCTION);
  lua_pushcfunction(L_, LuaErrorFunction);
  lua_insert(L_, n);
  // Call the function, then remove the message handler function from the
  // stack.
  int error = lua_pcall(L_, nargs, nresults, n);
  lua_remove(L_, n);

  // Handle any errors (the error handler may have already output stack
  // backtrace information).
  if (error != LUA_OK) {
    if (error != LUA_ERRRUN) {
      Error(LuaStrerror(error));
    }
    Error(lua_tostring(L_, -1));
  }
  return error;
}
