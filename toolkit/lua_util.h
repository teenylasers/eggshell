// Interfaces between Lua and C++.

#ifndef __TOOLKIT_LUA_UTIL_H__
#define __TOOLKIT_LUA_UTIL_H__

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "error.h"
#include <string>
#include <map>

// This must be defined externally. It is called when the lua engine encounters
// an unrecoverable error and the program must be aborted.
void LuaPanic(const char *message) __attribute__((noreturn));

// Utility: like lua_getglobal() but do a raw get on the globals table,
// bypassing any metatable functions.
int LuaRawGetGlobal(lua_State *L, const char *name);

// Utility: like luaL_error() but does not prefix with the luaL_where()
// information (useful if the stack backtrace contains that information). Also
// supports the complete set of printf() specifiers, unlike the lua formatted
// string functions.
int LuaError(lua_State *L, const char *fmt, ...)
  __attribute__((format(printf, 2, 3), noreturn));

// Utility: like lua_setfield but does a rawset.
void LuaRawSetField(lua_State *L, int index, const char *k);

// Utility: convert the given stack entry into a string. This extends
// lua_tostring by working on all standard lua data types.
const char *LuaToString(lua_State *L, int index);

// Base class for user data objects in the lua environment.
class LuaUserClass {
 public:
  virtual ~LuaUserClass();

  // These functions are called directly from various metamethods, e.g. Index()
  // is called by __index and should pop a key (such as a function name string)
  // from the stack and push the corresponding value, or call LuaError() if
  // there is no such value. The default behavior of these functions is to
  // raise an error.
  virtual int Index(lua_State *L);                      // Called by __index
  virtual int NewIndex(lua_State *L);                   // Called by __newindex
  virtual int FunctionCall(lua_State *L);               // Called by __call
  virtual int Length(lua_State *L);                     // Called by __len

  // Handle lua unary or binary operators. The operands to use are at stack
  // positions 1 and 2. For unary operators the second operand is a copy of the
  // first one. The operation to perform is specified by 'op' which is a
  // LUA_OPxxx constant (either an arithmetic or comparison operator). The
  // 'pos' value is 1 or 2 and indicates which stack value corresponds to this
  // object. Return true on success or false if the operation is not supported
  // for this class for the given arguments and position. If true is returned
  // then the result must be left in a single entry at the top of the stack. If
  // false is returned then the other argument of a binary operator will be
  // tried. If no operands of an operator support the operations then a runtime
  // error results.
  virtual bool Operator(lua_State *L, int op, int pos);
};

// Convenience class for running lua scripts.
class Lua {
 public:
  // Create the lua script context.
  Lua();

  // Destroy the lua script context.
  virtual ~Lua();

  // Get the script context.
  lua_State *L() const { return L_; }

  // Use the lua standard libraries. If safe is true then only use the
  // libraries that are sandboxed, i.e. no I/O, OS, coroutines or debug
  // libraries. Add a few extra functions: math.log10.
  void UseStandardLibraries(bool safe);

  // Load (and optionally run) a lua script, either directly from a string or
  // from a file. Return true on success or false if there was an error. On any
  // runtime error the StackBacktrace() function will be called zero or more
  // times. For all errors the Error() function will be called once. If run_it
  // is false then the script is not run and the compiled chunk is left at the
  // top of the stack.
  bool RunString(const std::string &script, bool run_it = true);
  bool RunFile(const std::string &filename, bool run_it = true);

  // Like lua_pcall but using the built-in message handler for errors.
  int PCall(int nargs, int nresults);

  // Given a function on the top of the lua stack, dump it as a binary chunk to
  // the string 's'. If strip is true then the binary representation is created
  // without debug information about the function. This does not pop the
  // function from the stack. Functions with the same implementation will
  // return the same dump. However the upvalues for the function are not
  // represented, so two functions with the same dump called with the same
  // arguments might still return different results.
  void Dump(std::string *s, bool strip);

  // Given a function on the top of the lua stack, return the MD5 hash of its
  // Dump() in 'hash'. A binary string is returned that may contain embedded
  // zeros. The strip argument is given to Dump(). This does not pop the
  // function from the stack. See the caveat for the Dump() function.
  void Hash(std::string *hash, bool strip);

  // Call these functions to display error messages or stack backtraces. These
  // differ from the Handle*() variants only in that they set the
  // ThereWereErrors() flag.
  void StackBacktrace(const char *message) {
    there_were_errors_ = true;
    HandleStackBacktrace(message);
  }
  void Error(const char *message) {
    there_were_errors_ = true;
    HandleError(message);
  }

  // Functions defined by the user to handle the display of error messages.
  virtual void HandleStackBacktrace(const char *message) = 0;
  virtual void HandleError(const char *message) = 0;

  // Return true if any of the error functions (Error, StackBacktrace) have
  // been called since the last run function (RunString, RunFile).
  bool ThereWereErrors() const { return there_were_errors_; }

  // Functions defined by the user to handle the output of print(). By default
  // this prints to stdout.
  virtual int Print();

  // Set and get pointers to user objects. This is used by LuaGlobalStub2() to
  // access objects to call functions on. Note that this mechanism is not type
  // safe.
  void SetUserObject(int index, void *o) { user_objects_[index] = o; }
  void *GetUserObject(int index) { return user_objects_[index]; }

 private:
  lua_State *L_;
  bool there_were_errors_;
  std::map<int, void*> user_objects_;

  bool Run(const std::string &s, bool s_is_filename, bool run_it);

  DISALLOW_COPY_AND_ASSIGN(Lua);
};

// Get the 'class Lua'-derived object address corresponding to the lua state.
Lua *LuaGetObject(lua_State *L);

// A stub that allows calling a 'class Lua'-derived member function from a
// global lua function.
template<class T, int (T::*fn)()> int LuaGlobalStub(lua_State *L) {
  T *obj = dynamic_cast<T*>(LuaGetObject(L));
  if (!obj) {
    LuaPanic("Bad lua context in LuaGlobalStub");
  }
  return (obj->*fn)();
}

// Like LuaGlobalStub() but allow T::fn to be called for the user object at
// 'index'. Note that this mechanism is not type safe.
template<class T, int (T::*fn)(lua_State *), int index>
int LuaGlobalStub2(lua_State *L) {
  Lua *lua = LuaGetObject(L);
  if (!lua) {
    LuaPanic("Bad lua context in LuaGlobalStub2");
  }
  T *obj = reinterpret_cast<T*>(lua->GetUserObject(index));
  if (!obj) {
    LuaPanic("Bad object index in LuaGlobalStub2");
  }
  return (obj->*fn)(L);
}

// Create a new instance of the LuaUserClass-derived class T and leave the
// corresponding userdata at the top of the lua stack. This comes in separate
// C- and lua-callable versions that differ in their return type, but note that
// in both cases it is lua that owns the memory.
template<class T> T *LuaUserClassCreateObj(lua_State *L) {
  // Create a new userdata.
  void *mem = lua_newuserdata(L, sizeof(T));
  T* obj = new (mem) T();

  // Set the standard LuaUserClass metatable. Object is at the stack top.
  extern char lua_registry_metatable_key;
  if (lua_rawgetp(L, LUA_REGISTRYINDEX, &lua_registry_metatable_key) !=
      LUA_TTABLE) {
    LuaPanic("lua_registry_metatable_key missing");
  }
  lua_setmetatable(L, -2);
  return obj;
}
template<class T> int LuaUserClassCreate(lua_State *L) {
  LuaUserClassCreateObj<T>(L);
  return 1;
}

// Register class T with the lua environment. A new instance of this class can
// be created in lua with: foo = name().
template<class T> void LuaUserClassRegister(const Lua &lua, const char *name) {
  // Create a global constructor function called 'name' that will return a new
  // instance of this class.
  lua_pushcfunction(lua.L(), LuaUserClassCreate<T>);
  lua_setglobal(lua.L(), name);
}

// A lua C function that when called with a LuaUserClass userdata as its first
// argument calls a corresponding function in the LuaUserClass object.
template<class T, int (T::*fn)(lua_State*)> int LuaUserClassStub(lua_State *L) {
  if (lua_gettop(L) >= 1 && lua_type(L, 1) == LUA_TUSERDATA) {
    T *obj = (T*) lua_topointer(L, 1);
    return (obj->*fn)(L);
  } else {
    LuaError(L, "Bad function invocation, call this as foo:bar()");
  }
  return 0;
}

// If the index'th stack entry is an instance of class T (which is derived from
// LuaUserClass) then return a pointer to it, otherwise return 0.
template<class T> T *LuaCastTo(lua_State *L, int index) {
  if (lua_gettop(L) >= index && lua_type(L, index) == LUA_TUSERDATA) {
    // All userdata is assumed to be derived from LuaUserClass.
    LuaUserClass *obj = (LuaUserClass*) lua_topointer(L, index);
    if (obj) {
      return dynamic_cast<T*>(obj);
    }
  }
  return 0;
}

#endif
