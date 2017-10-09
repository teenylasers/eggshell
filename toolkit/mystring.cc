
#include <stdlib.h>
#include <errno.h>
#include "error.h"
#include "mystring.h"

bool StrToDouble(const char *s, double *value) {
  while (*s && isspace(*s)) {
    s++;
  }
  if (*s == 0) {
    return false;       // String was all whitespace
  }
  errno = 0;
  char *endptr = 0;
  *value = strtod(s, &endptr);
  if (errno != 0) {
    return false;       // Overflow or underflow
  }
  while (*endptr && isspace(*endptr)) {
    endptr++;
  }
  return *endptr == 0;
}

bool StrToInt(const char *s, int *value) {
  while (*s && isspace(*s)) {
    s++;
  }
  if (*s == 0) {
    return false;       // String was all whitespace
  }
  errno = 0;
  char *endptr = 0;
  long int lvalue = strtol(s, &endptr, 10);
  if (errno != 0) {
    return false;       // Overflow or underflow
  }
  *value = lvalue;
  if (*value != lvalue) {       // Result can't fit in integer
    return false;
  }
  while (*endptr && isspace(*endptr)) {
    endptr++;
  }
  return *endptr == 0;
}

void StringPrintf(std::string *buffer, const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  buffer->clear();
  StringAppendV(buffer, msg, ap);
}

void StringVPrintf(std::string *buffer, const char *msg, va_list ap) {
  buffer->clear();
  StringAppendV(buffer, msg, ap);
}

void StringAppendF(std::string *buffer, const char *msg, ...) {
  va_list ap;
  va_start(ap, msg);
  StringAppendV(buffer, msg, ap);
}

void StringAppendV(std::string *buffer, const char *msg, va_list ap) {
  #ifdef __WXMSW__
    // Windows specific:
    // We need separate copies of 'ap' for all users, as users will mutate it.
    va_list ap2;
    va_copy(ap2, ap);
    int count = _vscprintf(msg, ap) + 1;
    char *buf = new char[count];
    _vsnprintf(buf, count, msg, ap2);
    *buffer += buf;
    delete[] buf;
  #else
    // Linux, Mac:
    char *strp = 0;
    CHECK(vasprintf(&strp, msg, ap) != -1);
    *buffer += strp;
    free(strp);
  #endif
}
