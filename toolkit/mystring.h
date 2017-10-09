
#ifndef __TOOLKIT_MYSTRING_H__
#define __TOOLKIT_MYSTRING_H__

#include <stdarg.h>
#include <string>

// Versions of strtod() and strtol that accepts both leading and trailing
// whitespace characters. True is returned if conversion is successful.
bool StrToDouble(const char *s, double *value);
bool StrToInt(const char *s, int *value);

// Version of sprintf() that works with std::string.
void StringPrintf(std::string *buffer, const char *msg, ...)
  __attribute__((format(printf, 2, 3)));
void StringAppendF(std::string *buffer, const char *msg, ...)
  __attribute__((format(printf, 2, 3)));
void StringVPrintf(std::string *buffer, const char *msg, va_list ap);
void StringAppendV(std::string *buffer, const char *msg, va_list ap);

#endif
