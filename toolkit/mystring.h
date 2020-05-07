// Copyright (C) 2014-2020 Russell Smith.
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
