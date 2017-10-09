
// Various simple platform differences.

#ifndef __TOOLKIT_PLATFORM_H__
#define __TOOLKIT_PLATFORM_H__

// Printf format specifier for size_t
#ifdef __WXMSW__
  #define PRINTF_SIZET "%Id"
#else
  #define PRINTF_SIZET "%zd"
#endif

// Special character strings.
#ifdef __APPLE__
#define DEGREE_SYMBOL "\xc2\xb0"        // UTF-8 encoding
#else
#define DEGREE_SYMBOL "\xb0"
#endif

// How to scale wx fonts so they look roughly the same size.
#ifdef __APPLE__
#define FONT_SCALE 1.8
#else
#define FONT_SCALE 1.0
#endif

#endif
