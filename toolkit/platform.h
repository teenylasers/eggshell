
// Various simple platform differences.

#ifndef __TOOLKIT_PLATFORM_H__
#define __TOOLKIT_PLATFORM_H__

// Printf format specifier for size_t
#if defined(__WINNT__)
  #define PRINTF_SIZET "%Id"
#else
  #define PRINTF_SIZET "%zd"
#endif

// Special character strings.
#if defined(__APPLE__) || defined(QT_CORE_LIB)
#define DEGREE_SYMBOL "\xc2\xb0"        // UTF-8 encoding
#else
#define DEGREE_SYMBOL "\xb0"
#endif

// How to scale fonts so they look roughly the same size on each platform.
#ifdef __APPLE__
#define FONT_SCALE 1.8
#else
#define FONT_SCALE 1.0
#endif

#endif
