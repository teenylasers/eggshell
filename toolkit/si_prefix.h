
#ifndef __TOOLKIT_SI_PREFIX_H__
#define __TOOLKIT_SI_PREFIX_H__

// Return an SI prefix character for the number 'n', and an associated scale,
// such that the number can be displayed as 'n/scale <character>'. The
// precision is the number of decimal places that will be displayed, e.g. for
// 3dp the precision is 0.999. Specifying the precision allows numbers that are
// slightly smaller than the prefix threshold to take that prefix when they are
// rounded up during display.

inline char SIPrefix(double n, double precision, double *scale) {
  n = (n < 0) ? -n : n;                 // Absolute value
  double k = 0.5 + 0.5*precision;       // precision = 0.999 --> k = 0.9995
  if (n >= 1e24  * k) { *scale = 1e24 ; return 'Y'; }   // yotta
  if (n >= 1e21  * k) { *scale = 1e21 ; return 'Z'; }   // zetta
  if (n >= 1e18  * k) { *scale = 1e18 ; return 'E'; }   // exa
  if (n >= 1e15  * k) { *scale = 1e15 ; return 'P'; }   // peta
  if (n >= 1e12  * k) { *scale = 1e12 ; return 'T'; }   // tera
  if (n >= 1e9   * k) { *scale = 1e9  ; return 'G'; }   // giga
  if (n >= 1e6   * k) { *scale = 1e6  ; return 'M'; }   // mega
  if (n >= 1e3   * k) { *scale = 1e3  ; return 'k'; }   // kilo
  if (n >= 1     * k) { *scale = 1    ; return ' '; }   //
  if (n >= 1e-3  * k) { *scale = 1e-3 ; return 'm'; }   // milli
  if (n >= 1e-6  * k) { *scale = 1e-6 ; return 'u'; }   // micro
  if (n >= 1e-9  * k) { *scale = 1e-9 ; return 'n'; }   // nano
  if (n >= 1e-12 * k) { *scale = 1e-12; return 'p'; }   // pico
  if (n >= 1e-15 * k) { *scale = 1e-15; return 'f'; }   // femto
  if (n >= 1e-18 * k) { *scale = 1e-18; return 'a'; }   // atto
  if (n >= 1e-21 * k) { *scale = 1e-21; return 'z'; }   // zepto
  if (n >= 1e-24 * k) { *scale = 1e-24; return 'y'; }   // yocto
  *scale = 1;
  return ' ';
}

#endif
