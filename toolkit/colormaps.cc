
#include <math.h>
#include <algorithm>
#include "colormaps.h"

namespace ColorMap {

// A function that is: 1 for |x| < 1/8
//                     0 for |x| > 3/8
//                     Linear ramp in between
static inline float blerp1(float x) {
  return std::min(1.0f, std::max(0.0f, -4*fabsf(x) + 1.5f));
}

// A function that is: 1 for |x| < 1/6
//                     0 for |x| > 2/3
//                     Linear ramp in between
static inline float blerp2(float x) {
  return std::min(1.0f, std::max(0.0f, -6*fabsf(x) + 2.0f));
}

void Jet(float x, float rgb[3]) {
  x = std::min(1.0f, std::max(0.0f, x));
  rgb[0] = blerp1(x - 0.75f);
  rgb[1] = blerp1(x - 0.5f);
  rgb[2] = blerp1(x - 0.25f);
}

void Hot(float x, float rgb[3]) {
  rgb[0] = std::min(1.0f, std::max(0.0f, (8.0f / 3.0f) * x));
  rgb[1] = std::min(1.0f, std::max(0.0f, (8.0f / 3.0f) * (x - 0.375f)));
  rgb[2] = std::min(1.0f, std::max(0.0f, 4.0f * (x - 0.75f)));
}

void Gray(float x, float rgb[3]) {
  rgb[0] = std::min(1.0f, std::max(0.0f, x));
  rgb[1] = std::min(1.0f, std::max(0.0f, x));
  rgb[2] = std::min(1.0f, std::max(0.0f, x));
}

void HSV(float x, float rgb[3]) {
  x = std::min(1.0f, std::max(0.0f, x));
  rgb[0] = blerp2(x) + blerp2(1 - x);
  rgb[1] = blerp2(x - 1.0f / 3.0f);
  rgb[2] = blerp2(x - 2.0f / 3.0f);
}

void Bone(float x, float rgb[3]) {
  float hot[3], gray[3];
  Hot(x, hot);
  Gray(x, gray);
  rgb[0] = 0.87 * gray[0] + 0.13f * hot[2];
  rgb[1] = 0.87 * gray[1] + 0.13f * hot[1];
  rgb[2] = 0.87 * gray[2] + 0.13f * hot[0];
}

void Copper(float x, float rgb[3]) {
  rgb[0] = std::min(1.0f, std::max(0.0f, x * 1.3f));
  rgb[1] = std::min(1.0f, std::max(0.0f, x * 0.75f));
  rgb[2] = std::min(1.0f, std::max(0.0f, x * 0.5f));
}

void Wheel(float x, float rgb[3]) {
  Jet(fmodf(x * 8.0f, 1.0f), rgb);
}

void Wave(float x, float rgb[3]) {
  rgb[0] = std::min(1.0f, std::max(0.0f, 0.3f - 0.6f*x) +
           std::max(0.0f, (x - 0.5f)/0.3f));
  rgb[1] = std::min(0.8f, x*4/3) + std::max(0.0f, x - 0.8f);
  rgb[2] = std::min(1.0f, x + 0.8f) - fabsf(x - 0.2f)/0.9f;
}

}  // namespace ColorMap
