#ifndef HSL_H
#define HSL_H

//
// Utilities for working with colors in different color spaces
//

#include "mbed.h"

// all values 0 to 1

// adapted from: https://github.com/ratkins/RGBConverter

struct FloatRGB {
  float r;
  float g;
  float b;
};

struct FloatHSL {
  float h;
  float s;
  float l;
};

struct FloatHSV {
  float h;
  float s;
  float v;
};

// --- Converting to RGB hex ---

/** Convert HSL to RGB hex */
uint32_t hsl2hex(const FloatHSL* hsl);

/** Convert HSV to RGB hex */
uint32_t hsv2hex(const FloatHSV* hsv);

/** Convert RGB to RGB hex */
uint32_t rgb2hex(FloatRGB* rgb);

// --- Itner-space conversion functions ---

/** Convert HSL to RGB */
void hsl2rgb(const FloatHSL* hsl, FloatRGB* rgb);

/** Convert RGB to HSL */
void rgb2hsl(const FloatRGB* rgb, FloatHSL* hsl);

/** Convert from HSV to HSL */
void hsv2hsl(const FloatHSV* hsv, FloatHSL* hsl);

/** Convert from HSL to HSV */
void hsl2hsv(const FloatHSL* hsl, FloatHSV* hsv);

/** Convert HSV to RGB ("handy" algo) */
void hsv2rgb(const FloatHSV* hsv, FloatRGB* rgb);

#endif /* HSL_H */