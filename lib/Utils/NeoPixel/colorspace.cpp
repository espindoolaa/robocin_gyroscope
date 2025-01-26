#include "mbed.h"
#include "colorspace.h"
#include "math.h"

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) > (b) ? (b) : (a))

static float threeway_max(float a, float b, float c) {
  return max(a, max(b, c));
}

static float threeway_min(float a, float b, float c) {
  return min(a, min(b, c));
}

static float hue2rgb(float p, float q, float t) {
  if (t < 0)
    t += 1;
  if (t > 1)
    t -= 1;
  if (t < 1 / 6.0f)
    return p + (q - p) * 6 * t;
  if (t < 1 / 2.0f)
    return q;
  if (t < 2 / 3.0f)
    return p + (q - p) * (2 / 3.0f - t) * 6;
  return p;
}

void hsl2rgb(const FloatHSL* hsl, FloatRGB* rgb) {
  float h = hsl->h;
  float s = hsl->s;
  float l = hsl->l;

  if (s == 0) {
    rgb->r = rgb->g = rgb->b = l; // achromatic
  } else {
    float q = l < 0.5f ? l * (1 + s) : l + s - l * s;
    float p = 2 * l - q;
    rgb->r = hue2rgb(p, q, h + 1 / 3.0f);
    rgb->g = hue2rgb(p, q, h);
    rgb->b = hue2rgb(p, q, h - 1 / 3.0f);
  }
}

void hsv2rgb(const FloatHSV* hsv, FloatRGB* rgb) {
  float r = 0, g = 0, b = 0;

  int i = floor(hsv->h * 6);
  float f = hsv->h * 6 - i;
  float p = hsv->v * (1 - hsv->s);
  float q = hsv->v * (1 - f * hsv->s);
  float t = hsv->v * (1 - (1 - f) * hsv->s);

  switch (i % 6) {
    case 0:
      r = hsv->v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = hsv->v;
      b = p;
      break;
    case 2:
      r = p;
      g = hsv->v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = hsv->v;
      break;
    case 4:
      r = t;
      g = p;
      b = hsv->v;
      break;
    case 5:
      r = hsv->v;
      g = p;
      b = q;
      break;
  }

  rgb->r = r;
  rgb->g = g;
  rgb->b = b;
}

void rgb2hsl(const FloatRGB* rgb, FloatHSL* hsl) {
  float rd = rgb->r;
  float gd = rgb->g;
  float bd = rgb->b;
  float max = threeway_max(rd, gd, bd);
  float min = threeway_min(rd, gd, bd);
  float h = 0, s, l = (max + min) / 2.0f;

  if (max == min) {
    h = s = 0; // achromatic
  } else {
    float d = max - min;
    s = l > 0.5f ? d / (2 - max - min) : d / (max + min);
    if (max == rd) {
      h = (gd - bd) / d + (gd < bd ? 6 : 0);
    } else if (max == gd) {
      h = (bd - rd) / d + 2;
    } else if (max == bd) {
      h = (rd - gd) / d + 4;
    }
    h /= 6;
  }
  hsl->h = h;
  hsl->s = s;
  hsl->l = l;
}

/** Convert from HSV to HSL  (this is possibly wrong) */
void hsv2hsl(const FloatHSV* hsv, FloatHSL* hsl) {
  float l = (2 - hsv->s) * hsv->v / 2.0f;
  float s = hsv->s;

  if (l != 0) {
    if (l == 1) {
      s = 0;
    } else if (l < 0.5f) {
      s = s * hsv->v / (l * 2);
    } else {
      s = s * hsv->v / (2 - l * 2);
    }
  }

  hsl->h = hsv->h;
  hsl->s = s;
  hsl->l = l;
}

/** Convert from HSL to HSV */
void hsl2hsv(const FloatHSL* hsl, FloatHSV* hsv) {
  float sat = hsv->s * ((hsl->l < 0.5f) ? hsl->l : (1 - hsl->l));
  hsv->s = 2 * sat / (hsl->l + sat);
  hsv->h = hsl->h;
  hsv->v = hsl->l + sat;
}

uint32_t hsl2hex(const FloatHSL* hsl) {
  FloatRGB rgb;
  hsl2rgb(hsl, &rgb);
  return rgb2hex(&rgb);
}

uint32_t hsv2hex(const FloatHSV* hsv) {
  FloatRGB rgb;
  hsv2rgb(hsv, &rgb);
  return rgb2hex(&rgb);
}

uint32_t rgb2hex(FloatRGB* rgb) {
  return ((int) floor(rgb->r * 255 + 0.5f) << 16) | ((int) floor(rgb->g * 255 + 0.5f) << 8) |
         ((int) floor(rgb->b * 255 + 0.5f));
}
