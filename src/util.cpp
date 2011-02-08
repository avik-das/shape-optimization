#include "util.hpp"

#include <sstream>
#include <math.h>
#include <limits>

const double PI = 3.14159265;

const float F_INF = numeric_limits<float>::infinity();

// RGB ========================================================================

RGB::RGB(float r, float g, float b) : r(r), g(g), b(b) {}

RGB RGB::operator*(const float c) const { return RGB(r * c, g * c, b * c); }

RGB RGB::operator/(const float c) const { return RGB(r / c, g / c, b / c); }

RGB RGB::operator*(const RGB &rgb) const {
  return RGB(r * rgb.r, g * rgb.g, b * rgb.b);
}

RGB RGB::operator/(const RGB &rgb) const {
  return RGB(r / rgb.r, g / rgb.g, b / rgb.b);
}

RGB RGB::operator+(const RGB &rgb) const {
  return RGB(r + rgb.r, g + rgb.g, b + rgb.b);
}

RGB RGB::operator-(const RGB &rgb) const {
  return RGB(r - rgb.r, g - rgb.g, b - rgb.b);
}

RGB & RGB::operator*=(const float c) { r *= c; g *= c; b *= c; return *this; }

RGB & RGB::operator/=(const float c) { r /= c; g /= c; b /= c; return *this; }

RGB & RGB::operator+=(const RGB &rgb) {
  r += rgb.r;
  g += rgb.g;
  b += rgb.b;
  return *this;
}

RGB & RGB::operator-=(const RGB &rgb) {
  r -= rgb.r;
  g -= rgb.g;
  b -= rgb.b;
  return *this;
}

RGB & RGB::operator*=(const RGB &rgb) {
  r *= rgb.r;
  g *= rgb.g;
  b *= rgb.b;
  return *this;
}

RGB & RGB::operator/=(const RGB &rgb) {
  r /= rgb.r;
  g /= rgb.g;
  b /= rgb.b;
  return *this;
}

bool RGB::operator==(const RGB &rgb) {
  return r == rgb.r && g == rgb.g && b == rgb.b;
}

bool RGB::operator!=(const RGB &rgb) {
  return r != rgb.r || g != rgb.g || b != rgb.b;
}

RGB & RGB::clamp() {
  r = max(0.0f, min(1.0f, r));
  g = max(0.0f, min(1.0f, g));
  b = max(0.0f, min(1.0f, b));
  return *this;
}

string RGB::str() {
  ostringstream res;
  res << "RGB(" << r << ", " << g << ", " << b << ")";
  return res.str();
}

const RGB RGB_BLACK  (0.0f, 0.0f, 0.0f);
const RGB RGB_WHITE  (1.0f, 1.0f, 1.0f);
const RGB RGB_RED    (1.0f, 0.0f, 0.0f);
const RGB RGB_GREEN  (0.0f, 1.0f, 0.0f);
const RGB RGB_BLUE   (0.0f, 0.0f, 1.0f);
const RGB RGB_GREY_50(0.5f, 0.5f, 0.5f);

// HSV ========================================================================

HSV::HSV(float h, float s, float v) : h(h), s(s), v(v) {}

// Used for HSV::toRGB
template <class T> T abs(T a) { return a < 0 ? -a : a; }

RGB HSV::toRGB() {
  // Thanks to Wikipedia for the algorithm:
  // http://en.wikipedia.org/wiki/HSL_and_HSV#Converting_to_RGB
  
  // First, we find the chroma:
  double c = v * s;
  
  // Then we find the point (r1, g1, b1) along the bottom three faces of
  // the RGB cude with the same hue and chroma as the HSV color. "x" is the
  // second largest component of this color.

  double hp = h / (PI / 3);
  // compensate for floating point inaccuracies
  if (abs(hp - 6.0f) <= 0.001f) { hp = 0; }

  double x = c * (1 - abs(fmod(hp, 2.0) - 1));

  double r1, g1, b1;
  if      (0 <= hp && hp < 1) { r1 = c, g1 = x, b1 = 0; }
  else if (1 <= hp && hp < 2) { r1 = x, g1 = c, b1 = 0; }
  else if (2 <= hp && hp < 3) { r1 = 0, g1 = c, b1 = x; }
  else if (3 <= hp && hp < 4) { r1 = 0, g1 = x, b1 = c; }
  else if (4 <= hp && hp < 5) { r1 = x, g1 = 0, b1 = c; }
  else if (5 <= hp && hp < 6) { r1 = c, g1 = 0, b1 = x; }
  else                        { r1 = 0, g1 = 0, b1 = 0; }

  // Finally, we match the value by adding the same amount to each
  // component, yielding (r, g, b).
  double m = v - c;
  float r = (float)(r1 + m), g = (float)(g1 + m), b = (float)(b1 + m);
  return RGB(r, g, b);
}
