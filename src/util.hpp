#ifndef UTIL_HPP
#define UTIL_HPP

#include <string>
using namespace std;

extern const double PI;
extern const float F_INF; // Positive infinity

/**
 * Useful viewport properties in a single class
 */
class Viewport {
  public: int w, h; // width and height
};

/**
 * A single color, represented by red, green and blue values. All values are
 * floats between 0.0f and 1.0f for no intensity to full intensity.
 */
class RGB {
  public:
    RGB(float r, float g, float b);
    float r, g, b; // between 0 and 1

    RGB   operator* (const float c)  const;
    RGB   operator/ (const float c)  const;
    RGB   operator* (const RGB &rgb) const; // point-wise
    RGB   operator/ (const RGB &rgb) const; // point-wise
    RGB   operator+ (const RGB &rgb) const;
    RGB   operator- (const RGB &rgb) const;
    RGB & operator*=(const float c) ;
    RGB & operator/=(const float c) ;
    RGB & operator*=(const RGB &rgb); // point-wise
    RGB & operator/=(const RGB &rgb); // point-wise
    RGB & operator+=(const RGB &rgb);
    RGB & operator-=(const RGB &rgb);

    bool operator==(const RGB &rgb);
    bool operator!=(const RGB &rgb);

    /**
     * Force the values of the components between 0.0f and 1.0f. Any values
     * less than 0.0f are set to 0.0f, and any values greater than 1.0f are set
     * to 1.0f. Acts in place.
     *
     * @return  this, for chaining
     */
    RGB & clamp();

    string str();
};

extern const RGB RGB_BLACK  ;
extern const RGB RGB_WHITE  ;
extern const RGB RGB_RED    ;
extern const RGB RGB_GREEN  ;
extern const RGB RGB_BLUE   ;
extern const RGB RGB_GREY_50;

/**
 * A single color, represented by hue, saturation and value (brightness). All
 * value are floats, with the hue being an angle in radians, and the others
 * being between 0.0f and 1.0f.
 *
 * Can be converted into an RBG value for use with OpenGL.
 */
class HSV {
  public:
    HSV(float h, float s, float v);
    float h,    // in radians, between 0 and 2*PI
          s, v; // between 0 and 1

    /**
     * Converts this HSV color to its RGB representation for use with OpenGL
     * functions.
     *
     * @return  this color, as an RGB object
     */
    RGB toRGB();
};

#endif // UTIL_HPP
