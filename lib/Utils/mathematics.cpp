#include "mathematics.h"

namespace math {
  /**
   * @brief Returns a max of X in the parameters bound.
   *
   * @param x
   * @param in_min
   * @param in_max
   * @param out_min
   * @param out_max
   * @return double
   */
  double map(double x, double in_min, double in_max, double out_min, double out_max) {
    if (x <= in_min)
      return out_min;
    if (x >= in_max)
      return out_max;

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  /**
   * @brief Returns Euclidian distace of two points.
   *
   * @param a
   * @param b
   * @return double
   */
  double distance(Vector a, Vector b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
  }

  /**
   * @brief Returns a angle between 0 and 2 * PI.
   *
   * @param angle
   * @return double
   */
  double to_positive_angle(double angle) {
    return fmod(angle + 2 * M_PI, 2 * M_PI);
  }

  /**
   * @brief Returns the smalles difference of two angles.
   *
   * @param target
   * @param source
   * @return double
   */
  double smallestAngleDiff(double target, double source) {
    double a;
    a = to_positive_angle(target) - to_positive_angle(source);

    if (a > M_PI)
      a = a - 2 * M_PI;

    else if (a < -M_PI)
      a = a + 2 * M_PI;

    return a;
  }
} // namespace math
