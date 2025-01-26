#ifndef MATHEMATICS_H
#define MATHEMATICS_H

#include <mbed.h>
#include <commTypes.h>

#define M_PI 3.14159265358979323846

namespace math {

  double map(double x, double in_min, double in_max, double out_min, double out_max);

  double distance(Vector a, Vector b);

  double to_positive_angle(double angle);

  double smallestAngleDiff(double target, double source);

} // namespace math

#endif // MATHEMATICS_H
