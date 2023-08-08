#include "num.h"

/*
   Returns the clipped value
   @param value the value
   @param min the minimum value
   @param max the maximum value
*/
const long clip(const long value, const long minValue, const long maxValue) {
  return min(max(value, minValue), maxValue);
}

/*
   Returns the clipped value
   @param value the value
   @param min the minimum value
   @param max the maximum value
*/
const int clip(const int value, const int minValue, const int maxValue) {
  return min(max(value, minValue), maxValue);
}

/*
  Returns normalized radians angle (in range -PI, PI)
*/
const float normalRad(const float rad) {
  float result = rad;
  while (result < -PI) {
    result += PI * 2;
  }
  while (result >= PI) {
    result -= PI * 2;
  }
  return result;
}

/*
  Returns normalized degrees angle (in range -180, 179)
*/
const int normalDeg(const int deg) {
  int result = deg;
  while (result < -180) {
    result += 360;
  }
  while (result >= 180) {
    result -= 360;
  }
  return result;
}
