/*
 * Copyright (c) 2023  Marco Marini, marco.marini@mmarini.org
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 *    END OF TERMS AND CONDITIONS
 *
 */

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
