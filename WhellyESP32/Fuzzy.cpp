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

#include "Arduino.h"

#include "num.h"

/*
   Returns the fuzzy positivity of value
   @param value the value
   @param the range
*/
const float fuzzyGreater(const float value, const float minValue, const float maxValue) {
  return min(max((value - minValue) / (maxValue - minValue), (float)0), (float)1);
}

/*
   Creates the Defuzzier class
*/
Defuzzier::Defuzzier()
  : _sum(0), _scale(0) {}

/**
   Adds a value with a weight
   @param value the value
   @param weight the weight
*/
void Defuzzier::add(const float value, const float weight) {
  _sum += value * weight;
  _scale += weight;
}

/*
   Reset the defuzzier
*/
void Defuzzier::reset() {
  _sum = 0;
  _scale = 0;
}

/*
   Returns the defuzzy value
*/
const float Defuzzier::defuzzy() const {
  return _sum / _scale;
}
