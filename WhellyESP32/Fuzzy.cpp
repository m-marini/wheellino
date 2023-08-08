#include "Arduino.h"

#include "num.h"

/*
   Returns the fuzzy positivity of value
   @param value the value
   @param the range
*/
const float fuzzyPositive(const float value, const float range) {
  return min(max(value / range, (float)0), (float)1);
}

/*
   Creates the Defuzzier class
*/
Defuzzier::Defuzzier() : _sum(0), _scale(0) {}

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
