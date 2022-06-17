#include "Fuzzy.h"

float fuzzyPositive(float value, float range) {
  return min(max(value / range, 0), 1);
}
Fuzzy::Fuzzy() : _sum(0), _scale(0) {}

void Fuzzy::add(float value, float weight) {
  _sum += value * weight;
  _scale += weight;
}
void Fuzzy::reset() {
  _sum = 0;
  _scale = 0;
}
const float Fuzzy::defuzzy() const {
  return _sum / _scale;
}
