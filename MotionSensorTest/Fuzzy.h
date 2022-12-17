#ifndef FUZZY_H
#define FUZZY_H

#include "Arduino.h"

float fuzzyPositive(float value, float range);

class Fuzzy {
  public:
    Fuzzy();
    void add(float value, float weight);
    void reset();
    const float defuzzy() const;
  private:
    float _sum;
    float _scale;
};

#endif
