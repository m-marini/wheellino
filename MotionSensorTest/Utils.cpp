#include "Utils.h"

/*
  Returns normalized radians angle (in range -PI, PI)
*/
float normalRad(float rad) {
  while (rad < -PI) {
    rad += PI * 2;
  }
  while (rad >= PI) {
    rad -= PI * 2;
  }
  return rad;
}
