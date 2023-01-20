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
/*
  Returns normalized degrees angle (in range -180, 179)
*/
int normalDeg(int deg) {
  while (deg < -180) {
    deg += 360;
  }
  while (deg >= 180) {
    deg -= 360;
  }
  return deg;
}
