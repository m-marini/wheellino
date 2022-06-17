#ifndef MotorCtrl_h
#define MotorCtrl_h

#include "Arduino.h"

#define NO_POINTS 5

/*
 * Multiplexer
 */
class MotorCtrl {
  public:
    MotorCtrl(byte forwPin, byte backPin);
    MotorCtrl& begin();
    MotorCtrl& speed(float value);
    MotorCtrl& setCorrection(float *x, float *y);
    
  private:
    byte _forwPin;
    byte _backPin;
    float _x[NO_POINTS],_y[NO_POINTS];
    float func(float x);
};

#endif
