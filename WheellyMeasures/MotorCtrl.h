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
    MotorCtrl& speed(int value);
    
  private:
    byte _forwPin;
    byte _backPin;
};

#endif
