#include "MotorCtrl.h"

#define MAX_VALUE 255

MotorCtrl::MotorCtrl(byte forwPin, byte backPin) {
  _forwPin = forwPin;
  _backPin = backPin;
}

MotorCtrl& MotorCtrl::begin() {
  pinMode(_forwPin, OUTPUT);
  pinMode(_backPin, OUTPUT);
  return *this;
}

/*
   Set speed
*/
MotorCtrl& MotorCtrl::speed(int value) {
  if (value == 0) {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, 0);
  } else if (value > 0) {
    analogWrite(_forwPin, value);
    analogWrite(_backPin, 0);
  } else {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, -value);
  }
  return *this;
}
