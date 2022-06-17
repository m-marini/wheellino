#include "MotorCtrl.h"

#define MAX_VALUE 255

MotorCtrl::MotorCtrl(byte forwPin, byte backPin) {
  _forwPin = forwPin;
  _backPin = backPin;
  _x[0] = _y[0] = -1;
  _x[1] = _y[1] = -0.5;
  _x[2] = _y[2] = 0;
  _x[3] = _y[3] = 0.5;
  _x[4] = _y[4] = 1;
}

MotorCtrl& MotorCtrl::begin() {
  pinMode(_forwPin, OUTPUT);
  pinMode(_backPin, OUTPUT);
  return *this;
}

MotorCtrl& MotorCtrl::setCorrection(float *x, float *y) {
  for (int i = 0; i < NO_POINTS; i++) {
    _x[i] = x[i];
    _y[i] = y[i];
  }
  return *this;
}

/*
   Set speed
*/
MotorCtrl& MotorCtrl::speed(float value) {
  float pwd = func(value);
  if (value == 0) {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, 0);
  } else if (value > 0) {
    int signal = round(pwd * MAX_VALUE);
    analogWrite(_forwPin, signal);
    analogWrite(_backPin, 0);
  } else {
    int signal = -round(pwd * MAX_VALUE);
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, signal);
  }
  return *this;
}

float MotorCtrl::func(float x) {
  int i = NO_POINTS - 2;
  for (int j = 1; j < NO_POINTS - 2; j++) {
    if (x < _x[j]) {
      i = j - 1;
      break;
    }
  }
  float result = (x - _x[i]) * (_y[i + 1] - _y[i]) / (_x[i + 1] - _x[i]) + _y[i];
  return min(max(-1, result), 1);
}
