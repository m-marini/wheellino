#include "MotionSensor.h"

//#define DEBUG

#include "debug.h"

#define ANGLE_PER_PULSE     (DISTANCE_PER_PULSE / TRACK)

MotionSensor::MotionSensor(byte leftPin, byte rightPin) {
  _leftPin = leftPin;
  _rightPin = rightPin;
  _leftForward = true;
  _rightForward = true;
}

MotionSensor& MotionSensor::begin() {
  pinMode(_leftPin, INPUT);
  pinMode(_rightPin, INPUT);
  return *this;
}

MotionSensor& MotionSensor::reset() {
  _leftForward = true;
  _rightForward = true;
  _xPulses = 0;
  _yPulses = 0;
  _leftPulses = 0;
  _rightPulses = 0;
  _angle = 0;
  return *this;
}

MotionSensor& MotionSensor::setDirection(float left, float right) {
  DEBUG_PRINT(F("// MotionSensor::setDirection "));
  DEBUG_PRINT(left);
  DEBUG_PRINT(F(" "));
  DEBUG_PRINT(right);
  DEBUG_PRINTLN();

  if (left > 0) {
    _leftForward = true;
  } else if (left < 0) {
    _leftForward = false;
  }
  if (right > 0) {
    _rightForward = true;
  } else if (right < 0) {
    _rightForward = false;
  }
  return *this;
}

MotionSensor& MotionSensor::polling(unsigned long clockTime) {
  int left = digitalRead(_leftPin);
  int right = digitalRead(_rightPin);
  int dl = left != _left ?
           _leftForward ? 1 : -1
           : 0;
  int dr = right != _right ?
           _rightForward ? 1 : -1
           : 0;
  _left = left;
  _right = right;
  if (dl != 0 || dr != 0) {
    update(dl, dr, clockTime);
  }
  return *this;
}

MotionSensor& MotionSensor::update(int dl, int dr, unsigned long clockTime) {
  DEBUG_PRINT(F("// MotionSensor::update "));
  DEBUG_PRINT(dl);
  DEBUG_PRINT(F(" "));
  DEBUG_PRINT(dr);
  DEBUG_PRINTLN();

  _leftPulses += dl;
  _rightPulses += dr;

  // Updates location
  float sa = sinf(_angle);
  float ca = cosf(_angle);
  float ds = ((float)(dl + dr)) / 2;
  _xPulses += ca * ds;
  _yPulses += sa * ds;

  DEBUG_PRINT(F("// x,y "));
  DEBUG_PRINT(_xPulses);
  DEBUG_PRINT(F(" "));
  DEBUG_PRINT(_yPulses);
  DEBUG_PRINTLN();

  // Updates angle
  _angle = normAngle(_angle + (dl - dr) * ANGLE_PER_PULSE);

  DEBUG_PRINT(F("// angle "));
  DEBUG_PRINT(_angle * 180 / PI);
  DEBUG_PRINTLN();
  return *this;
}


/*
   Returns the normalized angle in range (-PI, PI)
*/
float normAngle(float angle) {
  while (angle <= PI) {
    angle += (PI + PI);
  }
  while (angle > PI) {
    angle -= (PI + PI);
  }
  return angle;
}
