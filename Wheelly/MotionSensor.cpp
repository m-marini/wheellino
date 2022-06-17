#include "MotionSensor.h"


//#define DEBUG
#include "debug.h"

#include "Utils.h"

#define ANGLE_PER_PULSE (DISTANCE_PER_PULSE / TRACK)
#define FILTER_DECAY  (1.0 / 300.0)

/*

*/
void handleLeftSensor(void* context, int dPulse, unsigned long, MotorSensor&) {
  DEBUG_PRINT(F("// MotionSensor::handleLeftSensor "));
  DEBUG_PRINT(dPulse);
  DEBUG_PRINTLN();
  ((MotionSensor*) context)->setLeftPulses(dPulse);
}

/*

*/
void handleRightSensor(void* context, int dPulse, unsigned long, MotorSensor&) {
  DEBUG_PRINT(F("// MotionSensor::handleRightSensor "));
  DEBUG_PRINT(dPulse);
  DEBUG_PRINTLN();
  ((MotionSensor*) context)->setRightPulses(dPulse);
}

/*

*/
MotionSensor::MotionSensor(byte leftPin, byte rightPin) :
  _leftSensor(leftPin), _rightSensor(rightPin) {
  _leftSensor.onSample(handleLeftSensor, this);
  _rightSensor.onSample(handleRightSensor, this);
}

void MotionSensor::begin() {
  _leftSensor.begin();
  _rightSensor.begin();
}

void MotionSensor::reset() {
  _leftSensor.reset();
  _rightSensor.reset();
  _xPulses = 0;
  _yPulses = 0;
  _angle = 0;
}

void MotionSensor::setDirection(float left, float right) {
  DEBUG_PRINT(F("// MotionSensor::setDirection "));
  DEBUG_PRINT(left);
  DEBUG_PRINT(F(" "));
  DEBUG_PRINT(right);
  DEBUG_PRINTLN();

  _leftSensor.setDirection(left);
  _rightSensor.setDirection(right);
}

void MotionSensor::polling(unsigned long clockTime) {
  _dl = _dr = 0;
  _leftSensor.polling(clockTime);
  _rightSensor.polling(clockTime);
  if (_dl != 0 || _dr != 0) {
    update(clockTime);
  }
}

void MotionSensor::update(unsigned long clockTime) {
  DEBUG_PRINT(F("// MotionSensor::update "));
  DEBUG_PRINT(_dl);
  DEBUG_PRINT(F(" "));
  DEBUG_PRINT(_dr);
  DEBUG_PRINTLN();

  // Updates location
  float sa = sinf(_angle);
  float ca = cosf(_angle);
  float ds = ((float)(_dl + _dr)) / 2;
  _xPulses += ca * ds;
  _yPulses += sa * ds;

  DEBUG_PRINT(F("// x,y "));
  DEBUG_PRINT(_xPulses);
  DEBUG_PRINT(F(" "));
  DEBUG_PRINT(_yPulses);
  DEBUG_PRINTLN();

  // Updates angle
  _angle = normalRad(_angle + (_dl - _dr) * ANGLE_PER_PULSE);

  DEBUG_PRINT(F("// angle "));
  DEBUG_PRINT(_angle * 180 / PI);
  DEBUG_PRINTLN();

  DEBUG_PRINT(F("// location "));
  DEBUG_PRINT(x());
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(y());
  DEBUG_PRINTLN();

  DEBUG_PRINT(F("// pps "));
  DEBUG_PRINT(leftPps());
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(rightPps());
  DEBUG_PRINTLN();

  if (_onChange != NULL) {
    _onChange(_context, clockTime, *this);
  }
}

/*

*/
void MotionSensor::setOnChange(void (*callback)(void* context, unsigned long clockTime, MotionSensor& sensor), void* context = NULL) {
  _onChange = callback;
  _context = context;
}


/*

*/
MotorSensor::MotorSensor(byte sensorPin) {
  _sensorPin = sensorPin;
  _forward = true;
}

/*

*/
void MotorSensor::begin() {
  pinMode(_sensorPin, INPUT);
  _speedometer.reset();
  _filter.reset(millis());
}

/*

*/
void MotorSensor::reset() {
  _forward = true;
  _pulses = 0;
  _speedometer.reset();
  _filter.reset(millis());
}

/*

*/
void MotorSensor::setDirection(float speed) {
  DEBUG_PRINT(F("// MotorSensor::setDirection "));
  DEBUG_PRINT(speed);
  DEBUG_PRINTLN();

  if (speed > 0) {
    _forward = true;
  } else if (speed < 0) {
    _forward = false;
  } else {
    _speedometer.reset();
    _filter.reset(millis());
  }
}

/*

*/
void MotorSensor::polling(unsigned long clockTime) {
  int pulse = digitalRead(_sensorPin);
  int dPulse = pulse != _pulse ? _forward ? 1 : -1 : 0;

  _pulse = pulse;
  if (dPulse != 0) {
    update(dPulse, clockTime);
  }
  _filter.value(_speedometer.pps(clockTime), clockTime);
  DEBUG_PRINT(F("// MotorSensor::polling "));
  DEBUG_PRINT(_filter.value());
  DEBUG_PRINTLN();
}

/*

*/
void MotorSensor::update(int dPulse, unsigned long clockTime) {
  _pulses += dPulse;

  DEBUG_PRINT(F("// MotorSensor::update dPulse:"));
  DEBUG_PRINT(dPulse);
  DEBUG_PRINT(F(", _pulses "));
  DEBUG_PRINT(_pulses);
  DEBUG_PRINTLN();

  if (dPulse < 0) {
    _speedometer.backward(clockTime);
  } else if (dPulse > 0) {
    _speedometer.forward(clockTime);
  }
  if (_onSample != NULL) {
    _onSample(_context, dPulse, clockTime, *this);
  }
}

/*
   Speedometer section
*/

void Speedometer::forward(unsigned long clockTime) {
  unsigned long dt = clockTime - _prevTime;
  if (dt > 0) {
    _pps = 1000.0 / dt;
    _prevTime = clockTime;
    DEBUG_PRINT(F("// Speedometer::forward speed "));
    DEBUG_PRINTF(_pps, 3);
    DEBUG_PRINTLN();
  }
}

void Speedometer::backward(unsigned long clockTime) {
  unsigned long dt = clockTime - _prevTime;
  if (dt > 0) {
    _pps = -1000.0 / dt;
    _prevTime = clockTime;
    DEBUG_PRINT(F("// Speedometer::backward speed "));
    DEBUG_PRINTF(_pps, 3);
    DEBUG_PRINTLN();
  }
}

const float Speedometer::pps(unsigned long clockTime) const {
  unsigned long dt = clockTime - _prevTime;
  if (dt > 0 && _pps != 0) {
    float pps = (_pps < 0 ? -1000.0 : 1000.0) / dt;
    return abs(pps) <= abs(_pps) ? pps : _pps;
  } else {
    return _pps;
  }
}

void Speedometer::reset() {
  _pps = 0;
  _prevTime = millis();
}

/*
   Low pass filter section
*/

void LowPassFilter::value(float value, unsigned long clockTime) {
  float alpha = min((clockTime - _prevTime) * FILTER_DECAY, 1);
  _value = _value *(1-alpha) + value * alpha;
  _value += (-_value + value) * alpha;
  _prevTime = clockTime;
}

void LowPassFilter::reset(unsigned long clockTime) {
  _value = 0;
  _prevTime = clockTime;
}
