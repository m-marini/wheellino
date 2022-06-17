#ifndef MotionSensor_h
#define MotionSensor_h

#include "Arduino.h"

#define PULSES_PER_ROOT     40
#define WHEEL_DIAMETER      0.067
#define TRACK               0.136

#define DISTANCE_PER_PULSE  (WHEEL_DIAMETER * PI / PULSES_PER_ROOT)

/*
   Multiplexer
*/
class MotionSensor {
  public:
    MotionSensor(byte leftPin, byte rightPin);
    MotionSensor& begin();
    MotionSensor& polling(unsigned long clockTime);
    MotionSensor& setDirection(float leftForward, float rightForward);
    MotionSensor& angle(float angle) {
      _angle = angle;
      return *this;
    }
    MotionSensor& reset();

    const float angle() const {
      return _angle;
    }

    const long xPulses() const {
      return _xPulses;
    }

    const long yPulses() const {
      return _yPulses;
    }

    const long rightPulses() const {
      return _rightPulses;
    }

    const long leftPulses() const {
      return _leftPulses;
    }

    const long x() const {
      return _xPulses * DISTANCE_PER_PULSE;
    }
    const long y() const {
      return _yPulses * DISTANCE_PER_PULSE;
    }

  private:
    byte _leftPin;
    byte _rightPin;
    byte _left;
    byte _right;
    bool _leftForward;
    bool _rightForward;
    float _angle;
    float _xPulses;
    float _yPulses;
    long  _leftPulses;
    long  _rightPulses;

    MotionSensor& update(int dl, int dr, unsigned long clockTime);
};

float normAngle(float angle);

#endif
