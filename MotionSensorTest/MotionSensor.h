#ifndef MotionSensor_h
#define MotionSensor_h

#include "Arduino.h"

#define TRACK               0.136f

#define PULSES_PER_ROOT     40
#define WHEEL_DIAMETER      0.067f

#define DISTANCE_PER_PULSE  (WHEEL_DIAMETER * PI / PULSES_PER_ROOT)

#define MAX_PPS 60

class LowPassFilter {
  public:
    void value(float value, unsigned long clockTime);
    void reset(unsigned long clockTime);
    const float value() const {
      return _value;
    }
  private:
    float _value;
    unsigned long _prevTime;
};

/*
  Speedometer measures the speed
*/
class Speedometer {
  public:
    Speedometer() {}
    void forward(unsigned long time);
    void backward(unsigned long time);
    void reset();
    const float pps(unsigned long time) const;
    const unsigned long prevTime() const {
      return _prevTime;
    }
  private:
    float _pps;
    unsigned long _prevTime;
};

/*
   Motor sensor measures the movement of motor
*/
class MotorSensor {
  public:
    MotorSensor(byte sensorPin);
    void begin();
    void polling(unsigned long clockTime);
    void setDirection(float speed);
    void reset();
    // Sets the callback
    void onSample(void (*callback)(void* context, int dPulse, unsigned long clockTime, MotorSensor& sensor), void* context = NULL) {
      _onSample = callback;
      _context = context;
    }

    const long pulses() const {
      return _pulses;
    }

    const float distance() const {
      return _pulses * DISTANCE_PER_PULSE;
    }

    const float pps() const {
      return _filter.value();
    }

    const float speed() const {
      return _filter.value() * DISTANCE_PER_PULSE;
    }

  private:
    byte _sensorPin;
    byte _pulse;
    long _pulses;
    bool _forward;
    void (*_onSample)(void*, int, unsigned long clockTime, MotorSensor&);
    void *_context;
    Speedometer _speedometer;
    LowPassFilter _filter;

    void update(int dPulse, unsigned long clockTime);
};

/*
   Multiplexer
*/
class MotionSensor {
  public:
    MotionSensor(byte leftPin, byte rightPin);
    void begin();
    void polling(unsigned long clockTime);
    void setDirection(float leftForward, float rightForward);
    void setOnChange(void (*callback)(void* context, unsigned long clockTime, MotionSensor& sensor), void* context = NULL);

    void angle(float angle) {
      _angle = angle;
    }

    void reset();

    void setLeftPulses(int dPulse) {
      _dl = dPulse;
    }
    void setRightPulses(int dPulse) {
      _dr = dPulse;
    }

    const float angle() const {
      return _angle;
    }

    const float xPulses() const {
      return _xPulses;
    }

    const float yPulses() const {
      return _yPulses;
    }

    const long rightPulses() const {
      return _rightSensor.pulses();
    }

    const long leftPulses() const {
      return _leftSensor.pulses();
    }

    const float x() const {
      return _xPulses * DISTANCE_PER_PULSE;
    }
    const float y() const {
      return _yPulses * DISTANCE_PER_PULSE;
    }
    const float leftPps() const {
      return _leftSensor.pps();
    }
    const float rightPps() const {
      return _rightSensor.pps();
    }

    const float leftSpeed() const {
      return _leftSensor.speed();
    }
    const float rightSpeed() const {
      return _rightSensor.speed();
    }

  private:
    MotorSensor _leftSensor;
    MotorSensor _rightSensor;
    float _angle;
    float _xPulses;
    float _yPulses;
    int _dl;
    int _dr;
    void (*_onChange)(void*, unsigned long, MotionSensor&);
    void* _context;

    void update(unsigned long clockTime);
};

#endif
