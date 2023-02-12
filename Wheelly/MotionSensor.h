#ifndef MotionSensor_h
#define MotionSensor_h

#include "Arduino.h"

#define PULSES_PER_ROOT     40
#define WHEEL_DIAMETER      0.067f

#define DISTANCE_PER_PULSE  (WHEEL_DIAMETER * PI / PULSES_PER_ROOT)

/*
  Speedometer measures the speed
*/
class Speedometer {
  public:
    Speedometer();
    void update(unsigned long time, int step);
    void reset(unsigned long timestamp);
    void tau(unsigned long tau) {
      _tau = tau;
    }
    const float pps() const {
      return _pps;
    }
  private:
    float _pps;
    unsigned long _tau;
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
    void direction(int speed);
    void reset(unsigned long timestamp);
    void tau(unsigned long tau) {
      _speedometer.tau(tau);
    }
    // Sets the callback
    void onSample(void (*callback)(void* context, int dPulse, unsigned long clockTime, MotorSensor& sensor), void* context = NULL) {
      _onSample = callback;
      _context = context;
    }

    const long pulses() const {
      return _pulses;
    }

    const float pps() const {
      return _speedometer.pps();
    }

  private:
    byte _sensorPin;
    byte _pulse;
    long _pulses;
    bool _forward;
    void (*_onSample)(void*, int, unsigned long clockTime, MotorSensor&);
    void *_context;
    Speedometer _speedometer;

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
    void direction(int leftForward, int rightForward);
    void setOnChange(void (*callback)(void* context, unsigned long clockTime, MotionSensor& sensor), void* context = NULL);
    void tau(unsigned long tau);
    void angle(int angle) {
      _angle = angle;
    }

    void reset(unsigned long timestamp);

    void setLeftPulses(int dPulse) {
      _dl = dPulse;
    }
    void setRightPulses(int dPulse) {
      _dr = dPulse;
    }

    const int angle() const {
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

    const float leftPps() const {
      return _leftSensor.pps();
    }
    const float rightPps() const {
      return _rightSensor.pps();
    }
    void updateAngle(boolean updateAngle) {
      _updateAngle = updateAngle;
    }
    MotorSensor& leftSensor() {
      return _leftSensor;
    }
    MotorSensor& rightSensor() {
      return _rightSensor;
    }

  private:
    MotorSensor _leftSensor;
    MotorSensor _rightSensor;
    boolean _updateAngle;
    int _angle;
    float _xPulses;
    float _yPulses;
    int _dl;
    int _dr;
    void (*_onChange)(void*, unsigned long, MotionSensor&);
    void* _context;

    void update(unsigned long clockTime);
};

#endif
