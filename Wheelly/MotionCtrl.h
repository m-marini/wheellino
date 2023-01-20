#ifndef MotionCtrl_h
#define MotionCtrl_h

#include "MotionSensor.h"
#include "Timer.h"

#define NO_POINTS 5

/*
   Motor ontroller
*/
class MotorCtrl {
  public:
    MotorCtrl(byte forwPin, byte backPin);
    void begin();
    void speed(int value);
    void setCorrection_P(int *p);
    void setCorrection(int *p);

  private:
    byte _forwPin;
    byte _backPin;
    int _x[NO_POINTS], _y[NO_POINTS];
    int func(int x);
};

/*
   Motion controller
*/
class MotionCtrl {
  public:
    MotionCtrl(byte leftForwPin, byte leftBackPin, byte rightForwPin, byte rightBackPin, byte leftSensorPin, byte rightSensorPin);
    void begin();
    void polling(unsigned long clockTime = millis());
    void reset();
    void handleMotion(unsigned long clockTime);
    void move(int direction, int speed);
    void setCorrection(int *p);
    void setControllerConfig(int *p);
    void halt();
    const float xPulses() const {
      return _sensors.xPulses();
    }
    const float yPulses() const {
      return _sensors.yPulses();
    }
    const int angle() const {
      return _sensors.angle();
    }
    const float leftPps() const {
      return _sensors.leftPps();
    }
    const float rightPps() const {
      return _sensors.rightPps();
    }
    const boolean isForward() const;
    const boolean isBackward() const;

    const boolean isHalt() const {
      return _halt;
    }

    void angle(int angle) {
      _sensors.angle(angle);
    }
    const int speed() const {
      return _speed;
    }
    const int direction() const {
      return _direction;
    }

  private:
    MotorCtrl _leftMotor;
    MotorCtrl _rightMotor;
    MotionSensor _sensors;
    Timer _stopTimer;
    Timer _checkTimer;
    int _powerK;
    int _signalK;
    int _ppsK;
    int _moveRotThreshold;
    int _minRotRange;
    int _maxRotRange;

    int _direction;
    int _speed;
    boolean _halt;

    int _left;
    int _right;
    int _leftPower;
    int _rightPower;
    unsigned long _prevTime;

    void power(int left, int right);
};

#endif
