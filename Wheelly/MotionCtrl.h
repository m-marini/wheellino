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
    void speed(float value);
    void setCorrection(float *x, float *y);

  private:
    byte _forwPin;
    byte _backPin;
    float *_x, *_y;
    float func(float x);
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
    void move(float direction, float speed);
    void halt();

    const float x() const {
      return _sensors.x();
    }
    const float y() const {
      return _sensors.y();
    }
    const float angle() const {
      return _sensors.angle();
    }
    const float left() const {
      return _left;
    }
    const float right() const {
      return _right;
    }
    const float leftSpeed() const {
      return _sensors.leftSpeed();
    }
    const float rightSpeed() const {
      return _sensors.rightSpeed();
    }
    const boolean isForward() const;
    const boolean isBackward() const;

    const boolean isHalt() const {
      return _halt;
    }

    void angle(float angle) {
      _sensors.angle(angle);
    }
    const float speed() const {
      return _speed;
    }
    const float direction() const {
      return _direction;
    }

  private:
    MotorCtrl _leftMotor;
    MotorCtrl _rightMotor;
    MotionSensor _sensors;
    Timer _stopTimer;
    Timer _checkTimer;

    float _direction;
    float _speed;
    boolean _halt;

    float _left;
    float _right;
    unsigned long _prevTime;

    void power(float left, float right);
};

#endif
