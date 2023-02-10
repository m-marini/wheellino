#ifndef MotionCtrl_h
#define MotionCtrl_h

#include "MotionSensor.h"
#include "Timer.h"

/*
   Motor ontroller
*/
class MotorCtrl {
  public:
    MotorCtrl(byte forwPin, byte backPin, MotorSensor& sensor);
    void begin();
    void speed(int value);
    /*
     * Sets the configuration parameters
     * [ 
     *   nu, 
     *   p0Forw, p1Forw, muForw, hForw,
     *   p0Back, p1Back, muBack, hBack
     * ]
     */
    void config(int* parms);
    const int speed() const {return _speed;}

  private:
    byte _forwPin;
    byte _backPin;
    MotorSensor& _sensor;
    int _speed;
    int _power;
    int _nu;
    int _p0Forw;
    int _p1Forw;
    int _muForw;
    int _hForw;
    int _p0Back;
    int _p1Back;
    int _muBack;
    int _hBack;
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
    /*
     * Sets the configuration parameters
     * [moveRotThreshold, minRotRange, maxRotRange, maxRotPps ]
     */
    void controllerConfig(int *p);
    void decay(float p) {
      _sensors.decay(p);
    }

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
    MotorCtrl leftMotor() const {
      return _leftMotor;
    }
    MotorCtrl rightMotor() const {
      return _rightMotor;
    }

  private:
    MotorCtrl _leftMotor;
    MotorCtrl _rightMotor;
    MotionSensor _sensors;
    Timer _stopTimer;
    Timer _checkTimer;
    int _moveRotThreshold;
    int _minRotRange;
    int _maxRotRange;
    int _maxRotPps;

    int _direction;
    int _speed;
    boolean _halt;

    unsigned long _prevTime;

    void power(int left, int right);
};

#endif
