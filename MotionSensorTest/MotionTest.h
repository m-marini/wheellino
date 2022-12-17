#ifndef MOTION_TEST_H
#define MOTION_TEST_H

#include "Arduino.h"

#include "Timer.h"
#include "mpu6050mm.h"
#include "MotionCtrl.h"
#include "MotionSensor.h"

class MotionTest {
  public:
    MotionTest::MotionTest(const byte leftForwPin, const byte leftBackPin,
                           const byte rightForwPin, const byte rightBackPin,
                           const byte leftPin, const byte rightPin);
    void begin();
    void polling(unsigned long clockTime = millis());
    void start(const long timeout,
               const float leftSpeed, const float rightSpeed,
               const int numPulses);
    void setOnCompleted(void (*callback)(void* context, MotionTest& test), void* context = NULL);
    const long leftPulsesCounter() const {
      return _leftPulsesCounter;
    }
    const long rightPulsesCounter() const {
      return _rightPulsesCounter;
    }
    const float mpuAngle() const {
      return _mpuAngle;
    }
    const float sensAngle() const {
      return _sensAngle;
    }
    const long testTime() const {
      return _testTime;
    }
    const int mpuError() const {
      return _mpuError;
    }
  private:
    void complete();

    MotorCtrl _leftMotor;
    MotorCtrl _rightMotor;
    MotionSensor _sensors;
    MPU6050 _mpu;
    Timer testTimer;
    Timer _timer;

    boolean _running;
    int _numPulses;
    long _leftPulsesCounter;
    long _rightPulsesCounter;
    long _testTime;
    float _mpuAngle;
    float _sensAngle;
    int _mpuError;

    void *_context;
    void (*_onCompleted)(void*, MotionTest&);
    static void handleTestTimer(void *testPtr, unsigned long n);
};

#endif
