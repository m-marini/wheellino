#include "MotionTest.h"

#define DEBUG
#include "debug.h"
#include "Utils.h"

const float leftXCorrection[] PROGMEM = { -1,  -0.06055, 0, 0.02311, 1};
const float leftYCorrection[] PROGMEM = { -1, -0.30432, 0, 0.12577, 1};
const float rightXCorrection[] PROGMEM = { -1, -0.03759, 0, 0.02041, 1};
const float rightYCorrection[] PROGMEM = { -1, -0.2667, 0, 0.12648, 1};

MotionTest::MotionTest(const byte leftForwPin, const byte leftBackPin,
                       const byte rightForwPin, const byte rightBackPin,
                       const byte leftPin, const byte rightPin) :
  _leftMotor(leftForwPin, leftBackPin),
  _rightMotor(rightForwPin, rightBackPin),
  _sensors(leftPin, rightPin) {
}

void MotionTest::begin() {
  _mpu.begin();
  _mpu.calibrate();

  _leftMotor.begin();
  _leftMotor.setCorrection(leftXCorrection, leftYCorrection);
  _rightMotor.begin();
  _rightMotor.setCorrection(rightXCorrection, rightYCorrection);
  _leftMotor.speed(0);
  _rightMotor.speed(0);

  _sensors.begin();
}

void MotionTest::setOnCompleted(void (*callback)(void* context, MotionTest& test), void* context) {
  _onCompleted = callback;
  _context = context;
}

void MotionTest::start(const long duration, const float leftSpeed, const float rightSpeed,
                       const int numPulses) {
  _timer.interval(duration);
  _timer.onNext(handleTestTimer, this);
  _timer.start();
  _sensors.setDirection(leftSpeed, rightSpeed);
  _leftMotor.speed(leftSpeed);
  _rightMotor.speed(rightSpeed);
  _numPulses = numPulses;
  _leftPulsesCounter = -_sensors.leftPulses();
  _rightPulsesCounter = -_sensors.rightPulses();
  _mpuAngle = -_mpu.yaw();
  _sensAngle = -_sensors.angle();
  _testTime = -millis();
  _mpuError = _mpu.rc();
  _running = true;
}

void MotionTest::polling(unsigned long clockTime) {
  _mpu.polling(clockTime);
  _sensors.polling(clockTime);
  _timer.polling(clockTime);
  if (_running) {
    long pulses = abs(_sensors.leftPulses() + _leftPulsesCounter)
                  + abs(_sensors.rightPulses() + _rightPulsesCounter);
    if (pulses >= _numPulses) {
      _timer.stop();
      complete();
    }
  }
}

static void MotionTest::handleTestTimer(void *testPtr, unsigned long n) {
  ((MotionTest*)testPtr)->complete();
}

void MotionTest::complete() {
  _running = false;
  _leftMotor.speed(0);
  _rightMotor.speed(0);
  _leftPulsesCounter += _sensors.leftPulses();
  _rightPulsesCounter += _sensors.rightPulses();
  _mpuAngle = normalRad(_mpu.yaw() + _mpuAngle);
  _sensAngle = normalRad(_sensors.angle() + _sensAngle);
  _testTime += millis();
  if (!_mpuError) {
    _mpuError = _mpu.rc();
  }
  if (_onCompleted != NULL) {
    _onCompleted(_context, *this);
  }
}
