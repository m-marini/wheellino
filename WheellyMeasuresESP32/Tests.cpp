#include "Tests.h"

#include <esp_log.h>
static const char* TAG = "Tests";

#define MAX_POWER 255

/**
       Creates the motor test
    */
MotorTest::MotorTest(MotorCtrl& motor)
  : _motorCtrl(motor) {}

/**
       Starts the test
    */
void MotorTest::start(const unsigned long t0,
                      const int maxPower,
                      const unsigned long stepUpInterval,
                      const int stepUpPower,
                      const unsigned long stepDownInterval,
                      const int stepDownPower) {
  _maxPower = maxPower;
  _stepUpPower = stepUpPower;
  _stepDownPower = stepDownPower;
  _stepUpInterval = stepUpInterval;
  _stepDownInterval = stepDownInterval;

  _isTesting = true;
  _power = 0;
  _isSteppingUp = true;
  _nextStepInstant = t0 + stepUpInterval;
  changePower();
}

/**
       Stops the test
    */
void MotorTest::stop(void) {
  _isTesting = 0;
  _power = 0;
  changePower();
}

/*
  Poolling the test
  */
void MotorTest::pooling(const unsigned long t0) {
  if (_isTesting && t0 >= _nextStepInstant) {
    if (_maxPower > 0) {
      // forward testing
      if (_isSteppingUp) {
        // Stepping up
        _power += _stepUpPower;
        if (_power >= _maxPower) {
          // Step up completed
          _power = _maxPower;
          _isSteppingUp = false;
          _nextStepInstant = t0 + _stepDownInterval;
        } else {
          _nextStepInstant = t0 + _stepUpInterval;
        }
      } else {
        // Stepping down
        _power += _stepDownPower;
        if (_power <= 0) {
          // Step up completed
          _power = 0;
          _isTesting = false;
        } else {
          _nextStepInstant = t0 + _stepDownInterval;
        }
      }
    } else {
      // backward testing
      if (_isSteppingUp) {
        // Stepping up
        _power += _stepUpPower;
        if (_power < _maxPower) {
          // Step up completed
          _power = _maxPower;
          _isSteppingUp = false;
          _nextStepInstant = t0 + _stepDownInterval;
        } else {
          _nextStepInstant = t0 + _stepUpInterval;
        }
      } else {
        // Stepping down
        _power += _stepDownPower;
        if (_power >= 0) {
          // Step up completed
          _power = 0;
          _isTesting = false;
        } else {
          _nextStepInstant = t0 + _stepDownInterval;
        }
      }
    }
    changePower();
  }
}

/**
  Change power
  */
void MotorTest::changePower(void) {
  ESP_LOGD(TAG, "Power %d", _power);
  _motorCtrl.power(_power);
  if (_onPowerChange) {
    _onPowerChange(_context);
  }
}
