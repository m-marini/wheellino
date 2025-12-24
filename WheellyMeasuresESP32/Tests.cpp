#include "Tests.h"

#include <esp_log.h>
static const char* TAG = "Tests";

#define MAX_POWER 255
#define END_TEST_INTERVAL 500

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
  _isWaitingForEnd = false;
  _power = 0;
  _isSteppingUp = true;
  _nextStepInstant = t0 + stepUpInterval;
  changePower();
}

/**
       Stops the test
    */
void MotorTest::stop(void) {
  _power = 0;
  _isWaitingForEnd = true;
  _nextStepInstant = millis() + END_TEST_INTERVAL;
  changePower();
}

/*
  Poolling the test
  */
void MotorTest::pooling(const unsigned long t0) {
  if (_isTesting) {
    if (_isWaitingForEnd) {
      if (t0 >= _nextStepInstant) {
        // End test
        _power = 0;
        changePower();
        _isTesting = false;
        ESP_LOGI(TAG, "End test");
        changePower();
      }
    } else {
      if (t0 >= _nextStepInstant) {
        changePower();
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
              // Step down completed
              _power = 0;
              _isWaitingForEnd = true;
              _nextStepInstant = t0 + END_TEST_INTERVAL;
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
              // Step down completed
              _power = 0;
              _isWaitingForEnd = true;
              _nextStepInstant = t0 + END_TEST_INTERVAL;
            } else {
              _nextStepInstant = t0 + _stepDownInterval;
            }
          }
        }
      }
      changePower();
    }
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
