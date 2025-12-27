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
                      const unsigned long accelerationInterval,
                      const unsigned accelerationPower,
                      const unsigned long decelerationInterval,
                      const unsigned decelerationPower) {
  _maxPower = maxPower;
  _accelerationInterval = accelerationInterval;
  _decelerationInterval = decelerationInterval;
  _accelerationPower = accelerationPower;
  _decelerationPower = decelerationPower;

  _isTesting = true;
  _isWaitingForEnd = false;
  _power = 0;
  _isSteppingUp = true;
  _nextStepInstant = t0 + accelerationInterval;
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
            _power += _accelerationPower;
            if (_power >= _maxPower) {
              // Step up completed
              _power = _maxPower;
              _isSteppingUp = false;
              _nextStepInstant = t0 + _decelerationInterval;
            } else {
              _nextStepInstant = t0 + _accelerationInterval;
            }
          } else {
            // Stepping down
            _power -= _decelerationPower;
            if (_power <= 0) {
              // Step down completed
              _power = 0;
              _isWaitingForEnd = true;
              _nextStepInstant = t0 + END_TEST_INTERVAL;
            } else {
              _nextStepInstant = t0 + _decelerationInterval;
            }
          }
        } else {
          // backward testing
          if (_isSteppingUp) {
            // Stepping up
            _power -= _accelerationPower;
            if (_power < _maxPower) {
              // Step up completed
              _power = _maxPower;
              _isSteppingUp = false;
              _nextStepInstant = t0 + _decelerationInterval;
            } else {
              _nextStepInstant = t0 + _accelerationInterval;
            }
          } else {
            // Stepping down
            _power += _decelerationPower;
            if (_power >= 0) {
              // Step down completed
              _power = 0;
              _isWaitingForEnd = true;
              _nextStepInstant = t0 + END_TEST_INTERVAL;
            } else {
              _nextStepInstant = t0 + _decelerationInterval;
            }
          }
        }
        changePower();
      }
    }
  }
}

/**
  Change power
  */
void MotorTest::changePower(void) {
  ESP_LOGI(TAG, "Power %d @%lx", _power, (unsigned long)this);
  _motorCtrl.power(_power);
  if (_onPowerChange) {
    _onPowerChange(_context);
  }
}
