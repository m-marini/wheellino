/*
 * Copyright (c) 2023  Marco Marini, marco.marini@mmarini.org
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 *    END OF TERMS AND CONDITIONS
 *
 */

#include <esp_log.h>
static char* TAG = "MotorCtrl";

#include "Arduino.h"

#include "MotorCtrl.h"

#include "num.h"
#include "pins.h"

//#define DEFAULT_VOLTAGE 2500
#define DEFAULT_TAU 300ul
#define DEFAULT_ASR 200
#define DEFAULT_PWM_FACTOR 1400
#define DEFAULT_MAX_PULSE_INTERVAL 1000
#define DEFAULT_LAMBDA_FACTOR 10
#define DEFAULT_DELAYED_INTERVAL 10

static int MAX_PWM = 255;
static long ASR_SCALE = 1000;

/**
Handles the pulse from sensor
*/
void handlePulses(void* context, const int pulses, const unsigned long t, MotorSensor* sensor) {
  MotorCtrl* controller = (MotorCtrl*)context;
  controller->onPulses(pulses, t);
}

/*
   Creates the motor controller
*/
MotorCtrl::MotorCtrl(const uint8_t forwPin, const uint8_t backPin, const uint8_t sensorPin)
  : _forwPin(forwPin),
    _backPin(backPin),
    _status(MOTOR_HALT),
    _sensor(sensorPin),
    _automatic(true),
    _asr(true),
    _fPwmFactor(DEFAULT_PWM_FACTOR),
    _bPwmFactor(-DEFAULT_PWM_FACTOR),
    _tcs({ .asr = DEFAULT_ASR,
           .maxPulseInterval = DEFAULT_MAX_PULSE_INTERVAL,
           .lambdaFactor = DEFAULT_LAMBDA_FACTOR,
           .delayedInterval = DEFAULT_DELAYED_INTERVAL }) {
}

/*
   Initializes motor controller
*/
void MotorCtrl::begin() {
  ESP_LOGI(TAG, "Begin");
  pinMode(_forwPin, OUTPUT);
  pinMode(_backPin, OUTPUT);
  _sensor.begin();
  _sensor.onSample(handlePulses, this);
}

/*
  Returns the target pwm
  */
const int MotorCtrl::computePwm(void) const {
  int pwm = (_speed > 0
               ? _fPwmFactor
             : _speed < 0
               ? _bPwmFactor
               : 0)
            / (long)_pulseInterval;
  return constrain(pwm, -MAX_PWM, MAX_PWM);
}

/**
  Returns the power factor
  @param pulseWidth the pulse width (ms)
*/
void MotorCtrl::updatePwmFactor(const long pulseWidth) {
  // Compute factor with last pulse width
  ESP_LOGD(TAG, "Update for pulse width %lu", pulseWidth);
  long pwmFactor = pulseWidth > 0 ? _pwm * pulseWidth : _pwm;
  if (pwmFactor == 0) {
    pwmFactor = _pwm > 0 ? 1 : -1;
  }
  // Average with previous value
  ESP_LOGD(TAG, "Update pwmFactor from %ld, %ld with %ld", _fPwmFactor, _bPwmFactor, pwmFactor);
  if (_pwm > 0) {
    _fPwmFactor = _fPwmFactor * (100 - _tcs.lambdaFactor) + pwmFactor * _tcs.lambdaFactor;
    _fPwmFactor /= 100;
  } else {
    _bPwmFactor = _bPwmFactor * (100 - _tcs.lambdaFactor) + pwmFactor * _tcs.lambdaFactor;
    _bPwmFactor /= 100;
  }
  ESP_LOGD(TAG, "                   to %ld, %ld", _fPwmFactor, _bPwmFactor);
}

/**
   Sets the pwm factor
  */
void MotorCtrl::pwm(const int pwm, const unsigned long t) {
  _targetPwm = pwm;
  ESP_LOGD(TAG, "target pwm = %d", pwm);
  if (!_asr || pwm == 0) {
    // immediate pwm if no asr or motor halt
    applyPwm(pwm, t);
  }
}

/*
   Applies the power to the motor
   @param pwr the power -255 ... 255
*/
void MotorCtrl::applyPwm(const int pwm, const unsigned long t0) {
  ESP_LOGD(TAG, "pwm=%d", pwm);
  if (_pwm != pwm || pwm == 0) {
    _lastPwmTime = t0;
  }
  _pwm = pwm;
  _sensor.direction(pwm);
  // Applies the power
  if (pwm == 0) {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, 0);
  } else if (pwm > 0) {
    analogWrite(_forwPin, pwm);
    analogWrite(_backPin, 0);
  } else {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, -pwm);
  }
}

/*
   Sets the motor speed
*/
void MotorCtrl::speed(const int value) {
  ESP_LOGD(TAG, "speed(%d)", value);
  const unsigned long t = millis();
  if (value == 0
      || _speed < 0 && value > 0
      || _speed > 0 && value < 0) {
    // Speed == 0 or speed direction != current speed: halt motor
    _status = MOTOR_HALT;
    _speed = 0;
    pwm(0, t);
    ESP_LOGD(TAG, "Stop motor");
  } else if (_speed == 0) {
    // motor stopped
    _status = MOTOR_STARTING;
    // set speed
    _speed = value;
    // set expected pulse interval
    _pulseInterval = 1000 / abs(value);
    // set last pulse time
    _lastPulsesTime = t;
    // Compute initial pwm and apply immediate
    int pwm = computePwm();
    ESP_LOGD(TAG, "Start pwm=%d pulseInterval=%lu", pwm, _pulseInterval);
    this->pwm(pwm, t);
  } else {
    // Change speed and pulse interval
    _speed = value;
    _pulseInterval = 1000 / abs(value);
    ESP_LOGD(TAG, "Change speed pulseInterval=%lu", _pulseInterval);
  }
}

/*
  Polls the motor controller
*/
void MotorCtrl::polling(const unsigned long t) {
  _sensor.polling(t);

  switch (_status) {
    case MOTOR_STARTING:
      starting(t);
      break;
    case MOTOR_RUNNING:
      running(t);
      break;
  }
  // Regulate pwm with anti-slip regulation
  if (_targetPwm != _pwm) {
    // computes the interval since last pwm
    long dt = t - _lastPwmTime;
    // Check for time elapsed
    if (dt > 0) {
      // compute the maximum pwm change
      int dpMax = _tcs.asr * dt / ASR_SCALE;
      // Compute the next pwm (absolute value)
      int newAbsPwm = min(abs(_targetPwm), abs(_pwm) + dpMax);
      if (dpMax > 0 && newAbsPwm != abs(_pwm)) {
        ESP_LOGD(TAG, "ASR: dt=%lu dpMax=%d, newAbsPwm=%d targetPwm=%d", dt, dpMax, newAbsPwm, _targetPwm);
        // pwm change valid
        applyPwm(_targetPwm > 0 ? newAbsPwm : -newAbsPwm, t);
        // store change pwm instant
        _lastPwmTime = t;
      }
    }
  }
}

/*
  Handling polling during starting status
*/
void MotorCtrl::starting(const unsigned long t) {
  long pulseInterval = t - _startTime;
  if (pulseInterval > _pulseInterval) {
    // no pulse detected (no motion)
    // Incremet pwm
    pwm(_speed > 0 ? MAX_PWM : -MAX_PWM, t);
  }
}

/*
  Handling polling during running status
*/
void MotorCtrl::running(const unsigned long t) {
  long dt = t - _lastPulsesTime;
  if (dt > _tcs.maxPulseInterval) {
    // no pulse detected (no motion)
    ESP_LOGD(TAG, "no pulses dt=%ul pulseInterval=%lu", dt, _pulseInterval);
    // Incremet pwm
    ESP_LOGD(TAG, "Starting motor");
    _status = MOTOR_STARTING;
    _startTime = t;
    pwm(computePwm(), t);
  } else if (dt > _pulseInterval && (t - _lastDelayedTime) > _tcs.delayedInterval) {
    // Delayed pulse
    ESP_LOGD(TAG, "delayed pulse dt=%ul pulseInterval=%lu", dt, _pulseInterval);
    _lastDelayedTime = t;
    updatePwmFactor(dt);
    pwm(computePwm(), t);
  }
}

/*
  Handles pulses from sensor
*/
void MotorCtrl::onPulses(const int pulses, const unsigned long t) {
  // Compute the time interval
  const long dt = t - _lastPulsesTime;
  _lastPulsesTime = t;
  if (dt == 0) {
    return;
  }
  // Compute the average pulse interval
  const long pulseWidth = max(dt / abs(pulses), 1l);
  ESP_LOGD(TAG, "pulse width=%ul, dt=%ul, pulses=%d", pulseWidth, dt, pulses);
  switch (_status) {
    case MOTOR_STARTING:
      // Starting phase
      ESP_LOGD(TAG, "Running motor");
      _status = MOTOR_RUNNING;
      ESP_LOGD(TAG, "Start at pwm=%d", _pwm);
      break;
    case MOTOR_RUNNING:
      // Running phase
      updatePwmFactor(pulseWidth);
      int nextPwm = computePwm();
      ESP_LOGD(TAG, "Running at pwm=%d", nextPwm);
      pwm(nextPwm, t);
  }
  if (_onSample) {
    _onSample(_context, pulses, t, &_sensor);
  }
}

/*
   Interrupt service routine for motor sensor
*/
static void ARDUINO_ISR_ATTR speedSensorChanged(void* arg) {
  MotorSensor* sensor = static_cast<MotorSensor*>(arg);
  sensor->update();
}

/*
  Creates the motor sensor
*/
MotorSensor::MotorSensor(const uint8_t sensorPin)
  : _sensorPin(sensorPin),
    _direction(0) {
}

/*
  Initializes motor sensor
*/
void MotorSensor::begin() {
  pinMode(_sensorPin, INPUT);
  _speedometer.reset(millis());
  attachInterruptArg(_sensorPin, &speedSensorChanged, this, CHANGE);
}

/*
   Resets the motor sensor
*/
void MotorSensor::reset(const unsigned long timestamp) {
  _direction = 0;
  _pulses = 0;
  _speedometer.reset(timestamp);
}

/*
  Sets the direction of motor
  @param direction > 0 if forward, < 0 if backward
*/
void MotorSensor::direction(const int direction) {
  ESP_LOGD(TAG, "MotorSensor::setDirection %d", direction);

  _direction = direction;
  if (direction == 0) {
    _speedometer.reset(millis());
  }
}

/*
  Polls the motoro sensor
*/
void MotorSensor::polling(const unsigned long clockTime) {
  // Computes the dPulse
  const long pulses = _pulses;
  const long dPulses = pulses - _lastPulses;
  _lastPulses = pulses;
  update(dPulses, clockTime);
}

/*
   Updates the pulse counter (called by interupt service routine)
*/
void MotorSensor::update(void) {
  if (_direction > 0) {
    _pulses++;
  } else if (_direction < 0) {
    _pulses--;
  }
}

/*
  Updates the motor sensor
  Computes the speed and invokes the callback function
*/
void MotorSensor::update(const int dPulses, const unsigned long clockTime) {
  _speedometer.update(clockTime, dPulses);
  if (_onSample && dPulses) {
    _onSample(_context, dPulses, clockTime, this);
  }
}

/*
   Creates the speedometer
*/
Speedometer::Speedometer()
  : _tau(DEFAULT_TAU) {
}

/*
   Updates the speedometer
   @param clockTime the instant
   @param dpulse the number of pulses
*/
void Speedometer::update(const unsigned long clockTime, const int dpulse) {
  const long dt = clockTime - _prevTime;
  if (dt > _tau) {
    _pps = 1000.0 * dpulse / dt;
  } else {
    _pps = (1000.0 * dpulse + _pps * (_tau - dt)) / _tau;
  }
  _prevTime = clockTime;
}

void Speedometer::reset(const unsigned long timestamp) {
  _pps = 0;
  _prevTime = timestamp;
}
