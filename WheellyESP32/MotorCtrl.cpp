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


#define DEFAULT_VI0 560
#define DEFAULT_VD0 100
#define DEFAULT_VX 800
#define DEFAULT_MU 20000
#define DEFAULT_AX 200
#define DEFAULT_ALPHA 50
#define DEFAULT_TAU 300ul
#define DEFAULT_VOLTAGE 2500

static unsigned long MIN_INTERVAL = 100ul;
static int MAX_PWM = 255;
static int REFERENCE_SPEED = 120;
static long ASR_SCALE = 1000;
static long ALPHA_SCALE = 100;
static long FEEDBACK_SCALE = 1000;

/*
   Creates the motor controller
*/
MotorCtrl::MotorCtrl(const uint8_t forwPin, const uint8_t backPin, MotorSensor& sensor)
  : _forwPin(forwPin),
    _backPin(backPin),
    _sensor(sensor),
    _supply(DEFAULT_VOLTAGE),
    _automatic(true),
    _tcs({ .fi0 = DEFAULT_VI0,
           .fix = DEFAULT_VX,
           .fd0 = DEFAULT_VD0,
           .fdx = DEFAULT_VX,
           .bi0 = -DEFAULT_VI0,
           .bix = DEFAULT_VX,
           .bd0 = -DEFAULT_VD0,
           .bdx = DEFAULT_VX,
           .muForw = DEFAULT_MU,
           .muBack = DEFAULT_MU,
           .alpha = DEFAULT_ALPHA,
           .ax = DEFAULT_AX }) {
}

/*
   Initializes motor controller
*/
void MotorCtrl::begin() {
  ESP_LOGI(TAG, "Begin");
  pinMode(_forwPin, OUTPUT);
  pinMode(_backPin, OUTPUT);
  _sensor.begin();
}

/**
   ASR function
*/
const long MotorCtrl::asr(const long dV, const long dt) const {
  const long px = _tcs.ax * dt / ASR_SCALE;
  return clip(dV, -px, px);
}

/*
   Sets the motor speed
*/
void MotorCtrl::speed(const int value) {
  if (value == 0
      || _speed < 0 && value > 0
      || _speed > 0 && value < 0) {
    pwm(0);
  }
  _speed = value;
}

/*
   Polls motor controller
*/
void MotorCtrl::polling(const unsigned long timestamp) {
  // Polling motor sensor
  _sensor.polling(timestamp);
  // Compute time interval from last tcs check
  const long dt = (long)(timestamp - _prevTimestamp);
  // Check for traction control system active and tcs reaction time
  if (_automatic && dt > MIN_INTERVAL) {
    // TCS check
    // Store tcs instant
    _prevTimestamp = timestamp;
    // Computes the real speed
    const int realSpeed = round(_sensor.pps());
    ESP_LOGD(TAG, "ctrl=0x%lx dt=%ld speed=%d, realSpeed=%d supply=%d voltage=%d",
             (const unsigned long)this, dt, _speed, realSpeed, _supply, _voltage);

    int vf = 0;
    // Check for motor direction
    if (_speed > 0) {
      // Move forward
      // Check for stationary or moving  motor
      const int vx = realSpeed == 0 ? _tcs.fix : _tcs.fdx;
      // Computes the theoretical voltage
      const long vt = (long)_speed * vx / REFERENCE_SPEED;
      // Compute the theoretical differential voltage
      const long dvt = vt - _voltage;
      // Computes the feedback differential voltage
      const long dvf = (long)_tcs.muForw * (_speed - realSpeed) * dt / FEEDBACK_SCALE;
      // Compute the differential voltage
      const long dv1 = ((long)(100 - _tcs.alpha) * dvt + _tcs.alpha * dvf) / 100;
      // Compute the asr differential voltage
      const long dv = asr(dv1, dt);
      // Check for stationary or moving  motor
      const int v0 = realSpeed == 0 ? _tcs.fi0 : _tcs.fd0;
      // Compute the final voltage
      int vf1 = _voltage + dv;
      vf = max(vf1, v0);
      ESP_LOGD(TAG, "v0=%d vx=%d vt=%ld dvt=%ld dvf=%ld dv1=%ld dv=%ld vf1=%d vf=%d",
               v0, vx, vt, dvt, dvf, dv1, dv, vf1, vf);
    } else if (_speed < 0) {
      // Move backward
      // Check for stationary or moving  motor
      const int vx = realSpeed == 0 ? _tcs.bix : _tcs.bdx;
      // Computes the theoretical voltage
      const long vt = (long)_speed * vx / REFERENCE_SPEED;
      // Compute the theoretical differential voltage
      const long dvt = vt - _voltage;
      // Computes the feedback differential voltage
      const long dvf = (long)_tcs.muBack * (_speed - realSpeed) * dt / FEEDBACK_SCALE;
      // Compute the differential voltage
      const long dv1 = ((long)(100 - _tcs.alpha) * dvt + _tcs.alpha * dvf) / 100;
      // Compute the asr differential voltage
      const long dv = asr(dv1, dt);
      // Check for stationary or moving  motor
      const int v0 = realSpeed == 0 ? _tcs.bi0 : _tcs.bd0;
      // Compute the final voltage
      int vf1 = _voltage + dv;
      vf = min(vf1, v0);
      ESP_LOGD(TAG, "v0=%d vx=%d vt=%ld dvt=%ld dvf=%ld dv1=%ld dv=%ld vf1=%d vf=%d",
               v0, vx, vt, dvt, dvf, dv1, dv, vf1, vf);
    }
    voltage(vf);
  }
}

/**
    Set the motor voltage
  */
void MotorCtrl::voltage(const int v) {
  _voltage = v;
  pwm(clip(MAX_PWM * v / _supply, -MAX_PWM, MAX_PWM));
}


/*
   Applies the power to the motor
   @param pwr the power -255 ... 255
*/
void MotorCtrl::pwm(const int pwm) {
  ESP_LOGD(TAG, "pwm=%d", pwm);
  _pwm = pwm;
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
  _sensor.direction(pwm);
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
  if (_onSample != NULL && dPulses != 0) {
    _onSample(_context, dPulses, clockTime, *this);
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
  const unsigned long dt = clockTime - _prevTime;
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
