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
static const char* TAG = "ProxySensor";

#include "ProxySensor.h"

static const unsigned long RESET_INTERVAL = 3000ul;
static const unsigned long INACTIVITY = 50ul;
static const unsigned long INACTIVITY_MICROS = INACTIVITY * 1000;
static const int NO_SAMPLES = 3;
static const float DEG_PER_MILLIS = 0.315;  // Max 0.333

/**
   Computes the spline coefficents
*/
static void spline(float& a, float& b, unsigned long& time, const int from, const int to) {
  float v = to >= from ? DEG_PER_MILLIS : -DEG_PER_MILLIS;
  int dy = to - from;
  time = (unsigned long)round(1.5 * dy / v);
  a = -16 * v * v * v / 27 / dy / dy;
  b = -4 * v * v / 3 / dy;
  ESP_LOGD(TAG, "ProxySensor::spline from=%d, to=%d", from, to);
  ESP_LOGD(TAG, "  dy=%d, v=%f", dy, v);
  ESP_LOGD(TAG, "  time=%lu, a=%.6f, b=%.6f", time, a, b);
}

/**
   Creates the proxy sensor
*/
ProxySensor::ProxySensor(const uint8_t servoPin, const uint8_t triggerPin, const uint8_t echoPin)
  : _servoPin(servoPin),
    _triggerPin(triggerPin),
    _echoPin(echoPin),
    _interval(INACTIVITY) {
}

/**
   Begins the sensor
*/
void ProxySensor::begin(void) {
  ESP_LOGD(TAG, "ProxySensor::begin");
  _servo.attach(_servoPin);
  _servo.write(90 - _direction - _offset);
  pinMode(_echoPin, INPUT);
  pinMode(_triggerPin, OUTPUT);
  /* Computes the time to position and scan */
  unsigned long t0 = millis();
  _pingTime = t0 + (unsigned long)round(90 / DEG_PER_MILLIS);
  _positionTime = _lastPoll = t0;
}

/**
   Sets the measure interval
*/
void ProxySensor::interval(const unsigned long interval) {
  _interval = max(interval, INACTIVITY);
}

/*
   Polls
*/
void ProxySensor::polling(const unsigned long t0) {
  if (t0 > _lastPoll) {
    /* Checks for reset timeout*/
    if (_resetTime != 0 && t0 >= _resetTime) {
      /* reset timed out */
      direction(0, t0);
      _resetTime = 0;
    } else if (_moving) { /* Checks for moving */
      moveServo(t0 < _positionTime ? _positionTime - t0 : 0);
    } else if (t0 >= _pingTime) { /* Check for scan time */
      ping(t0);
    }
    _lastPoll = t0;
  }
}

/**
   Scans for echo
*/
void ProxySensor::ping(const unsigned long t0) {
  /* Samples the echo sensor */
  digitalWrite(_triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_triggerPin, LOW);
  delayMicroseconds(2);
  unsigned long duration = pulseIn(_echoPin, HIGH, INACTIVITY_MICROS);
  ESP_LOGD(TAG, "ProxySensor::polling duration: %lu", duration);

  /* Update the measures */
  _noMeasures++;
  /* Checks for valid measure */
  if (duration > 0 || duration < INACTIVITY_MICROS) {
    _totalDuration += duration;
    _noValidSamples++;
  }
  /* Checks for last measure */
  if (_noMeasures >= NO_SAMPLES) {
    /* averages the measure and latch it */
    _echoTime = t0;
    _echoDirection = _direction;
    _echoDelay = _noValidSamples > 0 ? _totalDuration / _noValidSamples : 0;
    /* Resets the measures */
    _noValidSamples = _noMeasures = 0;
    _totalDuration = 0;
    /* set the wait timeout for next measure */
    _pingTime = t0 + _interval;
    /* Signals for measure */
    if (_onDataReady) {
      _onDataReady(_context, *this);
    }
  } else {
    /* Next measure */
    _pingTime = t0 + INACTIVITY;
  }
}

/**
   Moves the servo to position the sensor
*/
void ProxySensor::moveServo(const unsigned long dt) {
  /* Computes and move the direction of sensor */
  _direction = direction(dt);
  int wr = min(max(0, 90 - _direction - _offset), 180);
  ESP_LOGD(TAG, "ProxySensor::moveServo dt: %lu, dir=%d, wr: %d", dt, _direction, wr);
  _servo.write(wr);

  /* Set moving sensor if not yet in position */
  _moving = dt > 0;
}

/**
   Returns the interpolated direction (DEG) at dt time to position
   @param dt the time to position (ms)
*/
const int ProxySensor::direction(const unsigned long dt) {
  float dir = -_a * dt * dt * dt + _b * dt * dt + _toDirection;
  ESP_LOGD(TAG, "ProxySensor::direction dt=%lu, dir=%.3f", dt, dir);
  return (int)round(dir);
}

/*
   Sets the sensor direction
   @param angle the direction in (DEG)
*/
void ProxySensor::direction(const int value, const unsigned long t0) {
  ESP_LOGD(TAG, "ProxySensor::direction dir=%d", value);
  if (_toDirection != value) { /* Checks for direction change */
    _toDirection = value;
    /* Computes the spline parameters */
    unsigned long timeToPosition;
    spline(_a, _b, timeToPosition, _direction, _toDirection);
    /* Computes the position instant */
    _positionTime = t0 + timeToPosition;
    /* Computes the scan instant */
    _pingTime = max(_pingTime, _positionTime);
    /* Reset measures */
    _noMeasures = _noValidSamples = 0;
    _totalDuration = 0;
    _moving = true;
  }
  /* Computes the next reset instant */
  _resetTime = t0 + RESET_INTERVAL;
}
