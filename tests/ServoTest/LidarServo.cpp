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

#include "LidarServo.h"

#include <esp_log.h>
static const char* TAG = "WheellySensor";

static const unsigned long RESET_INTERVAL = 3000ul;
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
  ESP_LOGD(TAG, "from=%d, to=%d", from, to);
  ESP_LOGD(TAG, "  dy=%d, v=%f", dy, (double)v);
  ESP_LOGD(TAG, "  time=%lu, a=%.6f, b=%.6f", time, (double)a, (double)b);
}

/**
   Creates the proxy sensor
*/
LidarServo::LidarServo(const uint8_t servoPin)
  : _servoPin(servoPin) {
}

/**
   Begins the sensor
*/
void LidarServo::begin(void) {
  ESP_LOGI(TAG, "Begin");
  _servo.attach(_servoPin);
  _servo.write(90 - _direction - _offset);
  /* Computes the time to position and scan */
  unsigned long t0 = millis();
  _positionTime = _lastPoll = t0;
}

/*
   Polls
*/
void LidarServo::polling(const unsigned long t0) {
  if (t0 > _lastPoll) {
    /* Checks for reset timeout*/
    if (_resetTime != 0 && t0 >= _resetTime) {
      /* reset timed out */
      ESP_LOGD(TAG, "Resetting ...");
      direction(0, t0);
      _resetTime = 0;
    } else if (_moving) { /* Checks for moving */
      ESP_LOGD(TAG, "Moving ...");
      moveServo(t0 < _positionTime ? _positionTime - t0 : 0);
    }
    _lastPoll = t0;
  }
}

/**
   Moves the servo to position the sensor
*/
void LidarServo::moveServo(const unsigned long dt) {
  /* Computes and move the direction of sensor */
  _direction = direction(dt);
  int wr = min(max(0, 90 - _direction - _offset), 180);
  ESP_LOGD(TAG, "dt: %lu, dir=%d, wr: %d", dt, _direction, wr);
  _servo.write(wr);

  /* Set moving sensor if not yet in position */
  boolean wasMoving = _moving;
  _moving = dt > 0;
  if (wasMoving && !_moving && _onPosition) {
    _onPosition(_context, *this);
  }
}

/**
   Returns the interpolated direction (DEG) at dt time to position
   @param dt the time to position (ms)
*/
const int LidarServo::direction(const unsigned long dt) {
  float dir = -_a * dt * dt * dt + _b * dt * dt + _toDirection;
  ESP_LOGD(TAG, "dt=%lu, dir=%.3f", dt, (double)dir);
  return (int)round(dir);
}

/*
   Sets the sensor direction
   @param angle the direction in (DEG)
*/
void LidarServo::direction(const int value, const unsigned long t0) {
  ESP_LOGD(TAG, "dir=%d", value);
  if (_toDirection != value) { /* Checks for direction change */
    _toDirection = value;
    /* Computes the spline parameters */
    unsigned long timeToPosition;
    spline(_a, _b, timeToPosition, _direction, _toDirection);
    /* Computes the position instant */
    _positionTime = t0 + timeToPosition;
    _moving = true;
  } else if (_onPosition) {
    _onPosition(_context, *this);
  }
  /* Computes the next reset instant */
  _resetTime = t0 + RESET_INTERVAL;
}
