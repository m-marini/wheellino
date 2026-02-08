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
static const unsigned ROOT_DEG = 360;
static const unsigned long MILLIS_PER_ROOT = 1143;  // ~= 0.315 DEG/millis

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
  _servo.write(90 - _toDirection - _offset);
  /* Computes the time to position and scan */
  unsigned long t0 = millis();
  _fromTime = _toTime = t0;
}

/*
   Polls
*/
void LidarServo::polling(const unsigned long t0) {
  if (t0 > _lastPoll) {
    /* Checks for reset timeout*/
    if (_resetTime > 0 && t0 >= _resetTime) {
      /* reset timed out */
      ESP_LOGD(TAG, "Resetting ...");
      direction(0, t0);
      _resetTime = 0;
    } else if (t0 >= _toTime) {
      // Head in position
      if (_moving) {
        // Head reached the position
        ESP_LOGD(TAG, "Position reached");
        _direction = _toDirection;
        _moving = false;
        if (_onPosition) {
          _onPosition(_context, *this);
        }
      }
    } else {
      // Head not in position
      // Computes current position
      int ox = _direction;
      _direction = _toTime > _fromTime ? map(t0, _fromTime, _toTime, _fromDirection, _toDirection) : _toDirection;
    }
    _lastPoll = t0;
  }
}

/*
   Sets the sensor direction
   @param angle the direction in (DEG)
*/
void LidarServo::direction(const int angle, const unsigned long t0) {
  ESP_LOGD(TAG, "direction=%d", angle);
  _resetTime = t0 + RESET_INTERVAL;
  _moving = true;
  if (_direction != angle) {
    // Compute the time to reach the target
    unsigned long dt = abs(angle - _fromDirection) * MILLIS_PER_ROOT / ROOT_DEG;
    _fromDirection = _direction;
    _fromTime = t0;
    _toTime = t0 + dt;
    _toDirection = angle;

    // Computes the servo angle command
    int wr = min(max(0, 90 - _toDirection - _offset), 180);
    ESP_LOGD(TAG, "dir=%d, wr: %d", _toDirection, wr);
    _servo.write(wr);
  } else {
    ESP_LOGD(TAG, "Head already in position");
    _fromDirection = _toDirection = angle;
    _fromTime = _toTime = t0;
  }
}
