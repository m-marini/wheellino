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

#ifndef LidarServo_h
#define LidarServo_h

#include "Arduino.h"
#include <ESP32Servo.h>

class LidarServo;

typedef void (*OnPositionCallBack_t)(void *context, LidarServo &servo);

/*
   Proxy sensor
*/
class LidarServo {
private:
  const uint8_t _servoPin;
  Servo _servo;
  int _direction;
  int _toDirection;
  int _offset;
  boolean _moving;
  unsigned long _lastPoll;
  unsigned long _positionTime;
  unsigned long _resetTime;
  float _a;
  float _b;
  OnPositionCallBack_t _onPosition;
  void *_context;

  /**
       Moves the servo to position the sensor
    */
  void moveServo(const unsigned long t0);

  /**
       Returns the interpolated direction (DEG) at dt time to position
       @param dt the time to position (ms)
    */
  const int direction(const unsigned long dt);

public:
  LidarServo(const uint8_t servoPin);

  /**
       Begins the sensor
    */
  void begin(void);

  /*
       Sets callback on reached
    */
  void onPosition(OnPositionCallBack_t callback, void *context = NULL) {
    _onPosition = callback;
    _context = context;
  }

  /*
       Sets the angle
       @param angle the angle in (DEG)
    */
  void direction(const int value, const unsigned long t0);

  /*
       Polls
    */
  void polling(const unsigned long t0);

  /*
       Sets the offset
    */
  void offset(const int value) {
    _offset = value;
  }

  /**
       Returns the servo direction
    */
  const int direction(void) const {
    return _direction;
  }

  /**
       Returns the servo direction
    */
  const int targetDirection(void) const {
    return _toDirection;
  }

  /**
  * Returns true if the servo is moving
  */
  const bool moving() const {
    return _moving;
  }
};

#endif
