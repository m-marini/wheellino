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

#ifndef ProxySensor_h
#define ProxySensor_h

#include "Arduino.h"
#include <ESP32Servo.h>

/*
   Proxy sensor
*/
class ProxySensor {
private:
  const uint8_t _servoPin;
  const uint8_t _triggerPin;
  const uint8_t _echoPin;
  Servo _servo;
  int _direction;
  int _toDirection;
  int _offset;
  boolean _moving;
  unsigned long _interval;
  unsigned long _lastPoll;
  unsigned long _positionTime;
  unsigned long _pingTime;
  unsigned long _resetTime;
  unsigned long _echoTime;
  unsigned long _echoDelay;
  int _echoDirection;
  int _noMeasures;
  int _noValidSamples;
  unsigned long _totalDuration;
  float _a;
  float _b;
  void (*_onDataReady)(void *, ProxySensor &);
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

  /**
       Pings for echo
    */
  void ping(const unsigned long t0);

public:
  ProxySensor(const uint8_t servoPin, const uint8_t triggerPin, const uint8_t echoPin);

  /**
       Begins the sensor
    */
  void begin(void);

  /*
       Sets callback on reached
    */
  void onDataReady(void (*callback)(void *context, ProxySensor &sensor), void *context = NULL) {
    _onDataReady = callback;
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
       Sets the measure interval
    */
  void interval(const unsigned long interval);

  /**
       Returns the instant of echo signal (ms)
    */
  const unsigned long echoTime(void) const {
    return _echoTime;
  }

  /**
       Returns the direction of echo signal (DEG)
    */
  const int echoDirection(void) const {
    return _echoDirection;
  }

  /**
       Returns the delay of echo (ns)
    */
  const unsigned long echoDelay(void) const {
    return _echoDelay;
  }

  /**
       Returns the sensor direction target
    */
  const int direction(void) const {
    return _direction;
  }
};

#endif
