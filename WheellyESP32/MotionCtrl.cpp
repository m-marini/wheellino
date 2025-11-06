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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE ORt
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 *    END OF TERMS AND CONDITIONS
 *
 */

#include <esp_log.h>
static char* TAG = "MotionCtrl";

#include "Arduino.h"

#include "MotionCtrl.h"

//#define MONITOR_ON
#include "num.h"
#include "pins.h"

#define SCALE 10000l
#define SPEED_THRESHOLD 0.5f

#define TRACK 0.136f

#define DEG_ANGLE_PER_PULSE (DISTANCE_PER_PULSE / TRACK * 180 / PI)
#define DEFAULT_TAU 300ul

#define MOTOR_SAFE_INTERVAL 2000ul
#define MOTOR_CHECK_INTERVAL 100ul

#define MIN_ROT_RANGE 3
#define MAX_ROT_RANGE 30
#define MAX_ANGULAR_VALUE 10

#define DEFAULT_P0 27
#define DEFAULT_P1 73
#define DEFAULT_MU 64
#define DEFAULT_AX 1270
#define DEFAULT_H DEFAULT_MU

/*
  Creates the motion controller
*/
MotionCtrlClass::MotionCtrlClass(byte leftForwPin, byte leftBackPin, byte rightForwPin, byte rightBackPin, byte leftSensorPin, byte rightSensorPin)
  : _sensors(leftSensorPin, rightSensorPin),
    _leftMotor(leftForwPin, leftBackPin, _sensors.leftSensor()),
    _rightMotor(rightForwPin, rightBackPin, _sensors.rightSensor()),
    _minRotRange(MIN_ROT_RANGE),
    _maxRotRange(MAX_ROT_RANGE),
    _maxRotPps(MAX_ANGULAR_VALUE) {

  _sensors.setOnChange([](void* context, unsigned long clockTime, MotionSensor&) {
    ESP_LOGD(TAG, "Motor sensors triggered");
    ((MotionCtrlClass*)context)->handleMotion(clockTime);
  },
                       this);

  _stopTimer.interval(MOTOR_SAFE_INTERVAL);
  _stopTimer.onNext([](void* ctx, unsigned long) {
    ESP_LOGD(TAG, "Stop motor timer triggered");
    ((MotionCtrlClass*)ctx)->halt();
  },
                    this);

  _checkTimer.interval(MOTOR_CHECK_INTERVAL);
  _checkTimer.continuous(true);
  _checkTimer.onNext([](void* ctx, unsigned long) {
    ((MotionCtrlClass*)ctx)->handleMotion(millis());
  },
                     this);
}

/*
  Initializes the motion controller
*/
void MotionCtrlClass::begin() {
  ESP_LOGI(TAG, "Begin");
  _leftMotor.begin();
  _rightMotor.begin();
  _sensors.begin();

  ESP_LOGD(TAG, "Motion controller begin");

  halt();
}

/*
  Resets the controller
*/
void MotionCtrlClass::reset(unsigned long timestamp) {
  ESP_LOGD(TAG, "MotionCtrlClass::reset");
  _sensors.reset(timestamp);
}

/*

*/
void MotionCtrlClass::halt() {
  ESP_LOGD(TAG, "MotionCtrlClass::halt");
  _speed = 0;
  _halt = true;
  motorSpeed(0, 0);
  _stopTimer.stop();
  _checkTimer.stop();
}

/*
  Configures the controller
  @param p the controller parameters
*/
void MotionCtrlClass::configController(const int* p) {
  _minRotRange = p[0];
  _maxRotRange = p[1];
  _maxRotPps = p[2];
  ESP_LOGD(TAG, "MotionCtrlClass::setControllerConfig, _minRotRange=%d, _maxRotRange=%d", _minRotRange, _maxRotRange);
}

/*
  Sets the movement direction and speed
*/
void MotionCtrlClass::move(int direction, int speed) {
  ESP_LOGD(TAG, "MotionCtrlClass::move %d, %d", direction, speed);
  _direction = direction;
  _speed = speed;
  if (_halt) {
    _halt = false;
    _stopTimer.start();
    _checkTimer.start();
    _prevTime = millis();
  } else {
    _stopTimer.restart();
    handleMotion(millis());
  }
}
/*

*/
void MotionCtrlClass::polling(unsigned long clockTime) {
  _sensors.polling(clockTime);
  _stopTimer.polling(clockTime);
  _checkTimer.polling(clockTime);
  _leftMotor.polling(clockTime);
  _rightMotor.polling(clockTime);
}

/*

*/
const boolean MotionCtrlClass::isForward() const {
  return _leftMotor.speed() > 0 || _rightMotor.speed() > 0
         || _sensors.leftPps() > SPEED_THRESHOLD || _sensors.rightPps() > SPEED_THRESHOLD;
}

/*

*/
const boolean MotionCtrlClass::isBackward() const {
  return _leftMotor.speed() < 0 || _rightMotor.speed() < 0
         || _sensors.leftPps() < -SPEED_THRESHOLD || _sensors.rightPps() < -SPEED_THRESHOLD;
}

/*

*/
void MotionCtrlClass::handleMotion(unsigned long clockTime) {
  unsigned long dt = clockTime - _prevTime;
  _prevTime = clockTime;
  ESP_LOGD(TAG, "MotionCtrlClass::handleMotion %lu", clockTime);
  if (_halt || dt <= 0) {
    return;
  }
  // Compute motor power
  int dir1 = angle();
  int toDir1 = _direction;
  int turn1 = normalDeg(toDir1 - dir1);

  ESP_LOGD(TAG, "    dir: %d, to: %d, turn: %d", dir1, toDir1, turn1);

  float isRot = fuzzyGreater(abs(turn1), _minRotRange, _maxRotRange);
  float isLin = 1 - isRot;

  int rotLeft = turn1 < 0 ? -_maxRotPps : _maxRotPps;
  int rotRight = turn1 < 0 ? _maxRotPps : -_maxRotPps;

  Defuzzier fuzzy;

  fuzzy.add(rotLeft, isRot);
  fuzzy.add(_speed, isLin);
  fuzzy.add(0, 1 - max(isRot, isLin));
  int left = round(fuzzy.defuzzy());

  fuzzy.reset();
  fuzzy.add(rotRight, isRot);
  fuzzy.add(_speed, isLin);
  fuzzy.add(0, 1 - max(isRot, isLin));
  int right = round(fuzzy.defuzzy());

  ESP_LOGD(TAG, "    motors: %d, %d", left, right);
  motorSpeed(left, right);
}

/*

*/
void MotionCtrlClass::motorSpeed(int left, int right) {

  ESP_LOGD(TAG, "MotionCtrlClass::power %d, %d", left, right);
  _leftMotor.speed(left);
  _rightMotor.speed(right);
}

/*

*/
static void handleLeftSensor(void* context, int dPulse, unsigned long, MotorSensor&) {
  ESP_LOGD(TAG, "MotionSensor::handleLeftSensor %d", dPulse);
  ((MotionSensor*)context)->setLeftPulses(dPulse);
}

/*

*/
static void handleRightSensor(void* context, int dPulse, unsigned long, MotorSensor&) {
  ESP_LOGD(TAG, "MotionSensor::handleRightSensor %d", dPulse);
  ((MotionSensor*)context)->setRightPulses(dPulse);
}

/*

*/
MotionSensor::MotionSensor(byte leftPin, byte rightPin)
  : _leftSensor(leftPin), _rightSensor(rightPin),
    _updateAngle(false) {
  _leftSensor.onSample(handleLeftSensor, this);
  _rightSensor.onSample(handleRightSensor, this);
}

void MotionSensor::begin() {
  _leftSensor.begin();
  _rightSensor.begin();
}

void MotionSensor::reset(unsigned long timestamp) {
  _leftSensor.reset(timestamp);
  _rightSensor.reset(timestamp);
  _xPulses = 0;
  _yPulses = 0;
  _angle = 0;
}

void MotionSensor::tau(unsigned long tau) {
  _leftSensor.tau(tau);
  _rightSensor.tau(tau);

  ESP_LOGD(TAG, "MotionCtrl::tau %lu", tau);
}

void MotionSensor::direction(int left, int right) {
  ESP_LOGD(TAG, "MotionSensor::setDirection %d,%d", left, right);

  _leftSensor.direction(left);
  _rightSensor.direction(right);
}

void MotionSensor::polling(unsigned long clockTime) {
  _dl = _dr = 0;
  _leftSensor.polling(clockTime);
  _rightSensor.polling(clockTime);
  if (_dl != 0 || _dr != 0) {
    update(clockTime);
  }
}

void MotionSensor::update(const unsigned long clockTime) {
  ESP_LOGD(TAG, "MotionSensor::update %d, %d", _dl, _dr);

  // Updates location
  float angle = _angle * PI / 180;
  float sa = sinf(angle);
  float ca = cosf(angle);
  float ds = ((float)(_dl + _dr)) / 2;
  _xPulses += sa * ds;
  _yPulses += ca * ds;

  ESP_LOGD(TAG, "x,y %f,%f$", _xPulses, _yPulses);

  // Updates angle
  if (_updateAngle) {
    _angle = normalDeg(_angle + roundf((_dl - _dr) * DEG_ANGLE_PER_PULSE));

    ESP_LOGD(TAG, "    angle %d", _angle);
  }

  ESP_LOGD(TAG, "pps %f, %f", leftPps(), rightPps());

  if (_onChange != NULL) {
    _onChange(_context, clockTime, *this);
  }
}

/*

*/
void MotionSensor::setOnChange(void (*callback)(void*, const unsigned long, MotionSensor&), void* context) {
  _onChange = callback;
  _context = context;
}

/*

*/
const unsigned long MotionSensor::tau(void) const {
  return (_leftSensor.tau() + _rightSensor.tau()) / 2;
}
