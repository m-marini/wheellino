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

#define DEFAULT_MIN_ROT_RANGE 3
#define DEFAULT_MAX_ROT_RANGE 30
#define DEFAULT_MAX_ANGULAR_VALUE 10
#define DEFAULT_MAX_SPEED 60
#define DEFAULT_HALT_DISTANCE 20
#define DEFAULT_DECELERATE_DISTANCE 200

#define DEFAULT_P0 27
#define DEFAULT_P1 73
#define DEFAULT_MU 64
#define DEFAULT_AX 1270
#define DEFAULT_H DEFAULT_MU
#define MAX_SPEED 60

/*
  Creates the motion controller
*/
MotionCtrlClass::MotionCtrlClass(const uint8_t leftForwPin, const uint8_t leftBackPin, const uint8_t rightForwPin, const uint8_t rightBackPin, const uint8_t leftSensorPin, const uint8_t rightSensorPin)
  : _leftMotor(leftForwPin, leftBackPin, leftSensorPin),
    _rightMotor(rightForwPin, rightBackPin, rightSensorPin),
    _sensors(_leftMotor, _rightMotor),
    _config({ .minRotRange = DEFAULT_MIN_ROT_RANGE,
              .maxRotRange = DEFAULT_MAX_ROT_RANGE,
              .maxRotPps = DEFAULT_MAX_ANGULAR_VALUE,
              .maxSpeed = DEFAULT_MAX_SPEED,
              .haltDistance = DEFAULT_HALT_DISTANCE,
              .decelerateDistance = DEFAULT_DECELERATE_DISTANCE }),
    _status(MotionStatus::HALT) {

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
  ESP_LOGD(TAG, "Reset");
  _sensors.reset(timestamp);
}

/*
  Halt the robot
*/
void MotionCtrlClass::halt() {
  ESP_LOGD(TAG, "Halt");
  _status = HALT;
  motorSpeed(0, 0);
  _stopTimer.stop();
  _checkTimer.stop();
}

/*
  Sets the movement direction and speed
*/
void MotionCtrlClass::rotate(const int direction) {
  ESP_LOGD(TAG, "Rotation %d DEG", direction);
  _direction = direction;
  if (isHalt()) {
    _status = ROTATING;
    _stopTimer.start();
    _checkTimer.start();
    handleMotion(millis());
  } else {
    _stopTimer.restart();
  }
}

/*
  Sets the forward movement
*/
void MotionCtrlClass::forward(const int xTarget, const int yTarget) {
  ESP_LOGD(TAG, "Forward to %d, %d", xTarget, yTarget);
  _xTarget = xTarget;
  _yTarget = yTarget;
  if (isHalt()) {
    _status = FORWARD;
    _stopTimer.start();
    _checkTimer.start();
    handleMotion(millis());
  } else {
    _status = FORWARD;
    _stopTimer.restart();
  }
}

/*
  Sets the backward movement
*/
void MotionCtrlClass::backward(const int xTarget, const int yTarget) {
  ESP_LOGD(TAG, "Backward to %d,%d", xTarget, yTarget);
  _xTarget = xTarget;
  _yTarget = yTarget;
  if (isHalt()) {
    _status = BACKWARD;
    _stopTimer.start();
    _checkTimer.start();
    handleMotion(millis());
  } else {
    _status = BACKWARD;
    _stopTimer.restart();
  }
  ESP_LOGD(TAG, "  status=%d", _status);
}

/*
  Configures the controller
  @param p the controller parameters
*/
void MotionCtrlClass::config(const motionConfig_t& cfg) {
  _config = cfg;
}

/*

*/
void MotionCtrlClass::polling(unsigned long clockTime) {
  _stopTimer.polling(clockTime);
  _checkTimer.polling(clockTime);
  _leftMotor.polling(clockTime);
  _rightMotor.polling(clockTime);
  _sensors.polling(clockTime);
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
void MotionCtrlClass::handleMotion(const unsigned long clockTime) {
  unsigned long dt = clockTime - _prevTime;
  _prevTime = clockTime;
  if (dt > 0) {
    ESP_LOGD(TAG, "handleMotion %lu status=%d", clockTime, _status);
    // motion not yet handled
    switch (_status) {
      case FORWARD:
        handleForward();
        break;
      case BACKWARD:
        handleBackward();
        break;
      case ROTATING:
        handleRotation();
        break;
      default:
        break;
    }
  }
}

/**
 Returns the rotation speed
 @param angle rotation angle (DEG)
 @param config motion configuration
*/

static int computeRotSpeed(const int angle, const motionConfig_t& config) {
  int speed = config.maxRotPps;
  const int absTurn = abs(angle);
  if (absTurn <= config.maxRotRange) {
    // Approching target direction
    // Compute approaching speed
    ESP_LOGD(TAG, "Approaching target direction %d DEG (actual %d DEG)", _direction, dir1);
    speed = map(absTurn, 0, config.maxRotRange, 0, config.maxRotPps);
  }
  ESP_LOGD(TAG, "Speed %d", speed);
  return angle < 0 ? -speed : speed;
}

/**
  Computes the left and right motor speed from linera speed and rotation speed
  @param left left motor speed output value
  @param right motor speed output value
  @param speed the linear speed (pps)
  @param rotSpeed the rotation spped (pps)
*/
static void computeMotorSpeed(int& left, int& right, const int speed, const int rotSpeed) {
  left = 0;
  right = 0;
  int max = abs(speed) + abs(rotSpeed);
  int rel = min(max, MAX_SPEED);
  left = (speed + rotSpeed) * rel / max;
  right = (speed - rotSpeed) * rel / max;
}

/*
 Handles the forward motion
*/
void MotionCtrlClass::handleForward(void) {
  // Computes the the distance from target (pulses)
  float xRobot = xPulses();
  float yRobot = yPulses();
  float dx = _xTarget - xRobot;
  float dy = _yTarget - yRobot;
  int d = round(sqrt(dx * dx + dy * dy));
  if (d <= _config.haltDistance) {
    // Reached target position
    ESP_LOGI(TAG, " Reached target position");
    halt();
    return;
  }
  // Compute the direction to the target
  int toDir = round(atan2f(dx, dy) * 180 / PI);
  int dir1 = angle();
  int turn = normalDeg(toDir - dir1);
  int absTurn = abs(turn);
  ESP_LOGD(TAG, " Target at R%d, turn by %d DEG", toDir, turn);
  // Compute motor speeds
  int linSpeed;
  if (absTurn > _config.maxRotRange) {
    // Rotate to target position
    linSpeed = 0;
    ESP_LOGD(TAG, " Rotate by %d", turn);
  } else if (d < _config.decelerateDistance) {
    // decelerate to target position
    ESP_LOGD(TAG, " Decelerate");
    linSpeed = _config.maxSpeed * d / _config.decelerateDistance;
  } else {
    ESP_LOGD(TAG, " Move");
    // move at max speed
    linSpeed = _config.maxSpeed;
  }
  const int rotSpeed = computeRotSpeed(turn, _config);
  ESP_LOGD(TAG, " Speed lin=%d, rot=%d", linSpeed, rotSpeed);
  int leftSpeed;
  int rightSpeed;
  computeMotorSpeed(leftSpeed, rightSpeed, linSpeed, rotSpeed);
  motorSpeed(leftSpeed, rightSpeed);
  ESP_LOGD(TAG, " Speed %d, %d", leftSpeed, rightSpeed);
}

/*
 Handles the backward motion
*/
void MotionCtrlClass::handleBackward(void) {
  float dx = _xTarget - xPulses();
  float dy = _yTarget - yPulses();
  int d = round(sqrt(dx * dx + dy * dy));
  if (d <= _config.haltDistance) {
    // Reached target position
    ESP_LOGI(TAG, "Reached target position");
    halt();
    return;
  }
  // Compute the direction from the target
  int toDir = round(atan2f(dx, dy) * 180 / PI);
  int dir1 = angle();
  int turn = normalDeg(toDir - dir1 - 180);
  int absTurn = abs(turn);
  ESP_LOGD(TAG, "Target R%d D%d", toDir, d);
  // Compute motor speeds
  int linSpeed;
  if (absTurn > _config.maxRotRange) {
    // Rotate to target position
    ESP_LOGD(TAG, "Rotate");
    linSpeed = 0;
  } else if (d < _config.decelerateDistance) {
    // decelerate to target position
    ESP_LOGD(TAG, "Decelerate");
    linSpeed = -_config.maxSpeed * d / _config.decelerateDistance;
  } else {
    ESP_LOGD(TAG, "Move");
    // move at max speed
    linSpeed = -_config.maxSpeed;
  }
  const int rotSpeed = computeRotSpeed(turn, _config);
  ESP_LOGD(TAG, "Speed %d, %d", linSpeed, rotSpeed);
  int leftSpeed;
  int rightSpeed;
  computeMotorSpeed(leftSpeed, rightSpeed, linSpeed, rotSpeed);
  motorSpeed(leftSpeed, rightSpeed);
}

/*
  Handles the rotation
*/
void MotionCtrlClass::handleRotation(void) {
  int dir1 = angle();
  int toDir1 = _direction;
  int turn = normalDeg(toDir1 - dir1);
  int absTurn = abs(turn);
  if (absTurn <= _config.minRotRange) {
    // Reached target direction
    ESP_LOGI(TAG, "Reached target direction %d DEG (actual %d DEG)", _direction, dir1);
    halt();
    return;
  }
  ESP_LOGI(TAG, "Target direction %d DEG (actual %d DEG)", _direction, dir1);
  const int rotSpeed = computeRotSpeed(turn, _config);
  ESP_LOGI(TAG, "Speed %d", rotSpeed);
  int leftSpeed;
  int rightSpeed;
  computeMotorSpeed(leftSpeed, rightSpeed, 0, rotSpeed);
  motorSpeed(leftSpeed, rightSpeed);
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
static void handleLeftSensor(void* context, int dPulse, unsigned long, MotorSensor*) {
  ESP_LOGD(TAG, "MotionSensor::handleLeftSensor %d", dPulse);
  ((MotionSensor*)context)->setLeftPulses(dPulse);
}

/*

*/
static void handleRightSensor(void* context, int dPulse, unsigned long, MotorSensor*) {
  ESP_LOGD(TAG, "MotionSensor::handleRightSensor %d", dPulse);
  ((MotionSensor*)context)->setRightPulses(dPulse);
}

/*

*/
MotionSensor::MotionSensor(MotorCtrl& leftMotor, MotorCtrl& rightMotor)
  : _leftMotor(leftMotor), _rightMotor(rightMotor),
    _updateAngle(false) {
  _leftMotor.onSample(handleLeftSensor, this);
  _rightMotor.onSample(handleRightSensor, this);
}

void MotionSensor::reset(unsigned long timestamp) {
  _leftMotor.reset(timestamp);
  _rightMotor.reset(timestamp);
  _xPulses = 0;
  _yPulses = 0;
  _angle = 0;
}

void MotionSensor::tau(const unsigned long tau) {
  _leftMotor.tau(tau);
  _rightMotor.tau(tau);

  ESP_LOGD(TAG, "MotionCtrl::tau %lu", tau);
}

void MotionSensor::direction(int left, int right) {
  ESP_LOGD(TAG, "MotionSensor::setDirection %d,%d", left, right);

  _leftMotor.sensorDirection(left);
  _rightMotor.sensorDirection(right);
}

void MotionSensor::polling(unsigned long clockTime) {
  update(clockTime);
}

void MotionSensor::update(const unsigned long clockTime) {
  ESP_LOGD(TAG, "MotionSensor::update %d, %d", _dl, _dr);

  if (_dl != 0 || _dr != 0) {
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
  _dl = _dr = 0;
}

/*

*/
void MotionSensor::setOnChange(void (*callback)(void*, const unsigned long, MotionSensor&), void* context) {
  _onChange = callback;
  _context = context;
}
