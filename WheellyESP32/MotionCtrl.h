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

#ifndef Motion_h
#define Motion_h

#include "MotorCtrl.h"
#include "Timer.h"

#define PULSES_PER_ROOT 40
#define WHEEL_DIAMETER 0.067f

#define DISTANCE_PER_PULSE (WHEEL_DIAMETER * PI / PULSES_PER_ROOT)

/*
  Motion sensor detects the movement from left and right motors
*/
class MotionSensor {
private:
  MotorCtrl& _leftMotor;
  MotorCtrl& _rightMotor;
  boolean _updateAngle;
  int _angle;
  float _xPulses;
  float _yPulses;
  int _dl;
  int _dr;
  void (*_onChange)(void*, const unsigned long, MotionSensor&);
  void* _context;

  void update(const unsigned long clockTime);

public:
  /*
       Creates the sensor
    */
  MotionSensor(MotorCtrl& leftMotor, MotorCtrl& rightMotor);

  /*
       Initializes the sensor
    */
  void begin(void) {}

  /*
       Polls the sensor
    */
  void polling(const unsigned long clockTime);

  /*
       Set the motor directions
       @param leftForward left speed
       @param rightForward left speed
    */
  void direction(const int leftForward, const int rightForward);

  /*
       Set the callback on change
    */
  void setOnChange(void (*callback)(void* context, const unsigned long clockTime, MotionSensor& sensor), void* context = NULL);

  /*
       Sets the tau parameter
    */
  void tau(const unsigned long tau);

  /*
       Sets the direction angle
    */
  void angle(const int angle) {
    _angle = angle;
  }
  /*
       Resets the sensor
    */
  void reset(const unsigned long timestamp);

  /*
       Sets left pulses
       @param dPulse the number of pulse
    */
  void setLeftPulses(const int dPulse) {
    _dl = dPulse;
  }

  /*
       Sets left pulses
       @param dPulse the number of pulse
    */
  void setRightPulses(const int dPulse) {
    _dr = dPulse;
  }

  /*
       Returns the direction (DEG)
    */
  const int angle(void) const {
    return _angle;
  }

  /*
       Returns the x position coordinate (pulses)
    */
  const float xPulses(void) const {
    return _xPulses;
  }

  /*
       Returns the y position coordinate (pulses)
    */
  const float yPulses(void) const {
    return _yPulses;
  }

  /*
       Returns the right pulses counter
    */
  const long rightPulses(void) const {
    return _rightMotor.pulses();
  }


  /*
       Returns the left pulses counter
    */
  const long leftPulses(void) const {
    return _leftMotor.pulses();
  }

  /*
       Returns the left speed (pps)
    */
  const float leftPps(void) const {
    return _leftMotor.pps();
  }

  /*
       Returns the right speed (pps)
    */
  const float rightPps(void) const {
    return _rightMotor.pps();
  }

  /*
       Updates the angle
    */
  void updateAngle(const boolean updateAngle) {
    _updateAngle = updateAngle;
  }

  const unsigned long tau(void) const;
};

enum MotionStatus {
  HALT = 0,
  FORWARD = 1,
  BACKWARD = 2,
  ROTATING = 3
};

/**
 Motion configuration parameters
*/
typedef struct {
  unsigned int minRotRange;
  unsigned int maxRotRange;
  unsigned int maxRotPps;
  unsigned int maxSpeed;
  unsigned int haltDistance;
  unsigned int decelerateDistance;
} motionConfig_t;

/*
   Handles the two motors power (left and right) to drive for selected speed and direction
*/
class MotionCtrlClass {
private:
  MotorCtrl _leftMotor;
  MotorCtrl _rightMotor;
  MotionSensor _sensors;
  Timer _stopTimer;
  Timer _checkTimer;
  motionConfig_t _config;

  MotionStatus _status;
  int _direction;
  int _xTarget;
  int _yTarget;

  unsigned long _prevTime;

  void motorSpeed(const int left, const int right);

  /*
       Handles the motion
    */
  void handleMotion(const unsigned long clockTime);

  /*
       Handles the motion
    */
  void handleForward(void);

  /*
       Handles the motion
    */
  void handleBackward(void);

  /*
       Handles the motion
    */
  void handleRotation(void);

public:
  /*
       Create the motion controller
    */
  MotionCtrlClass(const uint8_t leftForwPin, const uint8_t leftBackPin,
                  const uint8_t rightForwPin, const uint8_t rightBackPin,
                  const uint8_t leftSensorPin, const uint8_t rightSensorPin);

  /*
       Initializes the motion controller
    */
  void begin(void);

  /*
       Polls the controller
    */
  void polling(const unsigned long clockTime = millis());

  /*
       Resets the controller
    */
  void reset(const unsigned long timestamp);

  /*
    Halts the motion
    */
  void halt(void);

  /*
   Moves the robot forward to the target position
   @param xTarget the x target (pulses)
   @param yTarget the y target (pulses)
  */
  void forward(const int xTarget, const int yTarget);

  /*
   Moves the robot backward to the target position
   @param xTarget the x target (pulses)
   @param yTarget the y target (pulses)
  */
  void backward(const int xTarget, const int yTarget);

  /*
       Moves to given direction at given speed
       @param direction the direction (DEG)
    */
  void rotate(const int direction);

  /*
       Sets the configuration parameters
    */
  void config(const motionConfig_t& config);

  /*
       Sets the tau parameter
    */
  void tau(const unsigned long tau) {
    _sensors.tau(tau);
  }

  /*
   Returns the status
  */
  const MotionStatus status(void) const {
    return _status;
  }

  /*
       Returns true if halt
    */
  const boolean isHalt(void) const {
    return _status == MotionStatus::HALT;
  }

  /*
       Returns the x position pulses
    */
  const float xPulses(void) const {
    return _sensors.xPulses();
  }

  /*
       Returns the y position pulses
    */
  const float yPulses(void) const {
    return _sensors.yPulses();
  }

  /*
       Returns the direction angle (DEG)
    */
  const int angle(void) const {
    return _sensors.angle();
  }

  /*
       Returns the left speed (pps)
    */
  const float leftPps(void) const {
    return _sensors.leftPps();
  }

  /*
       Returns the left speed (pps)
    */
  const float rightPps(void) const {
    return _sensors.rightPps();
  }

  /*
       Returns true if moving forward
    */
  const boolean isForward(void) const;

  /*
       Returns true if moving backward
    */
  const boolean isBackward(void) const;

  /*
       Returns the x target position
   */
  const int xTarget(void) const {
    return _xTarget;
  }

  /*
      Returns the y target position
   */
  const int yTarget(void) const {
    return _yTarget;
  }

  /*
       Sets current direction angle (DEG)
       (used with MPU)
    */
  void angle(const int angle) {
    _sensors.angle(angle);
  }

  /*
       Returns the expected direction (DEG)
    */
  const int direction(void) const {
    return _direction;
  }

  /*
       Returns the left motor controller
    */
  MotorCtrl& leftMotor(void) {
    return _leftMotor;
  }

  /*
       Returns the right motor controlller
    */
  MotorCtrl& rightMotor(void) {
    return _rightMotor;
  }

  /**
   Returns the configuration parameters
   */
  const motionConfig_t& config(void) const {
    return _config;
  }

  /**
       Returns the motion sensors
    */
  MotionSensor& sensors() {
    return _sensors;
  }
};

#endif
