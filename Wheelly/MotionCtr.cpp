#include "MotionCtrl.h"

//#define DEBUG
#include "debug.h"
#include "Utils.h"
#include "Fuzzy.h"

#define SCALE 256
#define MAX_ANGULAR_VALUE 10
#define SPEED_THRESHOLD   0.5f

#define MOTOR_SAFE_INTERVAL 1000ul
#define MOTOR_CHECK_INTERVAL 100ul

#define MOVE_ROT_THRESHOLD 30
#define MIN_ROT_RANGE 3
#define MAX_ROT_RANGE 30

#define DEFAULT_P0 27
#define DEFAULT_P1 73
#define DEFAULT_NU 240
#define DEFAULT_MU 284
#define DEFAULT_H DEFAULT_MU

/*
  Creates the motion controller
*/
MotionCtrl::MotionCtrl(byte leftForwPin, byte leftBackPin, byte rightForwPin, byte rightBackPin, byte leftSensorPin, byte rightSensorPin)
  : _sensors(leftSensorPin, rightSensorPin),
    _leftMotor(leftForwPin, leftBackPin, _sensors.leftSensor()),
    _rightMotor(rightForwPin, rightBackPin, _sensors.rightSensor()),
    _moveRotThreshold(MOVE_ROT_THRESHOLD),
    _minRotRange(MIN_ROT_RANGE),
    _maxRotRange(MAX_ROT_RANGE),
    _maxRotPps(MAX_ANGULAR_VALUE) {

  _sensors.setOnChange([](void*context, unsigned long clockTime, MotionSensor&) {
    DEBUG_PRINTLN(F("// Motor sensors triggered"));
    ((MotionCtrl *)context)-> handleMotion(clockTime);
  }, this);

  _stopTimer.interval(MOTOR_SAFE_INTERVAL);
  _stopTimer.onNext([](void *ctx, unsigned long) {
    DEBUG_PRINTLN(F("// Motor timer triggered"));
    ((MotionCtrl*)ctx)->halt();
  }, this);

  _checkTimer.interval(MOTOR_CHECK_INTERVAL);
  _checkTimer.continuous(true);
  _checkTimer.onNext([](void *ctx, unsigned long) {
    ((MotionCtrl*)ctx)->handleMotion(millis());
  }, this);
}

/*
  Initializes the motion controller
*/
void MotionCtrl::begin() {
  _leftMotor.begin();
  _rightMotor.begin();
  _sensors.begin();

  DEBUG_PRINTLN(F("// Motion controller begin"));

  halt();
}

/*
  Resets the controller
*/
void MotionCtrl::reset() {
  DEBUG_PRINTLN(F("// MotionCtrl::reset"));
  _sensors.reset();
}

/*

*/
void MotionCtrl::halt() {
  DEBUG_PRINTLN(F("// MotionCtrl::alt"));
  _speed = 0;
  _halt = true;
  power(0, 0);
  _stopTimer.stop();
  _checkTimer.stop();
}

/*

*/
void MotionCtrl::controllerConfig(int* p) {
  _moveRotThreshold = p[0];
  _minRotRange = p[1];
  _maxRotRange = p[2];
  _maxRotPps = p[3];
  DEBUG_PRINTLN(F("// MotionCtrl::setControllerConfig"));
  DEBUG_PRINT(F(", _moveRotThreshold: "));
  DEBUG_PRINT(_moveRotThreshold);
  DEBUG_PRINT(F(", _minRotRange: "));
  DEBUG_PRINT(_minRotRange);
  DEBUG_PRINT(F(", _maxRotRange "));
  DEBUG_PRINT(_maxRotRange);
  DEBUG_PRINTLN();
}

/*

*/
void MotionCtrl::move(int direction, int speed) {
  DEBUG_PRINT(F("// MotionCtrl::move "));
  DEBUG_PRINT(direction);
  DEBUG_PRINT(F(" "));
  DEBUG_PRINT(speed);
  DEBUG_PRINTLN();
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
void MotionCtrl::polling(unsigned long clockTime) {
  _sensors.polling(clockTime);
  _stopTimer.polling(clockTime);
  _checkTimer.polling(clockTime);
}

/*

*/
const boolean MotionCtrl::isForward() const {
  return _leftMotor.speed() > 0 || _rightMotor.speed() > 0
         || _sensors.leftPps() > SPEED_THRESHOLD || _sensors.rightPps() > SPEED_THRESHOLD ;
}

/*

*/
const boolean MotionCtrl::isBackward() const {
  return _leftMotor.speed() < 0 || _rightMotor.speed() < 0
         || _sensors.leftPps() < -SPEED_THRESHOLD || _sensors.rightPps() < -SPEED_THRESHOLD;
}

/*

*/
void MotionCtrl::handleMotion(unsigned long clockTime) {
  unsigned long dt = clockTime - _prevTime;
  _prevTime = clockTime;
  DEBUG_PRINT(F("// MotionCtrl::handleMotion "));
  DEBUG_PRINT(clockTime);
  DEBUG_PRINTLN();
  if (!_halt && dt > 0) {
    // Compute motor power
    int dir1 = angle();
    int toDir1 = _direction;
    int turn1 = normalDeg(toDir1 - dir1);

    DEBUG_PRINT(F("//     dir: "));
    DEBUG_PRINT(dir1);
    DEBUG_PRINT(F(", to: "));
    DEBUG_PRINT(toDir1);
    DEBUG_PRINT(F(", turn: "));
    DEBUG_PRINT(turn1);
    DEBUG_PRINTLN();

    int rotRange = _maxRotRange - _minRotRange;

    float isCw = fuzzyPositive(turn1 - _minRotRange, rotRange);
    float isCcw = fuzzyPositive(-turn1 - _minRotRange, rotRange);
    float isLin = 1 - fuzzyPositive(abs(turn1), _moveRotThreshold);
    Fuzzy fuzzy;

    fuzzy.add(_maxRotPps, isCw);
    fuzzy.add(-_maxRotPps, isCcw);
    fuzzy.add(0, 1 - max(isCw, isCcw));
    int cwSpeed = round(fuzzy.defuzzy());

    fuzzy.reset();
    fuzzy.add(_speed, isLin);
    fuzzy.add(0, 1 - isLin);
    int linSpeed = round(fuzzy.defuzzy());

    DEBUG_PRINT(F("//     isCw: "));
    DEBUG_PRINT(isCw);
    DEBUG_PRINT(F(", isCcw: "));
    DEBUG_PRINT(isCcw);
    DEBUG_PRINT(F(", isLin: "));
    DEBUG_PRINT(isLin);
    DEBUG_PRINT(F(", cwSpeed: "));
    DEBUG_PRINT(cwSpeed);
    DEBUG_PRINT(F(", linSpeed: "));
    DEBUG_PRINT(linSpeed);
    DEBUG_PRINTLN();

    int left = linSpeed + cwSpeed;
    int right = linSpeed - cwSpeed;

    /*
      float mx = max(max(abs(left), abs(right)), 1);

      left /= mx;
      right /= mx;
    */

    DEBUG_PRINT(F("//     motors: "));
    DEBUG_PRINT(left);
    DEBUG_PRINT(F(", "));
    DEBUG_PRINT(right);
    DEBUG_PRINTLN();
    power(left, right);
  }
}

/*

*/
void MotionCtrl::power(int left, int right) {

  DEBUG_PRINT(F("// MotionCtrl::power "));
  DEBUG_PRINT(left);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(right);
  DEBUG_PRINTLN();

  _leftMotor.speed(left);
  _rightMotor.speed(right);
}

/*
   Motor controller section
*/

MotorCtrl::MotorCtrl(byte forwPin, byte backPin, MotorSensor& sensor) :
  _forwPin(forwPin),
  _backPin(backPin),
  _sensor(sensor),
  _nu(DEFAULT_NU),
  _p0Forw(DEFAULT_P0),
  _p1Forw(DEFAULT_P1),
  _muForw(DEFAULT_MU),
  _hForw(DEFAULT_H),
  _p0Back(-DEFAULT_P0),
  _p1Back(-DEFAULT_P1),
  _muBack(DEFAULT_MU),
  _hBack(DEFAULT_H) {
}

void MotorCtrl::begin() {
  pinMode(_forwPin, OUTPUT);
  pinMode(_backPin, OUTPUT);
}

/*
   Sets the configuration parameters
   [
     nu,
     p0Forw, p1Forw, muForw, hForw,
     p0Back, p1Back, muBack, hBack
   ]
*/
void MotorCtrl::config(int* parms) {
  _nu = parms[0];
  _p0Forw = parms[1];
  _p1Forw = parms[2];
  _muForw = parms[3];
  _hForw = parms[4];
  _p0Back = parms[5];
  _p1Back = parms[6];
  _muBack = parms[7];
  _hBack = parms[8];
}

/*
   Set speed
*/
void MotorCtrl::speed(int value) {

  DEBUG_PRINT(F("// MotorCtrl::speed "));
  DEBUG_PRINT(value);
  DEBUG_PRINTLN();

  _speed = value;
  // Computes the power
  int realSpeed = round(_sensor.pps());

  DEBUG_PRINT(F("//   realSpeed: "));
  DEBUG_PRINT(realSpeed);
  DEBUG_PRINTLN();

  if (value == 0) {
    // Halt the motor
    _power = 0;
  } else if (value > 0) {
    // Move forward
    int pth = realSpeed == 0 ? _p1Forw : _p0Forw;

    long requiredPwr = ((long)_nu * (_power - pth) + SCALE * pth + _muForw * value - _hForw * realSpeed) / SCALE;

    DEBUG_PRINT(F("//   _power: "));
    DEBUG_PRINT(_power);
    DEBUG_PRINT(F(", pth: "));
    DEBUG_PRINT(pth);
    DEBUG_PRINT(F(", _nu: "));
    DEBUG_PRINT(_nu);
    DEBUG_PRINT(F(", _mu: "));
    DEBUG_PRINT(_muForw);
    DEBUG_PRINT(F(", _h: "));
    DEBUG_PRINT(_hForw);
    DEBUG_PRINT(F(", req: "));
    DEBUG_PRINT(requiredPwr);
    DEBUG_PRINTLN();

    _power = max(requiredPwr, pth);

  } else  {
    // Move backward
    int pth = realSpeed == 0 ? _p1Back : _p0Back;
    _power = min(
               (int)(((long)_nu * (_power - pth) + SCALE * pth + _muBack * value - _hBack * realSpeed) / SCALE),
               pth);
  }

  DEBUG_PRINT(F("//   _power: "));
  DEBUG_PRINT(_power);
  DEBUG_PRINTLN();

  // Applies the power
  if (_power == 0) {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, 0);
  } else if (_power > 0) {
    analogWrite(_forwPin, _power);
    analogWrite(_backPin, 0);
  } else {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, -_power);
  }
  _sensor.direction(_power);
}
