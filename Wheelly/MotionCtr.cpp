#include "MotionCtrl.h"

//#define DEBUG
#include "debug.h"
#include "Utils.h"
#include "Fuzzy.h"

#define MAX_VALUE 255
#define MAX_SPEED_VALUE 20

#define MOTOR_SAFE_INTERVAL 1000ul
#define MOTOR_CHECK_INTERVAL 100ul

#define MOVE_ROT_THRESHOLD 30
#define MIN_ROT_RANGE 3
#define MAX_ROT_RANGE 30

#define SIGNAL_K  464
#define PPS_K (-464)
#define POWER_K (230)

/*
  Creates the motion controller
*/
MotionCtrl::MotionCtrl(byte leftForwPin, byte leftBackPin, byte rightForwPin, byte rightBackPin, byte leftSensorPin, byte rightSensorPin)
  : _leftMotor(leftForwPin, leftBackPin),
    _rightMotor(rightForwPin, rightBackPin),
    _sensors(leftSensorPin, rightSensorPin),
    _powerK(POWER_K),
    _signalK(SIGNAL_K),
    _ppsK(PPS_K),
    _moveRotThreshold(MOVE_ROT_THRESHOLD),
    _minRotRange(MIN_ROT_RANGE),
    _maxRotRange(MAX_ROT_RANGE) {

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
void MotionCtrl::correction(int *p) {
  _leftMotor.setCorrection(p);
  _rightMotor.setCorrection(p + 4);
}

/*

*/
void MotionCtrl::controllerConfig(int* p) {
  _powerK = p[0];
  _signalK = p[1];
  _ppsK = p[2];
  _moveRotThreshold = p[3];
  _minRotRange = p[4];
  _maxRotRange = p[5];
  DEBUG_PRINTLN(F("// MotionCtrl::setControllerConfig"));
  DEBUG_PRINT(F("//     _powerK: "));
  DEBUG_PRINT(_powerK);
  DEBUG_PRINT(F(", _signalK: "));
  DEBUG_PRINT(_signalK);
  DEBUG_PRINT(F(", _ppsK: "));
  DEBUG_PRINT(_ppsK);
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
  return _speed > 0 || _left > 0 || _right > 0;
}

/*

*/
const boolean MotionCtrl::isBackward() const {
  return _speed < 0 || _left < 0 || _right < 0;
}

/*

*/
void MotionCtrl::handleMotion(unsigned long clockTime) {
  unsigned long dt = clockTime - _prevTime;
  _prevTime = clockTime;
  DEBUG_PRINT(F("// MotionCtrl::handleMotion "));
  DEBUG_PRINT(clockTime);
  DEBUG_PRINT(F(", dt: "));
  DEBUG_PRINT(dt);
  DEBUG_PRINT(F(", left: "));
  DEBUG_PRINT(_left);
  DEBUG_PRINT(F(", right: "));
  DEBUG_PRINT(_right);
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

    fuzzy.add(MAX_SPEED_VALUE, isCw);
    fuzzy.add(-MAX_SPEED_VALUE, isCcw);
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
  DEBUG_PRINTLN(right);
  _left = left;
  _right = right;
  if (_left != 0) {
    long leftPps = (long)_sensors.leftPps();
    DEBUG_PRINT(F("//     leftPps="));
    DEBUG_PRINT(leftPps);
    DEBUG_PRINT(F(", _leftPower="));
    DEBUG_PRINT(_leftPower);

    _leftPower = ((long)_leftPower * _powerK + _signalK * _left + _ppsK * leftPps) / MAX_VALUE;
    DEBUG_PRINT(F(", _leftPower'="));
    DEBUG_PRINT(_leftPower);

    _leftPower = min(max(_leftPower, -MAX_VALUE), MAX_VALUE);
    DEBUG_PRINT(F(", clipped _leftPower ="));
    DEBUG_PRINT(_leftPower);
    DEBUG_PRINTLN();
  } else {
    _leftPower = 0;
  }

  if (_right != 0) {
    long rightPps = (long)_sensors.rightPps();
    DEBUG_PRINT(F("//     rightPps="));
    DEBUG_PRINT(rightPps);
    DEBUG_PRINT(F(", _rightPower="));
    DEBUG_PRINT(_rightPower);

    _rightPower = ((long)_rightPower * _powerK + _signalK * _right + _ppsK * rightPps) / MAX_VALUE;
    DEBUG_PRINT(F(", _rightPower'="));
    DEBUG_PRINT(_rightPower);

    _rightPower = min(max(_rightPower, -MAX_VALUE), MAX_VALUE);
    DEBUG_PRINT(F(", clipped _rightPower ="));
    DEBUG_PRINT(_rightPower);
    DEBUG_PRINTLN();
  } else {
    _rightPower = 0;
  }

  DEBUG_PRINT(F("// MotionCtrl::power effective "));
  DEBUG_PRINT(_leftPower);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINTLN(_rightPower);

  _leftMotor.speed(_leftPower);
  _rightMotor.speed(_rightPower);
  _sensors.direction(_leftPower, _rightPower);
}

/*
   Motor controller section
*/

const int defaultCorrection[] PROGMEM = { -MAX_VALUE,  -MAX_VALUE / 2, 0, MAX_VALUE / 2, MAX_VALUE};

MotorCtrl::MotorCtrl(byte forwPin, byte backPin) {
  _forwPin = forwPin;
  _backPin = backPin;
  memcpy_P(_x, defaultCorrection, sizeof(int[NO_POINTS]));
  memcpy_P(_y, defaultCorrection, sizeof(int[NO_POINTS]));
}

void MotorCtrl::begin() {
  pinMode(_forwPin, OUTPUT);
  pinMode(_backPin, OUTPUT);
}

void MotorCtrl::setCorrection_P(int *p) {
  _x[1] = pgm_read_word(p);
  _y[1] = pgm_read_word(p + 1);
  _x[3] = pgm_read_word(p + 2);
  _y[3] = pgm_read_word(p + 3);
}

void MotorCtrl::setCorrection(int *p) {
  _x[1] = p[0];
  _y[1] = p[1];
  _x[3] = p[2];
  _y[3] = p[3];
}

/*
   Set speed
*/
void MotorCtrl::speed(int value) {
  int pwd = func((int)(value));
  if (value == 0) {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, 0);
  } else if (value > 0) {
    analogWrite(_forwPin, pwd);
    analogWrite(_backPin, 0);
  } else {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, -pwd);
  }
}

int MotorCtrl::func(int x) {
  DEBUG_PRINT(F("// MotorCtrl::func "));
  DEBUG_PRINT(x);
  DEBUG_PRINTLN();

  int i = NO_POINTS - 2;
  for (int j = 1; j <= NO_POINTS - 2; j++) {
    if (x < _x[j]) {
      i = j - 1;
      break;
    }
  }
  int result = (int)((long)(x - _x[i]) * (_y[i + 1] - _y[i]) / (_x[i + 1] - _x[i]) + _y[i]);

  DEBUG_PRINT(F("//     i: "));
  DEBUG_PRINT(i);
  DEBUG_PRINT(F(", x0: "));
  DEBUG_PRINT(_x[i]);
  DEBUG_PRINT(F(", x1: "));
  DEBUG_PRINT(_x[i + 1]);
  DEBUG_PRINT(F(", y0: "));
  DEBUG_PRINT(_y[i]);
  DEBUG_PRINT(F(", y1: "));
  DEBUG_PRINT(_y[i + 1]);
  DEBUG_PRINTLN();
  DEBUG_PRINT(F("//     f(x): "));
  DEBUG_PRINT(result);
  DEBUG_PRINTLN();

  return min(max(-MAX_VALUE, result), MAX_VALUE);
}
