#include "MotionCtrl.h"

//#define DEBUG
#include "debug.h"
#include "Utils.h"
#include "Fuzzy.h"

#define MAX_VALUE 255

#define MOTOR_SAFE_INTERVAL 1000ul
#define MOTOR_CHECK_INTERVAL 300ul

#define MAX_LIN_SPEED 0.280f

#define MIN_SPEED_DIRECTION_RAD  (3 * PI / 180)
#define MAX_SPEED_DIRECTION_RAD  (10 * PI / 180)
#define LINEAR_DIRECTION_RAD  (30 * PI / 180)

#define FEEDBACK_GAIN 2

const float leftXCorrection[] PROGMEM = { -1,  -0.06055, 0, 0.02311, 1};
const float leftYCorrection[] PROGMEM = { -1, -0.30432, 0, 0.12577, 1};
const float rightXCorrection[] PROGMEM = { -1, -0.03759, 0, 0.02041, 1};
const float rightYCorrection[] PROGMEM = { -1, -0.2667, 0, 0.12648, 1};

/*
  Creates the motion controller
*/
MotionCtrl::MotionCtrl(byte leftForwPin, byte leftBackPin, byte rightForwPin, byte rightBackPin, byte leftSensorPin, byte rightSensorPin)
  : _leftMotor(leftForwPin, leftBackPin),
    _rightMotor(rightForwPin, rightBackPin),
    _sensors(leftSensorPin, rightSensorPin) {

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
    DEBUG_PRINTLN("// Motor check timer triggered");
    ((MotionCtrl*)ctx)->handleMotion(millis());
  }, this);
}

/*
  Initializes the motion controller
*/
void MotionCtrl::begin() {
  _leftMotor.begin();
  _leftMotor.setCorrection(leftXCorrection, leftYCorrection);
  _rightMotor.begin();
  _rightMotor.setCorrection(rightXCorrection, rightYCorrection);
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
void MotionCtrl::move(float direction, float speed) {
  DEBUG_PRINT(F("// MotionCtrl::move "));
  DEBUG_PRINTF(direction * 180 / PI, 0);
  DEBUG_PRINT(F(" "));
  DEBUG_PRINT(speed);
  DEBUG_PRINTLN();
  _direction = direction;
  _speed = speed;
  if (_halt) {
    _halt = false;
    _stopTimer.start();
    _checkTimer.start();
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
    float dir = angle();
    float toDir = _direction;
    float turn = normalRad(toDir - dir);

    DEBUG_PRINT(F("//   dir: "));
    DEBUG_PRINTF(dir * 180 / 2, 0);
    DEBUG_PRINT(F(", to: "));
    DEBUG_PRINTF(toDir * 180 / 2, 0);
    DEBUG_PRINT(F(", turn: "));
    DEBUG_PRINTF(turn * 180 / 2, 0);
    DEBUG_PRINTLN();

    float isCw = fuzzyPositive(turn - MIN_SPEED_DIRECTION_RAD,
                               MAX_SPEED_DIRECTION_RAD - MIN_SPEED_DIRECTION_RAD);
    float isCcw = fuzzyPositive(-turn - MIN_SPEED_DIRECTION_RAD,
                                MAX_SPEED_DIRECTION_RAD - MIN_SPEED_DIRECTION_RAD);
    float isLin = 1 - fuzzyPositive(abs(turn), LINEAR_DIRECTION_RAD);
    Fuzzy fuzzy;

    fuzzy.add(1, isCw);
    fuzzy.add(-1, isCcw);
    fuzzy.add(0, 1 - max(isCw, isCcw));
    float cwSpeed = fuzzy.defuzzy();

    fuzzy.reset();
    fuzzy.add(_speed, isLin);
    fuzzy.add(0, 1 - isLin);
    float linSpeed = fuzzy.defuzzy();

    DEBUG_PRINT(F("//    isCw: "));
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

    float left = linSpeed + cwSpeed;
    float right = linSpeed - cwSpeed;

    float mx = max(max(abs(left), abs(right)), 1);

    left /= mx;
    right /= mx;

    DEBUG_PRINT(F("// motors: "));
    DEBUG_PRINT(left);
    DEBUG_PRINT(F(", "));
    DEBUG_PRINT(right);
    DEBUG_PRINTLN();
    power(left, right);
  }
}

/*

*/
void MotionCtrl::power(float left, float right) {
  DEBUG_PRINT(F("// MotionCtrl::power "));
  DEBUG_PRINT(left);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINTLN(right);
  _left = left;
  _right = right;

  float leftPwr = 0;
  if (_left != 0) {
    float leftPps = _sensors.leftPps() / MAX_PPS;
    float dLeftPps = left - leftPps;
    leftPwr = min(max(left + dLeftPps * FEEDBACK_GAIN, -1), 1);
  }

  float rightPwr = 0;
  if (_right != 0) {
    float rightPps = _sensors.rightPps() / MAX_PPS;
    float dRightPps = right - rightPps;
    rightPwr = min(max(right + dRightPps * FEEDBACK_GAIN, -1), 1);
  }

  _leftMotor.speed(leftPwr);
  _rightMotor.speed(rightPwr);
  _sensors.setDirection(leftPwr, rightPwr);
}

/*
   Motor controller section
*/

const float defaultCorrection[] PROGMEM = { -1,  -0.5, 0, 0.5, 1};

MotorCtrl::MotorCtrl(byte forwPin, byte backPin) {
  _forwPin = forwPin;
  _backPin = backPin;
  _x = defaultCorrection;
  _y = defaultCorrection;
}

void MotorCtrl::begin() {
  pinMode(_forwPin, OUTPUT);
  pinMode(_backPin, OUTPUT);
}

void MotorCtrl::setCorrection(float *x, float *y) {
  _x = x;
  _y = y;
}

/*
   Set speed
*/
void MotorCtrl::speed(float value) {
  float pwd = func(value);
  if (value == 0) {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, 0);
  } else if (value > 0) {
    int signal = round(pwd * MAX_VALUE);
    analogWrite(_forwPin, signal);
    analogWrite(_backPin, 0);
  } else {
    int signal = -round(pwd * MAX_VALUE);
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, signal);
  }
}

float MotorCtrl::func(float x) {
  int i = NO_POINTS - 2;
  float xx[NO_POINTS];
  float yy[NO_POINTS];
  memcpy_P(xx, _x, sizeof(float[NO_POINTS]));
  memcpy_P(yy, _y, sizeof(float[NO_POINTS]));
  for (int j = 1; j < NO_POINTS - 2; j++) {
    if (x < xx[j]) {
      i = j - 1;
      break;
    }
  }
  float result = (x - xx[i]) * (yy[i + 1] - yy[i]) / (xx[i + 1] - xx[i]) + yy[i];
  DEBUG_PRINT(F("// x: "));
  DEBUG_PRINT(x);
  DEBUG_PRINT(F(", f(x): "));
  DEBUG_PRINT(result);
  DEBUG_PRINTLN();

  return min(max(-1, result), 1);
}
