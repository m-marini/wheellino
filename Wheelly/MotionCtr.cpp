#include "MotionCtrl.h"

//#define DEBUG
#include "debug.h"
#include "Utils.h"
#include "Fuzzy.h"

#define MAX_VALUE 255

#define MAX_SPEED_VALUE 4

#define MOTOR_SAFE_INTERVAL 1000ul
#define MOTOR_CHECK_INTERVAL 300ul

#define MIN_SPEED_DIRECTION_RAD  (3 * PI / 180)
#define MAX_SPEED_DIRECTION_RAD  (10 * PI / 180)
#define LINEAR_DIRECTION_RAD  (30 * PI / 180)

#define FEEDBACK_GAIN (2 * MAX_VALUE)

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
void MotionCtrl::setCorrection(int *p) {
  _leftMotor.setCorrection(p);
  _rightMotor.setCorrection(p+4);
}

/*

*/
void MotionCtrl::move(float direction, int speed) {
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
    fuzzy.add((float)_speed / MAX_SPEED_VALUE, isLin);
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
    power(round(left * MAX_VALUE), round(right * MAX_VALUE));
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

  long leftPwr = 0;
  if (_left != 0) {
    long leftPps = (long)_sensors.leftPps() * MAX_VALUE / MAX_PPS;
    long dLeftPps = left - leftPps;
    DEBUG_PRINT(F("// MotionCtrl::power leftPps="));
    DEBUG_PRINT(leftPps);
    DEBUG_PRINT(F(" dLeftPps="));
    DEBUG_PRINT(dLeftPps);
    DEBUG_PRINTLN();
    leftPwr = min(max(left + dLeftPps * FEEDBACK_GAIN / MAX_VALUE,
                      -MAX_VALUE), MAX_VALUE);
  }

  long rightPwr = 0;
  if (_right != 0) {
    long rightPps = (long)_sensors.rightPps() * MAX_VALUE / MAX_PPS;
    long dRightPps = right - rightPps;
    DEBUG_PRINT(F("// MotionCtrl::power rightPps="));
    DEBUG_PRINT(rightPps);
    DEBUG_PRINT(F(" dRightPps="));
    DEBUG_PRINT(dRightPps);
    DEBUG_PRINTLN();
    rightPwr = min(max(right + dRightPps * FEEDBACK_GAIN / MAX_VALUE,
                       -MAX_VALUE), MAX_VALUE);
  }

  DEBUG_PRINT(F("// MotionCtrl::power effective "));
  DEBUG_PRINT(leftPwr);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINTLN(rightPwr);

  _leftMotor.speed(leftPwr);
  _rightMotor.speed(rightPwr);
  _sensors.setDirection(leftPwr, rightPwr);
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
  _y[1] = pgm_read_word(p+1);
  _x[3] = pgm_read_word(p+2);
  _y[3] = pgm_read_word(p+3);
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
  int i = NO_POINTS - 2;
  for (int j = 1; j < NO_POINTS - 2; j++) {
    if (x < _x[j]) {
      i = j - 1;
      break;
    }
  }
  int result = (int)((long)(x - _x[i]) * (_y[i + 1] - _y[i]) / (_x[i + 1] - _x[i]) + _y[i]);
  DEBUG_PRINT(F("// x: "));
  DEBUG_PRINT(x);
  DEBUG_PRINT(F(", x0: "));
  DEBUG_PRINT(_x[i]);
  DEBUG_PRINT(F(", x1: "));
  DEBUG_PRINT(_x[i + 1]);
  DEBUG_PRINT(F(", y0: "));
  DEBUG_PRINT(_y[i]);
  DEBUG_PRINT(F(", y1: "));
  DEBUG_PRINT(_y[i + 1]);
  DEBUG_PRINT(F(", f(x): "));
  DEBUG_PRINT(result);
  DEBUG_PRINTLN();

  return min(max(-MAX_VALUE, result), MAX_VALUE);
}
