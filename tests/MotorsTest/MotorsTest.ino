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

/*
   Test of motors.

  The test runs the motor forward and backward for 5 sec.
  It measures the movement from motion sensors.
*/
#include "pins.h"
#include "MotorCtrl.h"

//#define DEBUG
#include "debug.h"

#define SERIAL_BPS 115200
#define MOTOR_TEST_DURATION 3000
#define SPEED_THRESHOLD 20
#define MAX_POWER 255
#define TEST_SPEED 60

static MotorSensor leftSensor(LEFT_PIN);
static MotorSensor rightSensor(RIGHT_PIN);
static MotorCtrl leftMotor(LEFT_FORW_PIN, LEFT_BACK_PIN, leftSensor);
static MotorCtrl rightMotor(RIGHT_FORW_PIN, RIGHT_BACK_PIN, rightSensor);

class Test {
public:
  Test(void)
    : _completed(false), _valid(false) {}
  virtual void begin(void) = 0;
  virtual void polling(const unsigned long t0) = 0;
  const boolean completed(void) const {
    return _completed;
  }
  const boolean valid(void) const {
    return _valid;
  }
protected:
  boolean _completed;
  boolean _valid;
};

class MotorPowerTest : public Test {
public:
  MotorPowerTest(const char* name, MotorCtrl& motor, const int power)
    : _name(name), _motor(motor), _power(power) {}
  void begin(void);
  void polling(const unsigned long t0);
private:
  const char* _name;
  const int _power;
  MotorCtrl& _motor;
  unsigned long _timeout;
  long _oldPulses;
};

class SpeedTest : public Test {
public:
  SpeedTest(const char* name,
            const int leftSpeed, const int rightSpeed)
    : _name(name), _leftSpeed(leftSpeed), _rightSpeed(rightSpeed) {}
  void begin(void);
  void polling(const unsigned long t0);
private:
  const char* _name;
  const int _leftSpeed;
  const int _rightSpeed;
  unsigned long _timeout;
};

class SensorsTest : public Test {
public:
  void begin(void);
  void polling(const unsigned long t0);
private:
  long _leftPulses;
  long _rightPulses;
};

class SummaryTest : public Test {
public:
  void begin(void);
  void polling(const unsigned long t0);
};

static Test* tests[]{
  new MotorPowerTest("1. Test left motor forward power", leftMotor, MAX_POWER / 2),
  new MotorPowerTest("3. Test left motor backward power", leftMotor, -MAX_POWER / 2),
  new MotorPowerTest("2. Test right motor forward power", rightMotor, MAX_POWER / 2),
  new MotorPowerTest("4. Test right motor backward power", rightMotor, -MAX_POWER / 2),
  new SpeedTest("5. Test left speed forward", TEST_SPEED, 0),
  new SpeedTest("6. Test left speed backward", -TEST_SPEED, 0),
  new SpeedTest("7. Test right speed forward", 0, TEST_SPEED),
  new SpeedTest("8. Test right speed backward", 0, -TEST_SPEED),
  new SummaryTest(),
  new SensorsTest()
};

void setup() {
  Serial.begin(SERIAL_BPS);
  delay(500);
  Serial.println("");
  leftMotor.begin();
  rightMotor.begin();
  Serial.println("Start.");

  int n = 0;
  int err = 0;
  tests[0]->begin();
}

unsigned long timeout;

void loop() {
  static int currentTest = 0;
  Test& test = *tests[currentTest];
  if (test.completed()) {
    DEBUG_PRINTLN("// loop: Test completed");
    delay(1000);
  } else {
    test.polling(millis());
    if (test.completed()) {
      if (currentTest < (sizeof(tests) / sizeof(tests[0]) - 1)) {
        currentTest++;
        tests[currentTest]->begin();
      }
    }
  }
}

void MotorPowerTest::begin(void) {
  Serial.println();
  Serial.print(_name);
  Serial.println();

  _motor.automatic(false);
  _motor.power(_power);
  _timeout = millis() + MOTOR_TEST_DURATION;
}

void MotorPowerTest::polling(const unsigned long t0) {
  DEBUG_PRINT("// MotorPowerTest::polling t0: ");
  DEBUG_PRINT(t0);
  DEBUG_PRINTLN();
  MotorSensor& sensor = _motor.sensor();
  if (t0 < _timeout) {
    _motor.polling(t0);
    const long pulses = sensor.pulses();
    if (pulses != _oldPulses) {
      _oldPulses = pulses;
      DEBUG_PRINT("// MotorPowerTest::polling pulses: ");
      DEBUG_PRINT(pulses);
      DEBUG_PRINT(", pps: ");
      DEBUG_PRINT(sensor.pps());
      DEBUG_PRINTLN();
    }
  } else if (!_completed) {
    DEBUG_PRINT("//   completed");
    DEBUG_PRINTLN();

    _completed = true;
    const int pps = sensor.pps();
    _motor.power(0);
    _valid = true;
    if ((pps < 0 && _power > 0)
        || (pps > 0 && _power < 0)) {
      Serial.print("KO speed measured ");
      Serial.print(pps);
      Serial.println();
      _valid = false;
    }
    if (abs(pps) < SPEED_THRESHOLD) {
      Serial.print("KO speed measured ");
      Serial.print(pps);
      Serial.println();
      _valid = false;
    }
    if (_valid) {
      Serial.print("OK speed measured ");
      Serial.print(pps);
      Serial.println();
    }
  } else {
    DEBUG_PRINT("// MotorPowerTest::polling unexpected status");
    DEBUG_PRINTLN();
    _motor.power(0);
  }
}

void SpeedTest::begin(void) {
  Serial.println();
  Serial.print(_name);
  Serial.println();

  leftMotor.automatic(true);
  rightMotor.automatic(true);
  leftMotor.power(0);
  rightMotor.power(0);
  leftMotor.speed(_leftSpeed);
  rightMotor.speed(_rightSpeed);
  _timeout = millis() + MOTOR_TEST_DURATION;
}

void SpeedTest::polling(const unsigned long t0) {
  if (t0 < _timeout) {
    leftMotor.polling(t0);
    rightMotor.polling(t0);
    DEBUG_PRINT("Speeds: ");
    DEBUG_PRINT(leftSensor.pps());
    DEBUG_PRINT(", ");
    DEBUG_PRINT(rightSensor.pps());
    DEBUG_PRINTLN();
  } else if (!_completed) {
    _completed = true;
    const float leftSpeedMeasure = leftSensor.pps();
    const float rightSpeedMeasure = rightSensor.pps();
    leftMotor.speed(0);
    rightMotor.speed(0);
    leftMotor.power(0);
    rightMotor.power(0);

    _valid = true;
    char msg[256];
    if (abs(_leftSpeed - leftSpeedMeasure) >= SPEED_THRESHOLD) {
      Serial.print("KO left speed measured ");
      Serial.print(leftSpeedMeasure);
      Serial.print(" != expected ");
      Serial.print(_leftSpeed);
      Serial.println();
      _valid = false;
    }
    if (abs(_rightSpeed - rightSpeedMeasure) >= SPEED_THRESHOLD) {
      Serial.print("KO right speed measured ");
      Serial.print(rightSpeedMeasure);
      Serial.print(" != expected ");
      Serial.print(_rightSpeed);
      Serial.println();
      _valid = false;
    }
    if (_valid) {
      Serial.print("OK speeds measured ");
      Serial.print(leftSpeedMeasure);
      Serial.print(", ");
      Serial.print(rightSpeedMeasure);
      Serial.println();
    }
  } else {
    DEBUG_PRINT("// SpeedTest::polling unexpected status");
    DEBUG_PRINTLN();
  }
}

void SummaryTest::begin(void) {
  const size_t n = sizeof(tests) / sizeof(tests[0]);
  int failed = 0;
  int valid = 0;
  for (int i = 0; i < n - 2; i++) {
    if (tests[i]->valid()) {
      valid++;
    } else {
      failed++;
    }
  }
  Serial.println();
  Serial.print("Failed ");
  Serial.print(failed);
  Serial.print(", success ");
  Serial.print(valid);
  Serial.print(", total ");
  Serial.print(n - 2);
  Serial.println();
}

void SummaryTest::polling(const unsigned long) {
  _completed = true;
  _valid = true;
}

void SensorsTest::begin(void) {
  Serial.println();
  Serial.print("Testing sensors ...");
  Serial.println();
  leftSensor.direction(1);
  rightSensor.direction(1);
}

void SensorsTest::polling(const unsigned long t0) {
  leftSensor.polling(t0);
  rightSensor.polling(t0);
  const long left = leftSensor.pulses();
  if (left != _leftPulses) {
    _leftPulses = left;
    Serial.print("Left pulses: ");
    Serial.print(left);
    Serial.print(", pps: ");
    Serial.print(leftSensor.pps());
    Serial.println();
  }
  const long right = rightSensor.pulses();
  if (right != _rightPulses) {
    _rightPulses = right;
    Serial.print("Right pulses: ");
    Serial.print(right);
    Serial.print(", pps: ");
    Serial.print(rightSensor.pps());
    Serial.println();
  }
}
