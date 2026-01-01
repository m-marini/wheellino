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

#include <esp_log.h>
static char* TAG = "MotorTest";

#define SERIAL_BPS 115200
#define MOTOR_TEST_DURATION 5000
#define SPEED_THRESHOLD 20
#define MAX_POWER 255
#define TEST_SPEED 30

static MotorSensor leftSensor(LEFT_PIN);
static MotorSensor rightSensor(RIGHT_PIN);
static MotorCtrl leftMotor(LEFT_FORW_PIN, LEFT_BACK_PIN, leftSensor);
static MotorCtrl rightMotor(RIGHT_FORW_PIN, RIGHT_BACK_PIN, rightSensor);

/*
  Left motor controller configuration
*/
const static tcsParams_t leftCfg = {
  .fi0 = 904,
  .fix = 5127,
  .fd0 = 500,
  .fdx = 5261,
  .bi0 = -690,
  .bix = 1680,
  .bd0 = -450,
  .bdx = 1702,
  .muForw = 30000,
  .muBack = 30000,
  .alpha = 100,
  .ax = 1000
};

/*
  Right motor controller configuration
*/
const static tcsParams_t rightCfg = {
  .fi0 = 628,
  .fix = 2165,
  .fd0 = 400,
  .fdx = 2322,
  .bi0 = -620,
  .bix = 1569,
  .bd0 = -400,
  .bdx = 1468,
  .muForw = 30000,
  .muBack = 30000,
  .alpha = 50,
  .ax = 1000
};


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
  //new MotorPowerTest("1. Test left motor forward power", leftMotor, MAX_POWER / 2),
  //new MotorPowerTest("3. Test left motor backward power", leftMotor, -MAX_POWER / 2),
  //new MotorPowerTest("2. Test right motor forward power", rightMotor, MAX_POWER / 2),
  //new MotorPowerTest("4. Test right motor backward power", rightMotor, -MAX_POWER / 2),
  new SpeedTest("5. Test left speed forward", TEST_SPEED, 0),
  new SpeedTest("6. Test left speed backward", -TEST_SPEED, 0),
  new SpeedTest("7. Test right speed forward", 0, TEST_SPEED),
  new SpeedTest("8. Test right speed backward", 0, -TEST_SPEED),
  new SummaryTest(),
  new SensorsTest()
};

void setup() {
  Serial.begin(SERIAL_BPS);
  while (!Serial) {
    delay(10);
  }

  /* Setup supplier sensor */
  pinMode(VOLTAGE_PIN, INPUT);

  leftMotor.begin();
  rightMotor.begin();
  leftMotor.tcs(leftCfg);
  rightMotor.tcs(rightCfg);
  ESP_LOGI(TAG, "Start.");

  int n = 0;
  int err = 0;
  tests[0]->begin();
}

unsigned long timeout;

void loop() {
  static int currentTest = 0;
  // Read motor supply voltage
  int supply = analogRead(VOLTAGE_PIN);
  leftMotor.supply(supply);
  rightMotor.supply(supply);
  Test& test = *tests[currentTest];
  if (test.completed()) {
    ESP_LOGD(TAG, "Test completed");
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
  ESP_LOGI(TAG, "%s", _name);

  _motor.automatic(false);
  _motor.pwm(_power);
  _timeout = millis() + MOTOR_TEST_DURATION;
}

void MotorPowerTest::polling(const unsigned long t0) {
  ESP_LOGD(TAG, "t0: %lu", t0);
  MotorSensor& sensor = _motor.sensor();
  if (t0 < _timeout) {
    _motor.polling(t0);
    const long pulses = sensor.pulses();
    if (pulses != _oldPulses) {
      _oldPulses = pulses;
      ESP_LOGD(TAG, "pulses: %ld, pps: %f", pulses, (double)sensor.pps());
    }
  } else if (!_completed) {
    ESP_LOGD(TAG, "completed");

    _completed = true;
    const int pps = sensor.pps();
    _motor.pwm(0);
    _valid = true;
    if ((pps < 0 && _power > 0)
        || (pps > 0 && _power < 0)) {
      ESP_LOGE(TAG, "KO speed measured %d", pps);
      _valid = false;
    }
    if (abs(pps) < SPEED_THRESHOLD) {
      ESP_LOGE(TAG, "KO speed measured %d", pps);
      _valid = false;
    }
    if (_valid) {
      ESP_LOGI(TAG, "OK speed measured %d", pps);
    }
  } else {
    ESP_LOGE(TAG, "unexpected status");
    _motor.pwm(0);
  }
}

void SpeedTest::begin(void) {
  ESP_LOGI(TAG, "%s", _name);

  leftMotor.automatic(true);
  rightMotor.automatic(true);
  leftMotor.pwm(0);
  rightMotor.pwm(0);
  leftMotor.speed(_leftSpeed);
  rightMotor.speed(_rightSpeed);
  _timeout = millis() + MOTOR_TEST_DURATION;
}

void SpeedTest::polling(const unsigned long t0) {
  if (t0 < _timeout) {
    leftMotor.polling(t0);
    rightMotor.polling(t0);
    ESP_LOGD(TAG, "Speeds: %f, %f", (double)leftSensor.pps(), (double)rightSensor.pps());
  } else if (!_completed) {
    _completed = true;
    const float leftSpeedMeasure = leftSensor.pps();
    const float rightSpeedMeasure = rightSensor.pps();
    leftMotor.speed(0);
    rightMotor.speed(0);
    leftMotor.pwm(0);
    rightMotor.pwm(0);

    _valid = true;
    char msg[256];
    if (abs(_leftSpeed - leftSpeedMeasure) >= SPEED_THRESHOLD) {
      ESP_LOGI(TAG, "KO left speed measured %.3f != expected %d", (double)leftSpeedMeasure, _leftSpeed);
      _valid = false;
    }
    if (abs(_rightSpeed - rightSpeedMeasure) >= SPEED_THRESHOLD) {
      ESP_LOGI(TAG, "KO right speed measured %.3f != expected %d", (double)rightSpeedMeasure, _rightSpeed);
      _valid = false;
    }
    if (_valid) {
      ESP_LOGI(TAG, "OK speeds measured %.3f,%.3f = %d,%d",
               (double)leftSpeedMeasure, rightSpeedMeasure, _leftSpeed, _rightSpeed);
    }
  } else {
    ESP_LOGE(TAG, "unexpected status");
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
  ESP_LOGI(TAG, "-------------------------------------------------------");
  ESP_LOGI(TAG, "Failed %d, success %d, total %d", failed, valid, n - 2);
  ESP_LOGI(TAG, "-------------------------------------------------------");
}

void SummaryTest::polling(const unsigned long) {
  _completed = true;
  _valid = true;
}

void SensorsTest::begin(void) {
  ESP_LOGI(TAG, "Testing sensors ...");
  leftSensor.direction(1);
  rightSensor.direction(1);
}

void SensorsTest::polling(const unsigned long t0) {
  leftSensor.polling(t0);
  rightSensor.polling(t0);
  const long left = leftSensor.pulses();
  if (left != _leftPulses) {
    _leftPulses = left;
    ESP_LOGI(TAG, "Left pulses: %ld, pps: %f", left, (double)leftSensor.pps());
  }
  const long right = rightSensor.pulses();
  if (right != _rightPulses) {
    _rightPulses = right;
    ESP_LOGI(TAG, "Right pulses: %ld, pps: %f", right, (double)rightSensor.pps());
  }
}
