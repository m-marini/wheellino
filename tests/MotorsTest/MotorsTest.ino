/*
   Test of motors.

  The test runs the motor forward and backward for 5 sec.
  It measures the movement from motion sensors.
*/
#include "pins.h"
#include "MotorCtrl.h"

//#define DEBUG
#include "debug.h"

#define SERIAL_BPS          115200
#define MOTOR_TEST_DURATION 3000
#define SPEED_THRESHOLD     20
#define MAX_POWER           255
#define TEST_SPEED          90

static MotorSensor leftSensor(LEFT_PIN);
static MotorSensor rightSensor(RIGHT_PIN);
static MotorCtrl leftMotor(LEFT_FORW_PIN, LEFT_BACK_PIN, leftSensor);
static MotorCtrl rightMotor(RIGHT_FORW_PIN, RIGHT_BACK_PIN, rightSensor);

void setup() {
  Serial.begin(SERIAL_BPS);
  delay(500);
  Serial.println("");
  leftMotor.begin();
  rightMotor.begin();
  Serial.println("Start.");

  int n = 0;
  int err = 0;
  /*
    leftSensor.direction(-1);
    testSensor(leftSensor, 10000);
  */

  if (!testMotorPower(++n, "Test left motor power", leftMotor, MAX_POWER / 2, MOTOR_TEST_DURATION)) {
    err++;
  }

  if (!testMotorPower(++n, "Test right motor power", rightMotor, MAX_POWER / 2, MOTOR_TEST_DURATION)) {
    err++;
  }

  if (!testSpeed(++n, "Test left forward", TEST_SPEED, 0)) {
    err++;
  }

  if (!testSpeed(++n, "Test left backward", -TEST_SPEED, 0)) {
    err++;
  }

  if (!testSpeed(++n, "Test right forward", 0, TEST_SPEED)) {
    err++;
  }

  if (!testSpeed(++n, "Test right backward", 0, -TEST_SPEED)) {
    err++;
  }

  Serial.println();
  Serial.print("Failed ");
  Serial.print(err);
  Serial.print(", success ");
  Serial.print(n - err);
  Serial.print(", total ");
  Serial.print(n);
  Serial.println();
}

unsigned long timeout;

void loop() {
  delay(1000);
  /*
    const unsigned long t0 = millis();
    leftMotor.polling(t0);
    rightMotor.polling(t0);
    if (t0 >= timeout) {
    timeout = t0 + 1000;
    Serial.print("Pulses: ");
    Serial.print(leftSensor.pulses());
    Serial.print(", speed: ");
    Serial.print(leftSensor.pps());
    Serial.println();
    }
  */
}

static boolean testSpeed(const int n, const char *name,
                         const int leftSpeed, const int rightSpeed) {
  Serial.println();
  Serial.print(n);
  Serial.print(". ");
  Serial.print(name);
  Serial.println();

  leftMotor.automatic(true);
  rightMotor.automatic(true);
  leftMotor.power(0);
  rightMotor.power(0);
  leftMotor.polling();
  rightMotor.polling();
  leftMotor.speed(leftSpeed);
  rightMotor.speed(rightSpeed);
  pollingFor(MOTOR_TEST_DURATION);
  const float leftSpeedMeasure = leftSensor.pps();
  const float rightSpeedMeasure = rightSensor.pps();
  leftMotor.speed(0);
  rightMotor.speed(0);
  leftMotor.power(0);
  rightMotor.power(0);

  boolean valid = true;
  char msg[256];
  if (abs(leftSpeed - leftSpeedMeasure) >= SPEED_THRESHOLD) {
    Serial.print("KO left speed measured ");
    Serial.print(leftSpeedMeasure);
    Serial.print(" != expected ");
    Serial.print(leftSpeed);
    Serial.println();
    valid = false;
  }
  if (abs(rightSpeed - rightSpeedMeasure) >= SPEED_THRESHOLD) {
    Serial.print("KO right speed measured ");
    Serial.print(rightSpeedMeasure);
    Serial.print(" != expected ");
    Serial.print(rightSpeed);
    Serial.println();
    valid = false;
  }
  if (valid) {
    Serial.print("OK speeds measured ");
    Serial.print(leftSpeedMeasure);
    Serial.print(", ");
    Serial.print(rightSpeedMeasure);
    Serial.println();
  }
  return valid;
}

static void pollingFor(const unsigned long duration) {
  unsigned long timeout = millis() + duration;
  unsigned t0;
  do {
    t0 = millis();
    leftMotor.polling(t0);
    rightMotor.polling(t0);
    DEBUG_PRINT("Speeds: ");
    DEBUG_PRINT(leftSensor.pps());
    DEBUG_PRINT(", ");
    DEBUG_PRINT(rightSensor.pps());
    DEBUG_PRINTLN();
  } while (t0 < timeout);
}

static void testSensor(MotorSensor& sensor, const unsigned long duration) {
  Serial.println("Start.");
  const unsigned long timeout = millis() + duration;
  unsigned t0;
  long oldPulses = 0;
  do {
    t0 = millis();
    sensor.polling(t0);
    const long pulses = sensor.pulses();
    if (pulses != oldPulses) {
      oldPulses = pulses;
      Serial.print("pulses: ");
      Serial.print(pulses);
      Serial.print(", pps: ");
      Serial.print(sensor.pps());
      Serial.println();
    }
  } while (t0 < timeout);
}

static boolean testMotorPower(const int n, const char* name, MotorCtrl& motor, const int power, const unsigned long duration) {
  Serial.println();
  Serial.print(n);
  Serial.print(". ");
  Serial.print(name);
  Serial.println();

  motor.automatic(false);
  motor.power(power);
  const unsigned long timeout = millis() + duration;
  unsigned t0;
  long oldPulses = 0;
  MotorSensor& sensor = motor.sensor();
  do {
    t0 = millis();
    motor.polling(t0);
    const long pulses = sensor.pulses();
    if (pulses != oldPulses) {
      oldPulses = pulses;
      DEBUG_PRINT("pulses: ");
      DEBUG_PRINT(pulses);
      DEBUG_PRINT(", pps: ");
      DEBUG_PRINT(sensor.pps());
      DEBUG_PRINTLN();
    }
  } while (t0 < timeout);
  const int pps = sensor.pps();
  motor.power(0);
  boolean valid = true;
  if ((pps < 0 && power > 0)
      || (pps > 0 && power < 0)) {
    Serial.print("KO speed measured ");
    Serial.print(pps);
    Serial.println();
    valid = false;
  }
  if (abs(pps) < SPEED_THRESHOLD) {
    Serial.print("KO speed measured ");
    Serial.print(pps);
    Serial.println();
    valid = false;
  }
  if (valid) {
    Serial.print("OK speed measured ");
    Serial.print(pps);
    Serial.println();
  }
  return valid;
}
