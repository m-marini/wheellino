/*
   Test of asynchronous servo.

   The test run for 40 sec. moving the proxy sensor from left to right
   and backward.
*/
#include "Arduino.h"

#include "pins.h"
#include "AsyncServo.h"

#define SERIAL_BPS  115200
#define PERIOD    4000ul
#define HALF_PERIOD (PERIOD / 2)
#define TEST_DURATION  (PERIOD * 10)
#define FRONT_ANGLE 90

static int lastAngle;
static unsigned long timeout;
static boolean completed;
static AsyncServoClass ProxyServo;

void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.println("");
  ProxyServo.onReached(handleReached);
  ProxyServo.attach(SERVO_PIN);
  ProxyServo.angle(FRONT_ANGLE);
  delay(500);
  Serial.println("Start.");
  timeout = millis() + TEST_DURATION;
}

void loop() {
  ProxyServo.polling();
}

static void handleReached(void*, const int angle) {
  if (angle != lastAngle) {
    lastAngle = angle;
    Serial.print("Angle ");
    Serial.print(angle);
    Serial.print(" DEG");
    Serial.println();
  }
  const unsigned long t0 = millis() % PERIOD;
  // : 180 - 180 * (t0 - HALF_PERIOD) / HALF_PERIOD;
  const int angle1 = t0 <= HALF_PERIOD
                     ? 180 * t0 / HALF_PERIOD
                     : 360 - 180 * t0 / HALF_PERIOD;
  if (millis() < timeout) {
    ProxyServo.angle(angle1);
  } else if (!completed) {
    ProxyServo.angle(FRONT_ANGLE);
    completed = true;
    Serial.println("Completed.");
  }
}
