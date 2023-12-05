/*
   Test of asynchronous servo.

   The test run for 40 sec. moving the proxy sensor from left to right
   and backward.
*/
#include "Arduino.h"

#include "pins.h"
#include "ProxySensor.h"

#define SERIAL_BPS  115200
#define PERIOD    4000ul
#define HALF_PERIOD (PERIOD / 2)
#define TEST_DURATION  (PERIOD * 10)
#define FRONT_ANGLE 0

static int lastAngle;
static unsigned long timeout;
static boolean completed;
static ProxySensor sensor(SERVO_PIN, TRIGGER_PIN, ECHO_PIN);

void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.println("");
  sensor.onDataReady(handleDataReady);
  sensor.begin();
  sensor.interval(0);
  sensor.offset(-5);
  delay(500);
  Serial.println("Start.");
  timeout = millis() + TEST_DURATION;
}

void loop() {
  sensor.polling(millis());
}

static void handleDataReady(void*, ProxySensor& sensor ) {
  const int angle = sensor.echoDirection();
  const unsigned long delay = sensor.echoDelay();
  const unsigned int distance = delay * 100 / 5887;
  Serial.print("Time: ");
  Serial.print(sensor.echoTime());
  Serial.print(", ");
  Serial.print(angle);
  Serial.print(" DEG, ");
  Serial.print(delay);
  Serial.print(" ns, ");
  Serial.print(distance);
  Serial.print(" cm");
  Serial.println();
  if (angle != lastAngle) {
    lastAngle = angle;
  }
  const unsigned long t0 = millis() % PERIOD;
  // : 180 - 180 * (t0 - HALF_PERIOD) / HALF_PERIOD;
  const int angle1 = t0 <= HALF_PERIOD
                     ? 180 * t0 / HALF_PERIOD - 90
                     : 360 - 90 - 180 * t0 / HALF_PERIOD;
  if (millis() < timeout) {
    sensor.direction(angle1, millis());
  } else if (!completed) {
    completed = true;
    Serial.println("Completed.");
    sensor.interval(500);
  }
}
