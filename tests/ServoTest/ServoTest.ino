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
   Test of asynchronous servo.

   The test run for 40 sec. moving the proxy sensor from left to right
   and backward.
*/
#include "Arduino.h"

#include <esp_log.h>
static const char* TAG = "ServoTest";

#include "pins.h"
#include "LidarServo.h"

#define SERIAL_BPS 115200
#define PERIOD 4000ul
#define HALF_PERIOD (PERIOD / 2)
#define TEST_DURATION (PERIOD * 10)
#define FRONT_ANGLE 0

static int lastAngle;
static unsigned long timeout;
static boolean completed;
static LidarServo sensor(SERVO_PIN);

void setup() {
  Serial.begin(SERIAL_BPS);
  while (!Serial) {
    delay(10);
  }
  ESP_LOGI(TAG, "Setting up ...");
  sensor.onPosition(handleDataReady);
  sensor.offset(-5);
  sensor.begin();
  ESP_LOGI(TAG, "Start.");
  timeout = millis() + TEST_DURATION;
  sensor.direction(0, millis());
}

void loop() {
  sensor.polling(millis());
}

static void handleDataReady(void*, LidarServo& sensor) {
  const unsigned long t0 = millis();
  const int angle = sensor.direction();
  ESP_LOGI(TAG, "%d DEG", angle);
  if (angle != lastAngle) {
    lastAngle = angle;
  }
  const unsigned long dt = t0 % PERIOD;
  // : 180 - 180 * (t0 - HALF_PERIOD) / HALF_PERIOD;
  const int angle1 = dt <= HALF_PERIOD
                       ? 180 * dt / HALF_PERIOD - 90
                       : 360 - 90 - 180 * dt / HALF_PERIOD;
  if (t0 < timeout) {
    sensor.direction(angle1, t0);
  } else if (!completed) {
    completed = true;
    ESP_LOGI(TAG, "Completed.");
  }
}
