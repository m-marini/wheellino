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
   Test of LCD display

   The test show the display functions
*/
#include "Arduino.h"

#include <esp_log.h>
static const char* TAG = "LCDTest";

#include "Display.h"

#define SERIAL_BPS 115200

#define DISPLAY_INTERVAL 10000
#define LEVEL_INTERVAL 200

static DisplayClass Display;

void setup() {
  Serial.begin(SERIAL_BPS);
  while (!Serial){
    delay(10);
  }
  ESP_LOGI(TAG, "Starting ...");

  Display.begin();

  ESP_LOGI(TAG, "Testing supply level");
  Display.clear();
  Display.showWiFiInfo("Testing supply level");
  Display.error(0);
  Display.move(false);
  Display.distance(-1);
  Display.block(NO_BLOCK);
  Display.connected(false);
  for (int i = 0; i < 10; i++) {
    for (int level = 0; level < 6; level++) {
      Display.supply(level);
      waitForTimeout(LEVEL_INTERVAL);
    }
  }

  ESP_LOGI(TAG, "Testing distance 5 cm");
  Display.clear();
  Display.showWiFiInfo("Testing distance 5 cm");
  Display.error(1);
  Display.block(FULL_BLOCK);
  Display.move(true);
  Display.distance(5);
  Display.connected(true);
  Display.activity();
  waitForTimeout(DISPLAY_INTERVAL);

  ESP_LOGI(TAG, "Testing distance 95 cm");
  Display.clear();
  Display.showWiFiInfo("Testing distance 95 cm");
  Display.error(2);
  Display.move(true);
  Display.block(FORWARD_BLOCK);
  Display.distance(95);
  Display.connected(true);
  Display.activity();
  waitForTimeout(DISPLAY_INTERVAL);

  ESP_LOGI(TAG, "Testing distance 345 cm");
  Display.clear();
  Display.showWiFiInfo("Testing distance 345 cm");
  Display.error(3);
  Display.move(true);
  Display.distance(345);
  Display.block(BACKWARD_BLOCK);
  Display.connected(true);
  Display.activity();
  waitForTimeout(DISPLAY_INTERVAL);

  ESP_LOGI(TAG, "Testing no distance");
  Display.clear();
  Display.showWiFiInfo("Testing no distance");
  Display.error(0);
  Display.move(false);
  Display.distance(-1);
  Display.block(NO_BLOCK);
  Display.connected(false);
  waitForTimeout(DISPLAY_INTERVAL);

  Display.error(0);
  Display.showWiFiInfo("");
  Display.distance(-1);
  Display.move(false);
  Display.block(NO_BLOCK);
  Display.connected(false);
  waitForTimeout(DISPLAY_INTERVAL);

  ESP_LOGI(TAG, "Completed");
  Display.clear();
}

void loop() {
  delay(1000);
}

static void waitForTimeout(const unsigned long interval) {
  unsigned long t0 = millis();
  const unsigned long timeout = t0 + interval;
  do {
    t0 = millis();
    Display.polling(t0);
  } while (t0 < timeout);
}
