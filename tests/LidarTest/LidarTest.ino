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

#include <stdio.h>
#include <Arduino.h>
#include <Wire.h>

#include <esp_log.h>

static char* TAG = "LidarTest";

#include "Lidar.h"
#include "pins.h"

static const unsigned WIRE_CLOCK = 400000;

/*
   Serial line buffer
*/
static const unsigned long SERIAL_TIMEOUT = 2000ul;

static Lidar lidar(FRONT_LIDAR_PIN, REAR_LIDAR_PIN);

static void handleLidarRange(void* context, Lidar& lidar, const uint16_t frontRange, const uint16_t rearRange) {
  ESP_LOGI(TAG, "Distances %u mm, %u mm", frontRange, rearRange);
}

/*
   Initial setup
*/
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(SERIAL_TIMEOUT);
  while (!Serial) {
    delay(1);
  }

  Wire.begin();
  Wire.setClock(WIRE_CLOCK);  // 400kHz I2C clock. Comment this line if having compilation difficulties

  // Initialises the configuration store (load the configuration)
  delay(1000);
  Serial.println();
  ESP_LOGI(TAG, "Startup sequence");

  if (!lidar.begin()) {
    ESP_LOGE(TAG, "Lidar begin failed");
    while (true) {
      delay(1000);
    }
  }

  lidar.onRange(&handleLidarRange);

  ESP_LOGI(TAG, "Startup sequence completed");
}

/*
   The loop function
*/
void loop() {
  const unsigned long now = millis();
  lidar.polling(now);
}
