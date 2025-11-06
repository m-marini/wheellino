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

#include "Contacts.h"
#include "pins.h"

#include <esp_log.h>
static const char* TAG = "ContactsTest";

#define SERIAL_BPS 115200

#define INTERVAL 100

static ContactSensors sensors(FRONT_CONTACTS_PIN, REAR_CONTACTS_PIN);

void setup() {
  Serial.begin(SERIAL_BPS);
  while (!Serial) {
    delay(10);
  }
  ESP_LOGI(TAG, "Begin");
  sensors.begin();
  ESP_LOGI(TAG, "Start");
}

static boolean prevFront;
static boolean prevRear;

unsigned long timeout;

void loop() {
  const unsigned long t0 = millis();
  sensors.polling(t0);
  const boolean front = sensors.frontClear();
  const boolean rear = sensors.rearClear();
  if (front != prevFront || rear != prevRear) {
    prevFront = front;
    prevRear = rear;
    ESP_LOGI(TAG, "%s, %s",
             front ? "  front" : "X front",
             rear ? "   rear" : "X rear");
  }
  delay(INTERVAL);
}
