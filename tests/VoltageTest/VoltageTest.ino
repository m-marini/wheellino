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

#include "pins.h"

//#define DEBUG
#include "debug.h"

#define SERIAL_BPS 115200
#define SAMPLE_INTERVAL 100
#define AVERAGE_INTERVAL 5000

void setup() {
  Serial.begin(SERIAL_BPS);
  delay(500);
  Serial.println("");
  pinMode(VOLTAGE_PIN, INPUT);

  Serial.println("Start");
}

static unsigned long sampleTimeout;
static unsigned long averageTimeout;
static long total;
static int count;

void loop() {
  // put your main code here, to run repeatedly:
  const unsigned long t0 = millis();
  if (t0 >= sampleTimeout) {
    total += analogRead(VOLTAGE_PIN);
    count++;
    sampleTimeout = t0 + SAMPLE_INTERVAL;
  }
  if (t0 >= averageTimeout) {
    if (count > 0) {
      int supply = total / count;
      Serial.print("Supply: ");
      Serial.print(supply);
      Serial.print(", count: ");
      Serial.print(count);
      Serial.println();
      total = 0;
      count = 0;
    }
    averageTimeout = t0 + AVERAGE_INTERVAL;
  }
}
