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
#include "Timer.h"

//#define DEBUG
#include "debug.h"

#define SERIAL_BPS 115200
#define INTERVAL 100

static Timer timer;

void setup() {
  Serial.begin(SERIAL_BPS);
  delay(500);
  Serial.println("");
  pinMode(STATUS_LED_PIN, OUTPUT);
  timer.interval(INTERVAL);
  timer.continuous(true);
  timer.onNext(handleTimer);

  Serial.println("Start");
  timer.start();
}

void loop() {
  timer.polling();
}

static void handleTimer(void*, const unsigned long n) {
  digitalWrite(STATUS_LED_PIN, (n % 2) != 0);
}
