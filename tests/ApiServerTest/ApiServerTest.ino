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

#include "Arduino.h"
#include "WiFiModule.h"
#include "ApiServer.h"

#define DEBUG
#include "debug.h"

#define SERIAL_BPS 115200

static WiFiModuleClass wiFiModule;

void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.println();

  ApiServer.wiFiModule(&wiFiModule);
  ApiServer.onActivity([](void*, ApiServerClass&) {
    Serial.print("ApiServer activity");
    Serial.println();
  });

  wiFiModule.onChange(handleOnChange);
  wiFiModule.begin();
}

void loop() {
  const unsigned long now = millis();
  wiFiModule.polling(now);
  ApiServer.polling(now);
}

static void handleOnChange(void* context, WiFiModuleClass& module) {
  char bfr[256];
  if (module.connected()) {
    DEBUG_PRINTLN("// ApiServer.begin()");
    ApiServer.begin();
    strcpy(bfr, module.ssid());
    strcat(bfr, " - IP: ");
    strcat(bfr, module.ipAddress().toString().c_str());
  } else if (module.connecting()) {
    strcpy(bfr, module.ssid());
    strcat(bfr, " connecting...");
  } else {
    strcpy(bfr, "Disconnected");
  }
  Serial.print(bfr);
  Serial.println();
}
