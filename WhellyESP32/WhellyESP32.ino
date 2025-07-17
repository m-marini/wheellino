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

#define DEBUG
#include "debug.h"

#include "Wheelly.h"
#include "WiFiModule.h"
#include "TelnetServer.h"
#include "ApiServer.h"

static const unsigned WIRE_CLOCK = 400000;

/*
   Serial line buffer
*/
static const unsigned long SERIAL_TIMEOUT = 2000ul;
static char line[100];

/*
   WiFi module
*/
static WiFiModuleClass wiFiModule;

/*
   Telnet server
*/
static TelnetServerClass telnetServer;

/*
   Wheelly controller
*/
static Wheelly wheelly;

/*
   Initial setup
*/
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println();
  Wire.begin();
  Wire.setClock(WIRE_CLOCK);  // 400kHz I2C clock. Comment this line if having compilation difficulties

  delay(100);
  Serial.println();

  wheelly.begin();
  wheelly.onReply([](void *, const char *data) {
    // Handles reply to remote controller
    DEBUG_PRINTLN(data);
    telnetServer.println(data);
  });

  ApiServer.wiFiModule(&wiFiModule);
  ApiServer.onActivity([](void *, ApiServerClass &) {
    wheelly.activity();
  });

  wiFiModule.onChange(handleOnChange);
  wiFiModule.begin();

  telnetServer.onLineReady(handleLineReady);
  telnetServer.onClient(handleOnClient);
  telnetServer.onActivity([](void *, TelnetServerClass &) {
    wheelly.activity();
  });
}

/*
   The loop function
*/
void loop() {
  const unsigned long now = millis();

  wiFiModule.polling(now);
  telnetServer.polling(now);
  ApiServer.polling(now);
  pollSerialPort(now);
  wheelly.polling(now);
}

/*
  Handles the wifi client connection, disconnection
*/
static void handleOnClient(void *, TelnetServerClass &telnet) {
  boolean hasClient = telnet.hasClient();
  DEBUG_PRINTLN(hasClient
                  ? "// Client connected"
                  : "// Client disconnected");
  wheelly.connected(hasClient);
  wheelly.queryStatus();
}

/*
   Polls the serial port for command
*/
static void pollSerialPort(const unsigned long time) {
  if (Serial.available()) {
    const size_t n = Serial.readBytesUntil('\n', line, sizeof(line) - 1);
    line[n] = 0;
    wheelly.execute(time, line);
  }
}

/*
   Handles the wifi module state change
*/
static void handleOnChange(void *, WiFiModuleClass &module) {
  char bfr[256];
  boolean connected = module.connected();
  if (connected) {
    ApiServer.begin();
    telnetServer.begin();
    strcpy(bfr, wiFiModule.ssid());
    strcat(bfr, " - IP: ");
    strcat(bfr, wiFiModule.ipAddress().toString().c_str());
  } else if (module.connecting()) {
    strcpy(bfr, wiFiModule.ssid());
    strcat(bfr, " connecting...");
  } else {
    strcpy(bfr, "Disconnected");
  }
  wheelly.onLine(connected);
  wheelly.display(bfr);
}

/*
   Handles line ready from wifi
   @param line the line
*/
static void handleLineReady(void *, const char *line) {
  wheelly.execute(millis(), line);
}
