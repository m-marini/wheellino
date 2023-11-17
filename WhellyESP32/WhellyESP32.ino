#include <stdio.h>
#include <Arduino.h>
#include <Wire.h>

//#define DEBUG
#include "debug.h"

#include "Wheelly.h"
#include "WiFiModule.h"
#include "TelnetServer.h"
#include "ApiServer.h"
#include "CommandInterpreter.h"

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
   Command interpreter
*/
static CommandInterpreter commandInterpreter(wheelly);

/*
   Initial setup
*/
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println();
  Wire.begin();
  Wire.setClock(WIRE_CLOCK); // 400kHz I2C clock. Comment this line if having compilation difficulties

  delay(100);
  Serial.println();

  wheelly.begin();
  wheelly.onReply([](void *, const char* data) {
    // Handles reply to remote controller
    DEBUG_PRINTLN(data);
    telnetServer.println(data);
  });

  ApiServer.wiFiModule(&wiFiModule);
  ApiServer.onActivity([](void *, ApiServerClass&) {
    wheelly.activity();
  });

  wiFiModule.onChange(handleOnChange);
  wiFiModule.begin();

  telnetServer.onLineReady(handleLineReady);
  telnetServer.onClient(handleOnClient);
  telnetServer.onActivity([](void *, TelnetServerClass&) {
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
static void handleOnClient(void*, TelnetServerClass& telnet) {
  boolean hasClient = telnet.hasClient();
  DEBUG_PRINTLN(hasClient
                ? "// Client connected"
                : "// Client disconnected");
  wheelly.connected(hasClient);
}

/*
   Polls the serial port for command
*/
static void pollSerialPort(const unsigned long time) {
  if (Serial.available()) {
    const size_t n = Serial.readBytesUntil('\n', line, sizeof(line) - 1);
    line[n] = 0;
    commandInterpreter.execute(time, line);
  }
}

/*
   Handles the wifi module state change
*/
static void handleOnChange(void *, WiFiModuleClass & module) {
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
static void handleLineReady(void *, const char* line) {
  commandInterpreter.execute(millis(), line);
}
