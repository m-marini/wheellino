#include "Arduino.h"
#include "WiFiModule.h"
#include "TelnetServer.h"
#include "ApiServer.h"

#define SERIAL_BPS  115200

static WiFiModuleClass wiFiModule;
static TelnetServerClass telnetServer;

void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.println();

  ApiServer.wiFiModule(&wiFiModule);
  ApiServer.onActivity([](void *, ApiServerClass&) {
    Serial.print("ApiServer activity");
    Serial.println();
  });

  wiFiModule.onChange(handleOnChange);
  wiFiModule.begin();

  telnetServer.onLineReady(handleLineReady);
  telnetServer.onClient(handleOnClient);
  telnetServer.onActivity([](void *, TelnetServerClass&) {
    Serial.print("TelnetServer activity");
    Serial.println();
  });

}

void loop() {
  const unsigned long now = millis();
  wiFiModule.polling(now);
  telnetServer.polling(now);
  ApiServer.polling(now);
}

static void handleOnChange(void* context, WiFiModuleClass& module) {
  char bfr[256];
  if (module.connected()) {
    ApiServer.begin();
    telnetServer.begin();
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

static void handleOnClient(void*, TelnetServerClass& telnet) {
  Serial.print("Client ");
  Serial.print(telnet.hasClient() ? "connected" : "disconnected");
  Serial.println();
}

static void handleLineReady(void* context, const char* line) {
  Serial.print("--> ");
  Serial.print(line);
  Serial.println();
  telnetServer.println(line);
  Serial.print("<-- ");
  Serial.print(line);
  Serial.println();
}
