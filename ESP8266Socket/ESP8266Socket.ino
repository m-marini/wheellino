#include <EEPROM.h>

//#define DEBUG
#include "debug.h"

#include <ArduinoJson.h>
#include "FS.h"
#include <LittleFS.h>

#include <ESP8266WiFi.h>
//#include <WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include "Timer.h"

#define LED LED_BUILTIN
#define SERIAL_TIMEOUT  2000ul
#define EEPROM_ADDRESS  0
#define CURRENT_VERSION 0

#define LED_LIGHT_INTERVAL  50ul
#define ACTIVITY_INTERVAL   500ul

#define MAX_ANALOG_VALUE    255

#define NUM_PULSES          40ul
#define FAST_NUM_PULSES     10ul
#define ACTIVITY_NUM_PULSES 2ul

struct WifiData {
  int version;
  bool active;
  char ssid[128];
  char password[64];
} wifiData;

WiFiServer server(22);
ESP8266WebServer webServer(80);

StaticJsonDocument<256> jsonDoc;

const char* defSSID = "Wheelly";

Timer ledTimer;

bool hasClient;
WiFiClient client;
bool activity;
unsigned long clearActivityTime;

/*

*/
void setup(void) {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  analogWriteRange(MAX_ANALOG_VALUE);

  Serial.begin(115200);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println();
  DEBUG_PRINTLN(F("// Mounting FS..."));

  if (!LittleFS.begin()) {
    DEBUG_PRINTLN(F("!! Failed to mount file system"));
    while (1) ;
  }

  DEBUG_PRINTLN(F("// Mounted FS"));

  bool loaded = loadConfig();

  WiFi.mode(WIFI_STA);

  if (loaded && wifiData.version == CURRENT_VERSION
      && wifiData.active) {
    if (connectWiFi() != WL_CONNECTED) {
      createAccessPoint();
    }
  } else {
    createAccessPoint();
  }

  if (MDNS.begin("esp8266")) {
    DEBUG_PRINTLN(F("// MDNS responder started"));
  }
  ledTimer.interval(LED_LIGHT_INTERVAL);
  ledTimer.continuous(true);
  ledTimer.onNext(handleLed);
  ledTimer.start();

  server.begin();

  webServer.on(F("/api/v1/wheelly/networks"), handleNetworkList);
  webServer.on(F("/api/v1/wheelly/networks/network"), handleNetworkConnection);
  webServer.onNotFound(handleNotFound);

  webServer.begin();
  DEBUG_PRINTLN(F("// Server started"));
}

/*

*/
void loop(void) {
  unsigned long now = millis();
  if (!hasClient) {
    client = server.available();
    if (client) {
      hasClient = true;
    }
  }
  if (hasClient) {
    if (client && client.connected()) {
      handleClient();
    } else {
      client.stop();
      hasClient = false;
      activity = false;
    }
  } else {
    flushSerial();
  }
  MDNS.update();
  if (activity && now >= clearActivityTime) {
    activity = false;
  }
  webServer.handleClient();
  ledTimer.polling();
}

void flushSerial() {
  while (Serial.available()) {
    Serial.read();
  }
}

void handleClient() {
  boolean comunicating = false;
  while (client.available() > 0) {
    comunicating = true;
    char c = client.read();
    Serial.write(c);
  }
  while (Serial.available()) {
    comunicating = true;
    char ch = Serial.read();
    client.write(ch);
  }
  if (comunicating) {
    setActivity();
  }
}

/**

*/
void handleLed(void *, unsigned long i) {
  unsigned long numPulses = activity ? ACTIVITY_NUM_PULSES
                            : hasClient ? FAST_NUM_PULSES
                            : NUM_PULSES;
  int n = abs(((long) (i % numPulses)) - (long)(numPulses / 2));

  int value = map((int)n, 0, numPulses / 2, MAX_ANALOG_VALUE, 0);
  //int value = abs((int) (2 * (n * MAX_ANALOG_VALUE / (numPulses - 1)) - MAX_ANALOG_VALUE));
  analogWrite(LED, value);
}

/*

*/
void handleNetworkList() {
  String ssid;
  int32_t rssi;
  uint8_t encryptionType;
  uint8_t* bssid;
  int32_t channel;
  bool hidden;
  int scanResult;

  scanResult = WiFi.scanNetworks(false, true);
  jsonDoc.clear();
  JsonArray data = jsonDoc.createNestedArray("networks");

  for (int8_t i = 0; i < scanResult; i++) {
    WiFi.getNetworkInfo(i, ssid, encryptionType, rssi, bssid, channel, hidden);
    data.add(ssid);
  }
  String body;
  serializeJson(jsonDoc, body);
  webServer.send(200, F("application/json"), body);
  setActivity();
}

/*
   Handles the network post to configure and store the network connection
*/
void handleNetworkConnection() {
  if (webServer.method() != HTTP_POST) {
    sendInvalidMethod();
    return;
  }
  String reqBody = webServer.arg(F("plain"));
  DeserializationError err = deserializeJson(jsonDoc, reqBody);
  if (err) {
    sendInvalidContent(err.c_str());
    return;
  }
  if (!jsonDoc.containsKey(F("active"))
      || !jsonDoc.containsKey(F("ssid"))
      || !jsonDoc.containsKey(F("password"))) {
    sendInvalidContent("Missing parameters");
    return;
  }
  wifiData.version = CURRENT_VERSION;
  wifiData.active = jsonDoc[F("active")].as<bool>();
  strncpy(wifiData.ssid, jsonDoc[F("ssid")].as<const char*>(), sizeof(wifiData.ssid) - 1);
  strncpy(wifiData.password, jsonDoc[F("password")].as<const char*>(), sizeof(wifiData.password) - 1);

  saveConfig();
  loadConfig();

  String body;
  serializeJson(jsonDoc, body);

  webServer.send(200, F("application/json"), body);
  setActivity();
}

/*

*/
void handleNotFound() {
  jsonDoc.clear();
  jsonDoc[F("message")] = F("Not Found");
  jsonDoc[F("uri")] = webServer.uri();
  jsonDoc[F("method")] = methodName();
  String body;
  serializeJson(jsonDoc, body);
  webServer.send(404, F("application/json"), body);
  setActivity();
}

/*

*/
void setActivity() {
  activity = true;
  clearActivityTime = millis() + ACTIVITY_INTERVAL;
}

/*
   Return true if able to load configuration
*/
bool loadConfig() {
  File configFile = LittleFS.open(F("/config.json"), "r");
  if (!configFile) {
    DEBUG_PRINTLN(F("!! Failed to open config file"));
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    DEBUG_PRINTLN(F("!! Config file size is too large"));
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  StaticJsonDocument<200> doc;
  auto error = deserializeJson(doc, buf.get());
  if (error) {
    DEBUG_PRINTLN(F("!! Failed to parse config file"));
    return false;
  }

  if (!doc.containsKey(F("version"))
      || !doc.containsKey(F("active"))
      || !doc.containsKey(F("ssid"))
      || !doc.containsKey(F("password"))) {
    DEBUG_PRINTLN(F("!! Wrong config file"));
    return false;
  }

  wifiData.version = doc[F("version")];
  wifiData.active = doc[F("active")];
  strncpy(wifiData.ssid, doc[F("ssid")].as<const char *>(), sizeof(wifiData.ssid));
  strncpy(wifiData.password, doc[F("password")].as<const char *>(), sizeof(wifiData.password));

  DEBUG_PRINT(F("// version: "));
  DEBUG_PRINTLN(wifiData.version);
  DEBUG_PRINT(F("// active:  "));
  DEBUG_PRINTLN(wifiData.active);
  DEBUG_PRINT(F("// ssid:    "));
  DEBUG_PRINTLN(wifiData.ssid);
  return true;
}

/*
   Returns the wifi status after connecting WiFi network
*/
wl_status_t connectWiFi() {
  DEBUG_PRINT(F("// Connecting to "));
  DEBUG_PRINT(wifiData.ssid);
  DEBUG_PRINTLN(F(" ..."));

  WiFi.begin(wifiData.ssid, wifiData.password);
  wl_status_t status = waitForConnection();
  if (status == WL_CONNECTED) {
    DEBUG_PRINT(F("// Connected to "));
    DEBUG_PRINTLN(wifiData.ssid);
    DEBUG_PRINT(F("// IP address "));
    DEBUG_PRINTLN(WiFi.localIP());
  } else {
    DEBUG_PRINT(F("!! Failed with status: "));
    DEBUG_PRINTLN(status);
  }
  return status;
}

/*
   Returns the wifi status after createing WiFi access point
*/
wl_status_t createAccessPoint() {
  DEBUG_PRINT(F("// Creating access point "));
  DEBUG_PRINT(defSSID);
  DEBUG_PRINTLN(F(" ..."));

  WiFi.softAP(defSSID, "");

  DEBUG_PRINT(F("// Access point "));
  DEBUG_PRINTLN(defSSID);
  DEBUG_PRINT(F("// IP address "));
  DEBUG_PRINTLN(WiFi.softAPIP());

  return WiFi.status();
}

/*

*/
wl_status_t waitForConnection() {
  // Wait for connection
  wl_status_t status;
  for (;;) {
    status = WiFi.status();
    if (status != WL_DISCONNECTED) {
      break;
    }
    digitalWrite(LED, LOW);
    delay(50);
    digitalWrite(LED, HIGH);
    delay(350);
  }
  return status;
}

/*

*/
bool saveConfig() {
  StaticJsonDocument<200> doc;
  doc[F("version")] = CURRENT_VERSION;
  doc[F("active")] = wifiData.active;
  doc[F("ssid")] = wifiData.ssid;
  doc[F("password")] = wifiData.password;

  File configFile = LittleFS.open(F("/config.json"), "w");
  if (!configFile) {
    DEBUG_PRINTLN(F("!! Failed to open config file for writing"));
    return false;
  }

  serializeJson(doc, configFile);
  return true;
}

/*

*/
String methodName() {
  switch (webServer.method()) {
    case HTTP_GET: return F("GET");
    case HTTP_POST: return F("POST");
    case HTTP_DELETE: return F("DELETE");
    default: return F("UNKNOWN");
  }
}

/*

*/
void sendInvalidMethod() {
  jsonDoc.clear();
  jsonDoc[F("message")] = F("Method Not Allowed");
  jsonDoc[F("method")] = methodName();
  String body;
  serializeJson(jsonDoc, body);
  webServer.send(405, F("application/json"), body);
  setActivity();
}

/*

*/
void sendInvalidContent(const char *msg) {
  jsonDoc.clear();
  jsonDoc[F("message")] = msg;
  String body;
  serializeJson(jsonDoc, body);
  webServer.send(400, F("application/json"), body);
  setActivity();
}
