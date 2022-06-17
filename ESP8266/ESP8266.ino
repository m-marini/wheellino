#include <EEPROM.h>

#include <ArduinoJson.h>
#include "FS.h"
#include <LittleFS.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include "Timer.h"

#define LED LED_BUILTIN
#define SERIAL_TIMEOUT  2000ul
#define EEPROM_ADDRESS  0
#define CURRENT_VERSION 0

#define LED_LIGHT_INTERVAL  100ul
#define MAX_ANALOG_VALUE    255
#define BLINKING_INTERVAL   2000ul
#define NUM_PULSE           (BLINKING_INTERVAL / LED_LIGHT_INTERVAL)

struct WifiData {
  int version;
  bool active;
  char ssid[128];
  char password[64];
} wifiData;

ESP8266WebServer server(80);

StaticJsonDocument<256> jsonDoc;

const char* defSSID = "Wheelly";

Timer ledTimer;

/*
 * 
 */
void setup(void) {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  analogWriteRange(MAX_ANALOG_VALUE);

  Serial.begin(115200);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println();
  Serial.println("// Mounting FS...");

  if (!LittleFS.begin()) {
    Serial.println("!! Failed to mount file system");
    while (1) ;
  }

  Serial.println("// Mounted FS");

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
    Serial.println(F("// MDNS responder started"));
  }
  ledTimer.interval(LED_LIGHT_INTERVAL)
    .continuous(true)
    .onNext([](void *, int, long i) {
      int n = i % NUM_PULSE;
      int value = abs((int) (2 * (n * MAX_ANALOG_VALUE / (NUM_PULSE - 1)) - MAX_ANALOG_VALUE));
      analogWrite(LED, value);
    })
    .start();

  server.on("/api/v1/wheelly/motors", handleMotors);
  server.on("/api/v1/wheelly/scan", handleScan);
  server.on("/api/v1/wheelly/status", handleStatus);
  server.on("/api/v1/wheelly/clock", handleClock);
  server.on("/api/v1/wheelly/networks", handleNetworkList);
  server.on("/api/v1/wheelly/networks/network", handleNetworkConnection);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println(F("// HTTP server started"));
}

/*
 * 
 */
void loop(void) {
  server.handleClient();
  MDNS.update();
  ledTimer.polling();
}

/*
 * Return true if able to load configuration
 */
bool loadConfig() {
  File configFile = LittleFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("!! Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("!! Config file size is too large");
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
    Serial.println("!! Failed to parse config file");
    return false;
  }

  if (!doc.containsKey("version")
    || !doc.containsKey("active")
    || !doc.containsKey("ssid")
    || !doc.containsKey("password")) {
    Serial.println("!! Wrong config file");
    return false;
  }
    
  wifiData.version = doc["version"];
  wifiData.active = doc["active"];
  strncpy(wifiData.ssid, doc["ssid"].as<const char *>(), sizeof(wifiData.ssid));
  strncpy(wifiData.password, doc["password"].as<const char *>(), sizeof(wifiData.password));
  
  Serial.print("// version: ");
  Serial.println(wifiData.version);
  Serial.print("// active:  ");
  Serial.println(wifiData.active);
  Serial.print("// ssid:    ");
  Serial.println(wifiData.ssid);
  return true;
}

/*
 * Returns the wifi status after connecting WiFi network
 */
wl_status_t connectWiFi() {
  Serial.print(F("// Connecting to "));
  Serial.print(wifiData.ssid);
  Serial.println(F(" ..."));

  WiFi.begin(wifiData.ssid, wifiData.password);
  wl_status_t status = waitForConnection();
  if (status == WL_CONNECTED) {
    Serial.print(F("// Connected to "));
    Serial.println(wifiData.ssid);
    Serial.print(F("// IP address "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.print(F("!! Failed with status: "));
    Serial.println(status);
  }
  return status;
}

/*
 * Returns the wifi status after createing WiFi access point
 */
wl_status_t createAccessPoint() {
  Serial.print(F("// Creating access point "));
  Serial.print(defSSID);
  Serial.println(F(" ..."));

  WiFi.softAP(defSSID, "");

  Serial.print(F("// Access point "));
  Serial.println(defSSID);
  Serial.print(F("// IP address "));
  Serial.println(WiFi.softAPIP());

  return WiFi.status();
}

/*
 *
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
 * Handles the network post to configure and store the network connection
 */
void handleNetworkConnection() {
  digitalWrite(LED, LOW);
  if (server.method() != HTTP_POST) {
    sendInvalidMethod();
    return;
  }
  String reqBody = server.arg(F("plain"));
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

  server.send(200, F("application/json"), body);
  digitalWrite(LED, HIGH);
}

/*
 * 
 */
bool saveConfig() {
  StaticJsonDocument<200> doc;
  doc["version"] = CURRENT_VERSION;
  doc["active"] = wifiData.active;
  doc["ssid"] = wifiData.ssid;
  doc["password"] = wifiData.password;

  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return false;
  }

  serializeJson(doc, configFile);
  return true;
}

/*
 *
 */
void handleMotors() {
  digitalWrite(LED, LOW);
  if (server.method() != HTTP_POST) {
    sendInvalidMethod();
    return;
  }
  String reqBody = server.arg(F("plain"));
  DeserializationError err = deserializeJson(jsonDoc, reqBody);
  if (err) {
    sendInvalidContent(err.c_str());
    return;
  }
  if (!jsonDoc.containsKey(F("left"))
      || !jsonDoc.containsKey(F("right"))
      || !jsonDoc.containsKey(F("validTo"))) {
    sendInvalidContent("Missing parameters");
    return;
  }
  int left = jsonDoc[F("left")].as<const int>();
  int right = jsonDoc[F("right")].as<const int>();
  unsigned long validTo = jsonDoc[F("validTo")].as<const unsigned long>();

  serialCleanup();
  Serial.print(F("mt "));
  Serial.print(validTo);
  Serial.print(F(" "));
  Serial.print(left);
  Serial.print(F(" "));
  Serial.print(right);
  Serial.println();
  Serial.flush();
  String resBody = waitForStatus();
  if (resBody == "") {
    sendTimeoutError();
    return;
  }
  server.send(200, F("application/json"), resBody);
  digitalWrite(LED, HIGH);
}

/*
 * 
 */
void handleStatus() {
  digitalWrite(LED, LOW);
  if (server.method() != HTTP_GET) {
    sendInvalidMethod();
    return;
  }
  serialCleanup();
  Serial.println(F("qs"));
  Serial.flush();
  String body = waitForStatus();
  if (body == "") {
    sendTimeoutError();
    return;
  }
  server.send(200, F("application/json"), body);
  digitalWrite(LED, HIGH);
}

/*
 * 
 */
void handleNetworkList() {
  String ssid;
  int32_t rssi;
  uint8_t encryptionType;
  uint8_t* bssid;
  int32_t channel;
  bool hidden;
  int scanResult;

  digitalWrite(LED, LOW);
  scanResult = WiFi.scanNetworks(false, true);
  jsonDoc.clear();
  JsonArray data = jsonDoc.createNestedArray("networks");

  for (int8_t i = 0; i < scanResult; i++) {
    WiFi.getNetworkInfo(i, ssid, encryptionType, rssi, bssid, channel, hidden);
    data.add(ssid);
  }
  String body;
  serializeJson(jsonDoc, body);
  server.send(200, F("application/json"), body);
  digitalWrite(LED, HIGH);
}

/*
 *
 */
void handleClock() {
  digitalWrite(LED, LOW);
  if (server.method() != HTTP_GET) {
    sendInvalidMethod();
    return;
  }
  String ck = server.arg(F("ck"));
  serialCleanup();
  Serial.print(F("ck "));
  Serial.println(ck);
  Serial.flush();
  String result = readTrimmedLine();
  if (result == "") {
    sendTimeoutError();
    return;
  }
  jsonDoc.clear();
  jsonDoc[F("clock")] = result;
  String message;
  serializeJson(jsonDoc, message);
  server.send(200, F("application/json"), message);
  digitalWrite(LED, HIGH);
}

/*
 * Returns the trimmed line from serial
 */
String readTrimmedLine() {
  String result = Serial.readStringUntil('\n');
  result.trim();
  return result;
}

/*
 *
 */
void handleScan() {
  digitalWrite(LED, LOW);
  if (server.method() != HTTP_POST) {
    sendInvalidMethod();
    return;
  }
  serialCleanup();
  Serial.println(F("sc"));
  Serial.flush();
  String body = waitForStatus();
  if (body == "") {
    sendTimeoutError();
    return;
  }
  server.send(200, F("application/json"), body);
  digitalWrite(LED, HIGH);
}

/*
 *
 */
void handleNotFound() {
  digitalWrite(LED, LOW);
  jsonDoc.clear();
  jsonDoc[F("message")] = F("Not Found");
  jsonDoc[F("uri")] = server.uri();
  jsonDoc[F("method")] = methodName();
  String body;
  serializeJson(jsonDoc, body);
  server.send(404, F("application/json"), body);
  digitalWrite(LED, HIGH);
}

/*
 *
 */
void serialCleanup() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

/*
 * 
 */
String waitForStatus() {
  String result = readTrimmedLine();
  if (result != "") {
    jsonDoc.clear();
    jsonDoc[F("status")] = result;
    String body;
    serializeJson(jsonDoc, body);
    return body;
  } else {
    return result;
  }
}

/*
 *
 */
String methodName() {
  switch (server.method()) {
    case HTTP_GET: return "GET";
    case HTTP_POST: return "POST";
    case HTTP_DELETE: return "DELETE";
    default: return "UNKNOWN";
  }
}

/*
 *
 */
void sendInvalidMethod() {
  jsonDoc.clear();
  jsonDoc[F("message")] = F("Method Not Allowed");
  jsonDoc[F("method")] = methodName();
  String body;
  serializeJson(jsonDoc, body);
  server.send(405, F("application/json"), body);
  digitalWrite(LED, HIGH);
}

/*
 *
 */
void sendInvalidContent(const char *msg) {
  jsonDoc.clear();
  jsonDoc[F("message")] = msg;
  String body;
  serializeJson(jsonDoc, body);
  server.send(400, F("application/json"), body);
  digitalWrite(LED, HIGH);
}

/*
 *
 */
void sendTimeoutError() {
  jsonDoc.clear();
  jsonDoc[F("message")] = F("Timeout");
  String body;
  serializeJson(jsonDoc, body);
  server.send(504, F("application/json"), body);
  digitalWrite(LED, HIGH);
}
