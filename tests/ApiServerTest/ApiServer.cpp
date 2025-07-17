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

#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <esp_system.h>

#include "ApiServer.h"

//#define DEBUG
#include "debug.h"

#define HTTP_METHOD_GET ((HTTPMethod)1)
#define HTTP_METHOD_POST ((HTTPMethod)3)
#define RESTART_DELAY 500

static const char *DEFAULT_SERVER_NAME = "wheelly";
static const unsigned long MAX_SCAN_DURATION = 1000000;

ApiServerClass ApiServer;


/*
  Send error
*/
void sendError(const int httpCode, const String &msg) {
  StaticJsonDocument<256> jsonDoc;
  String response;
  jsonDoc["message"] = msg;
  serializeJson(jsonDoc, response);
  ApiServer._server.send(httpCode, "application/json", response);
  ApiServer.setActivity();
}

static const String methodName(const HTTPMethod method) {
  String result("Unknown");
  switch (method) {
    case HTTP_METHOD_GET:
      result = "GET";
      break;
  }
  return result;
}

/*
  Handles not found page
*/
void handleNotFound(void) {
  DEBUG_PRINT("// handleNotFound method ");
  DEBUG_PRINTLN(ApiServer._server.method());
  StaticJsonDocument<256> jsonDoc;
  jsonDoc.clear();
  jsonDoc["message"] = "Not Found";
  jsonDoc["url"] = ApiServer._server.uri();
  jsonDoc["method"] = methodName(ApiServer._server.method());
  String response;
  serializeJson(jsonDoc, response);
  ApiServer._server.send(404, "application/json", response);
  ApiServer.setActivity();
}

/*
   Handles netwok list request
*/
void handleGetNetworkList(void) {
  DEBUG_PRINTLN("// Scanning networks...");
  ApiServer.setActivity();
  StaticJsonDocument<256> jsonDoc;
  JsonArray data = jsonDoc.createNestedArray("networks");
  int scanResult = WiFi.scanNetworks(false, true);
  for (int i = 0; i < scanResult; i++) {
    data.add(WiFi.SSID(i));
  }
  String response;
  serializeJson(jsonDoc, response);
  ApiServer._server.send(200, "application/json", response);
}

/*
   Handles the wifi configuration request
*/
void handleGetWiFiConfig(void) {
  DEBUG_PRINTLN("// handleGetWiFiConfig");
  WiFiModuleClass *wiFiModule = ApiServer._wiFiModule;
  if (!wiFiModule) {
    sendError(404, "WiFi configuration not available");
    return;
  }
  const WiFiConfig &wifiData = wiFiModule->config();
  StaticJsonDocument<256> jsonDoc;
  jsonDoc.clear();
  jsonDoc["version"] = CURRENT_VERSION;
  jsonDoc["active"] = wifiData.active;
  jsonDoc["ssid"] = wifiData.ssid;
  jsonDoc["password"] = wifiData.password;

  String response;
  serializeJson(jsonDoc, response);
  ApiServer._server.send(200, "application/json", response);
  ApiServer.setActivity();
}

/*
   Handles the wifi configuration request
*/
void handlePostWiFiConfig(void) {
  DEBUG_PRINTLN("// handlePostWiFiConfig");
  WiFiModuleClass *wiFiModule = ApiServer._wiFiModule;
  if (!wiFiModule) {
    sendError(404, "WiFi configuration not available");
    return;
  }
  const WiFiConfig &wifiData = wiFiModule->config();
  // Retrive body
  String body = ApiServer._server.arg("plain");
  DEBUG_PRINT("// Body=");
  DEBUG_PRINTLN(body);
  JsonDocument jsonDoc;
  if (deserializeJson(jsonDoc, body) != DeserializationError::Ok) {
    sendError(400, "Malformed JSON");
    return;
  }
  if (!jsonDoc.containsKey("active")
      || !jsonDoc.containsKey("ssid")
      || !jsonDoc.containsKey("password")) {
    sendError(400, "Missing parameters");
    return;
  }
  WiFiConfig config;
  config.version = CURRENT_VERSION;
  config.active = jsonDoc["active"].as<bool>();
  strncpy(config.ssid, jsonDoc["ssid"].as<const char *>(), sizeof(config.ssid) - 1);
  strncpy(config.password, jsonDoc["password"].as<const char *>(), sizeof(config.password) - 1);

  String response;
  serializeJson(jsonDoc, response);
  wiFiModule->setConfig(config);
  Serial.print("WiFi configuration: ");
  Serial.println(response);
  ApiServer._server.send(200, "application/json", response);
  ApiServer.setActivity();
}

/*
   Handles the wifi configuration request
*/
void handlePostRestart(void) {
  DEBUG_PRINTLN("// handleRestart");
  WiFiModuleClass *wiFiModule = ApiServer._wiFiModule;
  if (!wiFiModule) {
    sendError(404, "WiFi configuration not available");
    return;
  }
  const WiFiConfig &wifiData = wiFiModule->config();
  StaticJsonDocument<256> jsonDoc;
  jsonDoc.clear();
  jsonDoc["version"] = CURRENT_VERSION;
  jsonDoc["active"] = wifiData.active;
  jsonDoc["ssid"] = wifiData.ssid;
  jsonDoc["password"] = wifiData.password;
  jsonDoc["restart"] = true;

  String response;
  serializeJson(jsonDoc, response);
  ApiServer._server.send(200, "application/json", response);
  ApiServer.setActivity();
  ApiServer._restartInstant = millis() + RESTART_DELAY;
}

void ApiServerClass::begin(void) {
  DEBUG_PRINTLN("// Starting api server ...");

  if (!MDNS.begin(DEFAULT_SERVER_NAME)) {
    Serial.println("!! Error starting dns responder.");
  } else {
    DEBUG_PRINTLN("// Started MDNS");
  }
  DEBUG_PRINTLN("// _apiServer.begin()");
  _server.begin();
  DEBUG_PRINTLN("// _apiServer configuration");
  _server.on("/api/v1/wheelly/restart", HTTP_METHOD_POST, handlePostRestart);
  _server.on("/api/v1/wheelly/networks/network", HTTP_METHOD_GET, handleGetWiFiConfig);
  _server.on("/api/v1/wheelly/networks/network", HTTP_METHOD_POST, handlePostWiFiConfig);
  _server.on("/api/v1/wheelly/networks", HTTP_METHOD_GET, handleGetNetworkList);
  _server.onNotFound(handleNotFound);
  DEBUG_PRINTLN("// Started api server.");
}

void ApiServerClass::polling(const unsigned long t0) {
  _server.handleClient();
  if (_restartInstant > 0 && t0 > _restartInstant) {
    esp_restart();
  }
}
