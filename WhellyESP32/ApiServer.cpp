#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>

//#define DEBUG
#include "debug.h"

#include "ApiServer.h"

static const char* DEFAULT_SERVER_NAME = "wheelly";
static const unsigned long MAX_SCAN_DURATION = 1000000;

ApiServerClass ApiServer;

/*
   Callback netwok list request
*/
static void getNetworkListCallback(AsyncWebServerRequest *request) {
  ApiServer.handleGetNetworkList(*request);
}

/*
  Callback not found page
*/
static void notFoundCallback(AsyncWebServerRequest *request) {
  ApiServer.handleNotFound(*request);
}

/*
   Callback for the wifi configuration request
*/
static void getWifiConfigCallback(AsyncWebServerRequest *request) {
  ApiServer.handleGetWiFiConfig(*request);
}

/*
   Callback for post wifi connfiguration request
*/
static void postWifiConfigCallback(AsyncWebServerRequest *request, JsonVariant& json) {
  ApiServer.handlePostWiFiConfig(*request, json);
}

/*
   Loads the network list
*/
void ApiServerClass::polling(const unsigned long) {
  if (_networkListRequest) {
    AsyncWebServerRequest& request = *_networkListRequest;
    _networkListRequest = NULL;
    StaticJsonDocument<256> jsonDoc;
    JsonArray data = jsonDoc.createNestedArray("networks");
    int scanResult = WiFi.scanNetworks(false, true);
    for (int i = 0; i < scanResult; i++) {
      data.add(WiFi.SSID(i));
    }
    AsyncResponseStream *response = request.beginResponseStream("application/json");
    serializeJson(jsonDoc, *response);
    request.send(response);
    setActivity();
  }
}

/*
   Handles netwok list request
*/
void ApiServerClass::handleGetNetworkList(AsyncWebServerRequest &request) {
  DEBUG_PRINTLN("// Scanning networks...");
  if (_networkListRequest) {
    sendError(request, HTTP_CODE_TOO_MANY_REQUESTS, "Server is processing other request");
  } else {
    // Set rx timeout
    //request.client()->setRxTimeout(MAX_SCAN_DURATION);
    _networkListRequest = &request;
    setActivity();
  }
}

/*
   Handles the wifi configuration request
*/
void ApiServerClass::handleGetWiFiConfig(AsyncWebServerRequest & request) {
  DEBUG_PRINTLN("// handlePostWiFiConfig");
  WiFiModuleClass* wiFiModule = _wiFiModule;
  if (!wiFiModule) {
    sendError(request, HTTP_CODE_NOT_FOUND, "WiFi configuration not available");
    return;
  }
  const WiFiConfig& wifiData = wiFiModule->config();
  StaticJsonDocument<256> jsonDoc;
  jsonDoc.clear();
  jsonDoc["version"] = CURRENT_VERSION;
  jsonDoc["active"] = wifiData.active;
  jsonDoc["ssid"] = wifiData.ssid;
  jsonDoc["password"] = wifiData.password;

  AsyncResponseStream *response = request.beginResponseStream("application/json");
  serializeJson(jsonDoc, *response);
  request.send(response);
  setActivity();
}

/*
   Handle post wifi connfiguration request
*/
void ApiServerClass::handlePostWiFiConfig(AsyncWebServerRequest & request, JsonVariant & jsonDoc) {
  WiFiModuleClass* wiFiModule = _wiFiModule;
  if (wiFiModule) {
    sendError(request, HTTP_CODE_NOT_FOUND, "WiFi configuration not available");
    return;
  }
  if (!jsonDoc.containsKey("active")
      || !jsonDoc.containsKey("ssid")
      || !jsonDoc.containsKey("password")) {
    sendError(request, HTTP_CODE_BAD_REQUEST, "Missing parameters");
    return;
  }
  WiFiConfig config;
  config.version = CURRENT_VERSION;
  config.active = jsonDoc["active"].as<bool>();
  strncpy(config.ssid, jsonDoc["ssid"].as<const char*>(), sizeof(config.ssid) - 1);
  strncpy(config.password, jsonDoc["password"].as<const char*>(), sizeof(config.password) - 1);

  AsyncResponseStream *response = request.beginResponseStream("application/json");
  serializeJson(jsonDoc, *response);
  request.send(response);
  wiFiModule->setConfig(config);
  setActivity();
}

/*
  Handles not found page
*/
void ApiServerClass::handleNotFound(AsyncWebServerRequest & request) {
  StaticJsonDocument<256> jsonDoc;
  jsonDoc.clear();
  jsonDoc["message"] = "Not Found";
  jsonDoc["url"] = request.url();
  jsonDoc["method"] = methodName(request);

  AsyncResponseStream *response = request.beginResponseStream("application/json");
  response->setCode(HTTP_CODE_NOT_FOUND);
  serializeJson(jsonDoc, *response);
  request.send(response);
  setActivity();
}

/*
  Returns the string of method name
*/
const String ApiServerClass::methodName(AsyncWebServerRequest & request) {
  return request.methodToString();
}

/*
  Send error
*/
void ApiServerClass::sendError(AsyncWebServerRequest & request, const t_http_codes httpCode, const String & msg) {
  StaticJsonDocument<256> jsonDoc;
  jsonDoc["message"] = msg;

  AsyncResponseStream *response = request.beginResponseStream("application/json");
  response->setCode(httpCode);
  serializeJson(jsonDoc, *response);
  request.send(response);
  setActivity();
}

/*
   Begins the api server
*/
void ApiServerClass::begin(void) {
  DEBUG_PRINTLN("// Starting api server ...");

  if (!MDNS.begin(DEFAULT_SERVER_NAME)) {
    Serial.println("!! Error starting dns responder.");
  } else {
    DEBUG_PRINTLN("// Started MDNS");
  }
  _apiServer.begin();
  _apiServer.on("/api/v1/wheelly/networks/network", HTTP_GET, getWifiConfigCallback);
  //  _apiServer.on("/api/v1/wheelly/networks/network", HTTP_POST, postWifiConfigCallback);
  _apiServer.on("/api/v1/wheelly/networks", HTTP_GET, getNetworkListCallback);
  _apiServer.onNotFound(notFoundCallback);
  AsyncCallbackJsonWebHandler* handler = new AsyncCallbackJsonWebHandler("/api/v1/wheelly/networks/network", postWifiConfigCallback);
  _apiServer.addHandler(handler);
  DEBUG_PRINTLN("// Started api server.");
}

/*
   Callback for the activity
*/
void ApiServerClass::setActivity(void) {
  if (_onActivity) {
    _onActivity(_context, *this);
  }
}
