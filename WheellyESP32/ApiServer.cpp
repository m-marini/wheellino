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
#include <WiFi.h>

#include "ApiServer.h"
#include "ConfStore.h"

#include <esp_log.h>
static const char *TAG = "ApiServer";

#define HTTP_METHOD_GET ((HTTPMethod)1)
#define HTTP_METHOD_POST ((HTTPMethod)3)
#define RESTART_DELAY 500
#define SERVER_PORT 80

static const char *DEFAULT_SERVER_NAME = "wheelly";
static const unsigned long MAX_SCAN_DURATION = 1000000;

ApiServerClass ApiServer;

static WebServer _server(SERVER_PORT);
static void *_context;
static onActivity_cb_t _onActivity;
static const char *_wheellyId;
static unsigned long _restartInstant;

static void setActivity(void) {
  if (_onActivity) {
    _onActivity(_context, ApiServer);
  }
}

/*
  Send error
*/
static void sendError(const int httpCode, const char *msg) {
  StaticJsonDocument<256> jsonDoc;
  jsonDoc["message"] = msg;
  char response[256];
  serializeJson(jsonDoc, response, sizeof(response));
  _server.send(httpCode, "application/json", response);
  setActivity();
}

static const char *methodName(const HTTPMethod method) {
  char *result = "Unknown";
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
static void handleNotFound(void) {
  ESP_LOGD(TAG, "handleNotFound method %d", (int)ApiServer._server.method());
  StaticJsonDocument<256> jsonDoc;
  jsonDoc.clear();
  jsonDoc["message"] = "Not Found";
  jsonDoc["url"] = _server.uri();
  jsonDoc["method"] = methodName(_server.method());
  char response[256];
  serializeJson(jsonDoc, response, sizeof(response));
  ESP_LOGE(TAG, "404 - %s", response);
  _server.send(404, "application/json", response);
  setActivity();
}

/*
   Handles netwok list request
*/
static void handleGetNetworkList(void) {
  ESP_LOGD(TAG, "Scanning networks...");
  setActivity();
  StaticJsonDocument<256> jsonDoc;
  JsonArray data = jsonDoc.createNestedArray("networks");
  int scanResult = WiFi.scanNetworks(false, true);
  for (int i = 0; i < scanResult; i++) {
    data.add(WiFi.SSID(i));
  }
  char response[512];
  serializeJson(jsonDoc, response, sizeof(response));
  _server.send(200, "application/json", response);
}

/*
   Handles the configuration request
*/
static void handleGetConfig(void) {
  ESP_LOGD(TAG, "handleGetWiFiConfig");
  StaticJsonDocument<256> jsonDoc;
  ConfStore::toJson(jsonDoc, confStore.config());

  char response[512];
  serializeJson(jsonDoc, response, sizeof(response));
  _server.send(200, "application/json", response);
  setActivity();
}

/*
   Handles the configuration request
*/
static void handleGetWheellyId(void) {
  ESP_LOGD(TAG, "handleGetWheellyId");
  JsonDocument jsonDoc;
  jsonDoc["id"] = _wheellyId;

  char response[128];
  serializeJson(jsonDoc, response, sizeof(response));
  _server.send(200, "application/json", response);
  setActivity();
}

/*
   Handles the wifi configuration request
*/
static void handlePostConfig(void) {
  ESP_LOGD(TAG, "handlePostWiFiConfig");
  // Retrive body
  JsonDocument jsonDoc;
  if (deserializeJson(jsonDoc, _server.arg("plain")) != DeserializationError::Ok) {
    sendError(400, "Malformed JSON");
    return;
  }

  ConfigRecord config;
  const ConfStoreRetCode rc = ConfStore::fromJson(config, jsonDoc);
  char response[512];
  if (rc != SUCCESS) {
    snprintf(response, sizeof(response), "Bad configuration RC=%d", rc);
    sendError(400, response);
    return;
  }

  const ConfStoreRetCode rc1 = confStore.store(config);
  if (rc1 != SUCCESS) {
    snprintf(response, sizeof(response), "Bad configuration RC=%d", rc1);
    sendError(400, response);
    return;
  }

  serializeJson(jsonDoc, response, sizeof(response));
  ESP_LOGI(TAG, "WiFi configuration: %s", response);
  _server.send(200, "application/json", response);
  setActivity();
}

/*
   Handles the wifi configuration request
*/
static void handlePostRestart(void) {
  ESP_LOGD(TAG, "handleRestart");
  StaticJsonDocument<256> jsonDoc;
  jsonDoc.clear();
  jsonDoc["restart"] = true;

  char response[128];
  serializeJson(jsonDoc, response, sizeof(response));
  _server.send(200, "application/json", response);
  setActivity();
  _restartInstant = millis() + RESTART_DELAY;
}

void ApiServerClass::begin(const char *wheellyId) {
  ESP_LOGI(TAG, "Begin");
  _wheellyId = wheellyId;
}

void ApiServerClass::start(void) {
  ESP_LOGD(TAG, "Starting api server ...");

  if (!MDNS.begin(DEFAULT_SERVER_NAME)) {
    ESP_LOGD(TAG, "Error starting dns responder.");
  } else {
    ESP_LOGI(TAG, "Started MDNS");
  }
  ESP_LOGD(TAG, "_apiServer.begin()");
  _server.begin();
  ESP_LOGD(TAG, "_apiServer configuration");
  _server.on("/api/v2/wheelly/restart", HTTP_METHOD_POST, handlePostRestart);
  _server.on("/api/v2/wheelly/config", HTTP_METHOD_GET, handleGetConfig);
  _server.on("/api/v2/wheelly/config", HTTP_METHOD_POST, handlePostConfig);
  _server.on("/api/v2/wheelly/networks", HTTP_METHOD_GET, handleGetNetworkList);
  _server.on("/api/v2/wheelly/id", HTTP_METHOD_GET, handleGetWheellyId);
  _server.onNotFound(handleNotFound);
  ESP_LOGD(TAG, "Started api server.");
}

/*
       Sets callback on activity
    */
void ApiServerClass::onActivity(onActivity_cb_t callback, void *context) {
  _onActivity = callback;
  _context = context;
}

void ApiServerClass::polling(const unsigned long t0) {
  _server.handleClient();
  if (_restartInstant > 0 && t0 > _restartInstant) {
    esp_restart();
  }
}
