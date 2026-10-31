/*
 * Copyright (c) 2026  Marco Marini, marco.marini@mmarini.org
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

#include <esp_log.h>
static const char* TAG = "CameraWebServer";

#include <WiFi.h>
#include "SD_MMC.h"

#include "SDConfStore.h"
#include "Camera.h"
#include "WebServer.h"
#include "MqttClient.h"

#define CONNECTION_TIMEOUT 10000
#define CONNECTION_INTERVAL 1000
#define RETRY_INTERVAL 3000
#define LOOP_INTERVAL 10
#define WATCHDOG_INTERVAL 30000

#define WHEELLY_MESSAGES_VERSION "v0"

SDConfStore confStore;

static const char* morseStart = "-.-.-";                    // <CA>
static const char* morseStarting = "..";                    // I
static const char* morseStarted = ".-.-.";                  // <AR>
static const char* morseError = "... --- .../... --- ...";  // SOS SOS

static char pubSensorTopic[64];
static char subCommandTopics[64];

static boolean mqttConnected = false;
static unsigned long watchdogTime;

static const uint64_t macId(void) {
  uint64_t mac = ESP.getEfuseMac();
  uint64_t mac1 = 0;
  mac1 |= (mac & 0xffff000000000000ul);
  mac1 |= (mac & 0x0000ff0000000000ul) >> 40;
  mac1 |= (mac & 0x000000ff00000000ul) >> 24;
  mac1 |= (mac & 0x00000000ff000000ul) >> 8;
  mac1 |= (mac & 0x0000000000ff0000ul) << 8;
  mac1 |= (mac & 0x000000000000ff00ul) << 24;
  mac1 |= (mac & 0x00000000000000fful) << 40;

  return mac1;
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  while (!Serial) {}
  delay(500);
  ESP_LOGI(TAG, "Start sequence.");

  ESP_LOGI(TAG, "Mounting MicroSD Card");
  if (!SD_MMC.begin()) {
    ESP_LOGE(TAG, "MicroSD Card Mount Failed");
  }
  confStore.begin();

  ESP_LOGI(TAG, "Remounting MicroSD Card");
  if (!SD_MMC.begin("/sdcard", true)) {
    ESP_LOGE(TAG, "MicroSD Card Mount Failed");
  }

  // Initializing camrera
  ESP_LOGI(TAG, "Camera initialisation.");
  if (cameraBegin() != ESP_OK) {
    ESP_LOGE(TAG, "Camera initialisation failed");
    delay(1000);
    ESP.restart();
  }
  ESP_LOGI(TAG, "Camera initialized.");
  // Flashing startup
  morse(morseStart);

  // Start Wifi
  const char* ssid = confStore.config().wifiSsid;
  ESP_LOGI(TAG, "Connecting WiFi %s ...", ssid);
  WiFi.begin(ssid, confStore.config().wifiPassword);
  WiFi.setSleep(false);

  // Wait for connection
  int ct = 0;
  unsigned long timeout = millis() + CONNECTION_TIMEOUT;
  while (WiFi.status() != WL_CONNECTED && millis() <= timeout) {
    vTaskDelay(CONNECTION_INTERVAL / portTICK_PERIOD_MS);
    morse(morseStarting);
    ESP_LOGI(TAG, "Connecting WiFi...");
  }
  if (WiFi.status() != WL_CONNECTED) {
    ESP_LOGE(TAG, "WiFi connection failed.");
    morse(morseError);
    ESP.restart();
  }
  ESP_LOGI(TAG, "WiFi connected");

  ESP_LOGI(TAG, "Starts web server");
  startWebServer();


  // Creates topic names
  const uint64_t id = macId();
  snprintf(pubSensorTopic, sizeof(pubSensorTopic), "sens/wheellycam/%" PRIx64 "/" WHEELLY_MESSAGES_VERSION "/hi", id);
  snprintf(subCommandTopics, sizeof(subCommandTopics), "cmd/wheellycam/%" PRIx64 "/" WHEELLY_MESSAGES_VERSION "/+", id);
  ESP_LOGI(TAG, "Subtopics = [%s]", subCommandTopics);

  // Initialize MQTT
  ESP_LOGI(TAG, "Initialise mqtt client");
  const SDConfigRecord& mqttConfig = confStore.config();
  mqttClient.begin(mqttConfig.mqttBrokerHost, mqttConfig.mqttBrokerPort,
                   "wheellycam", mqttConfig.mqttUser, mqttConfig.mqttPsw,
                   subCommandTopics, 3000);

  // Connect MQTT
  ESP_LOGI(TAG, "Mqtt client connecting %s:%d ...", mqttConfig.mqttBrokerHost, mqttConfig.mqttBrokerPort);
  mqttClient.connect();

  morse(morseStarted);
  ESP_LOGI(TAG, "Start sequence completed.");
}

void loop() {
  unsigned long t0 = millis();
  mqttClient.polling(t0);
  bool conn = mqttClient.connected();
  if (conn != mqttConnected) {
    mqttConnected = conn;
    if (conn) {
      ESP_LOGI(TAG, "Mqtt client connected");
      watchdogTime = t0;
    } else {
      ESP_LOGI(TAG, "Mqtt client disconnected");
    }
  }
  if (conn && t0 >= watchdogTime) {
    mqttClient.send(pubSensorTopic, webUrl());
    watchdogTime = t0 + WATCHDOG_INTERVAL;
  }
  vTaskDelay(LOOP_INTERVAL / portTICK_PERIOD_MS);
}
