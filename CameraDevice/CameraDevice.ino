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
#include <pb.h>

#include "SD_MMC.h"

#include "SDConfStore.h"
#include "Camera.h"
#include "WebServer.h"
#include "MqttClient.h"

#include "src/camera.pb.h"

#define CONNECTION_TIMEOUT 10000
#define CONNECTION_INTERVAL 1000
#define RETRY_INTERVAL 3000
#define LOOP_INTERVAL 10
#define HEARTBEAT_INTERVAL 1000

#define WHEELLY_MESSAGES_VERSION "v1"

SDConfStore confStore;

static const char* morseStart = "-.-.-";                    // <CA>
static const char* morseStarting = "..";                    // I
static const char* morseStarted = ".-.-.";                  // <AR>
static const char* morseError = "... --- .../... --- ...";  // SOS SOS

static char mqttClientId[16];
static char pubSensorTopic[64];
static char subCommandTopics[64];

static boolean mqttConnected = false;
static unsigned long heartbeatTime;

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

static void sendHiMessage() {
  //mqttClient.send(pubSensorTopic, webUrl());
  wheelly_protobuf_webcam_CameraHiMessage protoMsg = wheelly_protobuf_webcam_CameraHiMessage_init_default;
  snprintf(protoMsg.url, sizeof(protoMsg.url), "%s", webUrl());

  uint8_t buffer[128];  // Buffer per i byte serializzati
  pb_ostream_t streamOut = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Codifica il messaggio
  bool status = pb_encode(&streamOut, wheelly_protobuf_webcam_CameraHiMessage_fields, &protoMsg);
  if (!status) {
    ESP_LOGE(TAG, "Errore di codifica: %s", PB_GET_ERROR(&streamOut));
    return;
  }

  size_t len = streamOut.bytes_written;
  mqttClient.send(pubSensorTopic, buffer, len);
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
  snprintf(mqttClientId, sizeof(mqttClientId), "%" PRIx64, id);
  snprintf(pubSensorTopic, sizeof(pubSensorTopic), "sens/wheellycam/%s/" WHEELLY_MESSAGES_VERSION "/hi", mqttClientId);
  snprintf(subCommandTopics, sizeof(subCommandTopics), "cmd/wheellycam/%s/" WHEELLY_MESSAGES_VERSION "/+", mqttClientId);
  ESP_LOGI(TAG, "Subtopics = [%s]", subCommandTopics);

  // Initialize MQTT
  ESP_LOGI(TAG, "Initialise mqtt client");
  const SDConfigRecord& mqttConfig = confStore.config();
  mqttClient.begin(mqttConfig.mqttBrokerHost, mqttConfig.mqttBrokerPort,
                   mqttClientId, mqttConfig.mqttUser, mqttConfig.mqttPsw,
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
      heartbeatTime = t0;
    } else {
      ESP_LOGI(TAG, "Mqtt client disconnected");
    }
  }
  if (conn && t0 >= heartbeatTime) {
    sendHiMessage();
    heartbeatTime = t0 + HEARTBEAT_INTERVAL;
  }
  vTaskDelay(LOOP_INTERVAL / portTICK_PERIOD_MS);
}
