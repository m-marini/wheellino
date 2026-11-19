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

#include "Arduino.h"
#include "WiFiModule.h"
#include "MqttClient.h"
#include "ConfStore.h"

#include <esp_log.h>
static char* TAG = "MqttTest";

#define SERIAL_BPS 115200

static const char* MQTT_CLIENT_ID = "0123456789ac";
static const char* MQTT_USER = "wheelly";
static const char* MQTT_PASSWORD = "wheelly";
static const char* MQTT_PUB_TOPIC = "sens/test";
static const char* MQTT_SUB_TOPICS = "sens/test";
static const unsigned long RETRY_INTERVAL = 3000;
static const unsigned long SEND_INTERVAL = 500;
static unsigned long sendTimeout;

static void onMessage(const char* topics, byte* message, unsigned int length) {
  char text[length + 1];
  memcpy(text, message, length);
  text[length] = '\0';
  ESP_LOGI(TAG, "%s %s", topics, text);
}

void setup() {
  Serial.begin(SERIAL_BPS);
  while (!Serial) {
    delay(10);
  }

  confStore.begin();
  const ConfigRecord& config = confStore.config();

  // Initialises wifi
  ESP_LOGI(TAG, "Initializing WiFi.");
  wiFiModule.begin(config);
  wiFiModule.onChange(handleOnChange);
  wiFiModule.start();

  // Initialises mqtt
  ESP_LOGI(TAG, "Initializing mqtt client");
  mqttClient.begin(config.mqttBrokerHost, config.mqttBrokerPort, MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_SUB_TOPICS, RETRY_INTERVAL);
  mqttClient.onMessage(onMessage);
}

void loop() {
  const unsigned long now = millis();
  wiFiModule.polling(now);
  mqttClient.polling(now);
  if (now >= sendTimeout) {
    char text[80];
    snprintf(text, sizeof(text), "%ld", now);
    mqttClient.send(MQTT_PUB_TOPIC, text);
    ESP_LOGI(TAG, "%s %s", MQTT_PUB_TOPIC, text);
    sendTimeout = now + SEND_INTERVAL;
  }
}

static void handleOnChange(void* context) {
  char bfr[256];
  if (wiFiModule.connected()) {
    mqttClient.connect();
    snprintf(bfr, sizeof(bfr), "%s - IP: %s", wiFiModule.ssid(), wiFiModule.ipAddress().toString().c_str());
  } else if (wiFiModule.connecting()) {
    snprintf(bfr, sizeof(bfr), "%s connecting", wiFiModule.ssid());
  } else {
    strcpy(bfr, "Disconnected");
  }
  ESP_LOGI(TAG, "%s", bfr);
}
