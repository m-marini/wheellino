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

#include <stdio.h>
#include <Arduino.h>
#include <Wire.h>

#include <esp_log.h>
static const char *TAG = "WheellyESP32";

#include "Wheelly.h"
#include "WiFiModule.h"
#include "ApiServer.h"
#include "MqttClient.h"

static const unsigned WIRE_CLOCK = 400000;

/*
   Serial line buffer
*/
static const unsigned long SERIAL_TIMEOUT = 2000ul;
static char line[100];

/*
  Mqqt parameters
*/
static const unsigned long MQTT_RETRY_INTERVAL = 3000ul;
static boolean mqttConnected;

/*
   WiFi module
*/
static WiFiModuleClass wiFiModule;

/*
   Wheelly controller
*/
static Wheelly wheelly;

/**
* Configuration store
*/
static ConfStore confStore;

/*
   Initial setup
*/
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(SERIAL_TIMEOUT);

  Wire.begin();
  Wire.setClock(WIRE_CLOCK);  // 400kHz I2C clock. Comment this line if having compilation difficulties

  // Initialises the configuration store (load the configuration)
  delay(100);
  Serial.println();
  ESP_LOGI(TAG, "Startup sequence");

  ESP_LOGI(TAG, "Initialise confStore");
  confStore.begin();

  ESP_LOGI(TAG, "Initialise wifi module");
  const ConfigRecord &config = confStore.config();
  wiFiModule.begin(config);

  ESP_LOGI(TAG, "Initialise mqtt client");
  mqttClient.begin(config.mqttBrokerHost, config.mqttBrokerPort, wheelly.id(), config.mqttUser, config.mqttPsw,
                   wheelly.subCommandTopics(), MQTT_RETRY_INTERVAL);
  mqttClient.onMessage(handleMqttMessage);

  ESP_LOGI(TAG, "Initialise whelly");
  wheelly.begin();
  wheelly.onReply([](void *, const String &topic, const String &data) {
    // Handles reply to remote controller
    if (mqttClient.connected()) {
      mqttClient.send(topic, data);
      wheelly.activity();
    }
  });
  ESP_LOGI(TAG, "WheellyId=", wheelly.id().c_str());

  ESP_LOGI(TAG, "Initialise api server");
  ApiServer.begin(wheelly.id(), confStore);
  ApiServer.onActivity([](void *, ApiServerClass &) {
    wheelly.activity();
  });

  ESP_LOGI(TAG, "Start wifi module");
  wiFiModule.onChange(handleOnChange);
  wiFiModule.start();
  ESP_LOGI(TAG, "Startup sequence completed");
}

/*
   The loop function
*/
void loop() {
  const unsigned long now = millis();

  wiFiModule.polling(now);
  ApiServer.polling(now);
  wheelly.polling(now);
  mqttClient.polling(now);
  if (mqttConnected != mqttClient.connected()) {
    mqttConnected = mqttClient.connected();
    wheelly.onLine(mqttConnected);
  }
}

/*
   Handles the wifi module state change
*/
static void handleOnChange(void *, WiFiModuleClass &module) {
  char bfr[256];
  boolean connected = module.connected();
  if (connected) {
    ApiServer.start();
    mqttClient.connect();
    strcpy(bfr, wiFiModule.ssid().c_str());
    strcat(bfr, " - IP: ");
    strcat(bfr, wiFiModule.ipAddress().toString().c_str());
  } else if (module.connecting()) {
    strcpy(bfr, wiFiModule.ssid().c_str());
    strcat(bfr, " connecting...");
  } else {
    strcpy(bfr, "Disconnected");
  }
  wheelly.display(bfr);
}

static void handleMqttMessage(const String &topic, const String &message) {
  wheelly.execute(millis(), topic, message);
}

static void handleMqttConnected(void) {
  wheelly.onLine(true);
}
