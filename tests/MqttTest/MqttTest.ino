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

#define DEBUG
#include "debug.h"

#define SERIAL_BPS 115200

static const String MQTT_CLIENT_ID("wheelly");
static const String MQTT_USER("wheelly");
static const String MQTT_PASSWORD("wheelly");
static const String MQTT_PUB_TOPIC("/wheelly/sensors");
static const String MQTT_SUB_TOPIC("/wheelly/commands");
static const unsigned long RETRY_INTERVAL = 3000;
static const unsigned long SEND_INTERVAL = 500;

static WiFiModuleClass wiFiModule;
static unsigned long sendTimeout;

static ConfStore confStore;

static void onMessage(const String& message) {
  Serial.print("Received ");
  Serial.println(message);
}

void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.println();

  confStore.begin();
  const ConfigRecord config=confStore.config();

  // Initialises wifi
  Serial.println("Initializing WiFi.");
  wiFiModule.begin(config);
  wiFiModule.onChange(handleOnChange);
  wiFiModule.start();

  // Initialises mqtt
  Serial.println("Initializing mqtt client");
  mqttClient.begin(config.mqttBrokerHost, config.mqttBrokerPort, MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_PUB_TOPIC, MQTT_SUB_TOPIC, RETRY_INTERVAL);
  //mqttClient.begin(config.mqttBrokerHost, config.mqttBrokerPort, MQTT_CLIENT_ID, config.mqttUser, config.mqttPsw, MQTT_PUB_TOPIC, MQTT_SUB_TOPIC, RETRY_INTERVAL);
  mqttClient.onMessage(onMessage);
}

void loop() {
  const unsigned long now = millis();
  wiFiModule.polling(now);
  mqttClient.polling(now);
  if (now >= sendTimeout) {
    mqttClient.send(String("time ") + String(now));
    sendTimeout = now + SEND_INTERVAL;
  }
}

static void handleOnChange(void* context, WiFiModuleClass& module) {
  char bfr[256];
  if (module.connected()) {
    mqttClient.connect();
    strcpy(bfr, module.ssid().c_str());
    strcat(bfr, " - IP: ");
    strcat(bfr, module.ipAddress().toString().c_str());
  } else if (module.connecting()) {
    strcpy(bfr, module.ssid().c_str());
    strcat(bfr, " connecting...");
  } else {
    strcpy(bfr, "Disconnected");
  }
  Serial.print(bfr);
  Serial.println();
}
