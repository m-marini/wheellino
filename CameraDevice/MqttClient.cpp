
/*
 * Copyright (c) 2025  Marco Marini, marco.marini@mmarini.org
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

#include "MqttClient.h"

static const char* TAG = "MqttClient";

MqttClient mqttClient;

static void callback(char* topic, byte* payload, unsigned int length) {
  char text[length + 1];
  for (int i = 0; i < length; i++) {
    text[i] = payload[i];
  }
  text[length] = '\0';
  ESP_LOGI(TAG, "Message at[%s] [%s]", topic, text);
  mqttClient.dispatch(topic, text);
}

MqttClient::MqttClient()
  : _client(_wiFiClient) {}

void MqttClient::begin(const char* brokerHost, const int brokerPort, const char* clientId, const char* user, const char* password,
                       const char* subTopics, const unsigned long retryInterval) {
  // Initialises mqtt
  ESP_LOGI(TAG, "Initialized mqtt client %s:%d", brokerHost, brokerPort);
  _client.setServer(brokerHost, brokerPort);
  _client.setCallback(callback);
  _clientId = clientId;
  _user = user;
  _password = password;
  _subTopics = subTopics;
  _retryInterval = retryInterval;
}

void MqttClient::polling(const unsigned long clockTime) {
  if (_init) {
    if (_client.connected()) {
      _client.loop();
    } else if (clockTime >= _retryTimeout) {
      ESP_LOGI(TAG, "Connecting client broker %s@%s", _user, _clientId);
      if (_client.connect(_clientId, _user, _password)) {
        ESP_LOGI(TAG, "Client broker connected");
        ESP_LOGI(TAG, "Subscribe to %s", _subTopics);
        _client.subscribe(_subTopics);
      } else {
        ESP_LOGE(TAG, "Client broker connection failed state=%d", _client.state());
        _retryTimeout = clockTime + _retryInterval;
      }
    }
  }
}

void MqttClient::connect(void) {
  _init = true;
}

void MqttClient::send(const char* topic, const char* msg) {
  if (_client.connected()) {
    _client.publish(topic, msg);
  }
}

void MqttClient::send(const char* topic, const byte* msg, const size_t len) {
  if (_client.connected()) {
    _client.publish(topic, msg, len);
  }
}