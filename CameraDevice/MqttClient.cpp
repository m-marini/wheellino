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