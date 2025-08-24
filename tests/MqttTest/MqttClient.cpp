#include "MqttClient.h"

//#define DEBUG
#include "debug.h"

MqttClient mqttClient;

static void callback(char* topic, byte* payload, unsigned int length) {
  char text[length + 1];
  for (int i = 0; i < length; i++) {
    text[i] = payload[i];
  }
  text[length] = '\0';
  mqttClient.dispatch(topic, text);
}

MqttClient::MqttClient()
  : _client(_wiFiClient) {}

void MqttClient::begin(const String& brokerHost, const int brokerPort, const String& clientId, const String& user, const String& password,
                       const String& subTopics, const unsigned long retryInterval) {
  // Initialises mqtt
  Serial.print("Initialized mqtt client ");
  Serial.print(brokerHost.c_str());
  Serial.print(":");
  Serial.print(brokerPort);
  Serial.println();
  _client.setServer(brokerHost.c_str(), brokerPort);
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
      Serial.print("Connecting client broker ");
      Serial.print( _user.c_str());
      Serial.print("@");
      Serial.print(_clientId.c_str());
      Serial.println();
      if (_client.connect(_clientId.c_str(), _user.c_str(), _password.c_str())) {
        Serial.println("Client broker connected");
        Serial.print("Subscribe to ");
        Serial.println(_subTopics);
        _client.subscribe(_subTopics.c_str());
      } else {
        Serial.print("Client broker connection failed state=");
        Serial.println(_client.state());
        _retryTimeout = clockTime + _retryInterval;
      }
    }
  }
}

void MqttClient::connect(void) {
  _init = true;
}

void MqttClient::send(const String& topic, const String& msg) {
  if (_client.connected()) {
    _client.publish(topic.c_str(), msg.c_str());
  }
}