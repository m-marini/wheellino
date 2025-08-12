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
  String data(text);
  mqttClient.dispatch(data);
}

MqttClient::MqttClient()
  : _client(_wiFiClient) {}

void MqttClient::begin(const String& brokerHost, const int brokerPort, const String& clientId, const String& user, const String& password,
                       const String& pubTopic, const String& subTopic, const unsigned long retryInterval) {
  // Initialises mqtt
  Serial.println("Initializing mqtt client ...");
  _client.setServer(brokerHost.c_str(), brokerPort);
  _client.setCallback(callback);
  _clientId = clientId;
  _user = user;
  _password = password;
  _pubTopic = pubTopic;
  _subTopic = subTopic;
  _retryInterval = retryInterval;
}

void MqttClient::polling(const unsigned long clockTime) {
  if (_init) {
    if (_client.connected()) {
      _client.loop();
    } else if (clockTime >= _retryTimeout) {
      Serial.println("Connecting client broker");
      if (_client.connect(_clientId.c_str(), _user.c_str(), _password.c_str())) {
        Serial.println("Client broker connected");
        Serial.print("Subscribe to ");
        Serial.println(_subTopic);
        _client.subscribe(_subTopic.c_str());
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

void MqttClient::send(const String& msg) {
  if (_client.connected()) {
    _client.publish(_pubTopic.c_str(), msg.c_str());
  }
}