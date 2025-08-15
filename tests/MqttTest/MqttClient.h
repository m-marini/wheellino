#include <PubSubClient.h>

#include <PubSubClient.h>

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

#ifndef MqttClient_h
#define MqttClient_h

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

/**
 * Send and receive text messages from a broker
 */
class MqttClient {
private:
  WiFiClient _wiFiClient;
  PubSubClient _client;
  String _clientId;
  String _user;
  String _password;
  String _subTopics;
  unsigned long _retryInterval;
  unsigned long _retryTimeout;
  bool _init;
  void (*_onMessage)(const String& topic, const String& message);

public:
  MqttClient();

  /**
  * Initializes client
  */
  void begin(const String& brokerHost, const int brokerPort, const String& clientId, const String& user, const String& password, const String& supTopics, const unsigned long retryInterval);

  /**
  * Connects the client
  */
  void connect(void);

  /**
  * Returns true if the client is connected
  */
  const bool connected() {
    return _client.connected();
  }

  /**
  * Polls for execution
  */
  void polling(const unsigned long clockTime = millis());

  /**
  * Sends a string to the outTopic
  */
  void send(const String& topic, const String& data);

  /**
  * Dispatches the message
  */
  void dispatch(const String& topic, const String& message) {
    if (_onMessage) {
      _onMessage(topic, message);
    }
  }

  /**
  * Sets the message callback
  */
  void onMessage(void (*callback)(const String& topic, const String& message)) {
    _onMessage = callback;
  }
};

/**
* the mqtt client Singleton
*/
extern MqttClient mqttClient;

#endif
