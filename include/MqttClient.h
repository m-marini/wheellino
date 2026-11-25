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

typedef void (*onMessage_cb_t)(const char *topic, byte *message, unsigned int length);

/**
 * Send and receive text messages from a broker
 */
class MqttClient
{
private:
  WiFiClient _wiFiClient;
  PubSubClient _client;
  const char *_clientId;
  const char *_user;
  const char *_password;
  const char *_subTopics;
  unsigned long _retryInterval;
  unsigned long _retryTimeout;
  bool _init;

public:
  MqttClient();

  /**
   * Initializes client
   */
  void begin(const char *brokerHost, const int brokerPort, const char *clientId, const char *user, const char *password, const char *supTopics, const unsigned long retryInterval);

  /**
   * Connects the client
   */
  void connect(void);

  /**
   * Returns true if the client is connected
   */
  bool connected()
  {
    return _client.connected();
  }

  /**
   * Polls for execution
   */
  void polling(const unsigned long clockTime = millis());

  /**
   * Sends a string to the outTopic
   */
  void send(const char *topic, const char *msg)
  {
    send(topic, (uint8_t *)msg, strlen(msg));
  }

  /**
   * Sends a byte array to the outTopic
   */
  void send(const char *topic, const uint8_t *msg, const size_t len);

  /**
   * Sets the message callback
   */
  void onMessage(onMessage_cb_t callback);
};

/**
 * the mqtt client Singleton
 */
extern MqttClient mqttClient;

#endif
