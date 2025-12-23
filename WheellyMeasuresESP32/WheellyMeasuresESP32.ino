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

/*
   Measure the motor speed.

   Sending command 'start' the robot will move randomly
   and after the test interval (1s) will print the selected motor powers
   and the motor speed measured (pps).
*/

#include <esp_log.h>
static const char* TAG = "WheellyMeasuresESP32";

#include "pins.h"

#include "WiFiModule.h"
#include "Timer.h"
#include "MotorCtrl.h"
#include "Contacts.h"
#include "Display.h"
#include "Tests.h"
#include "MqttClient.h"

/*
   Serial config
*/
#define SERIAL_BPS 115200
#define MAX_POWER 255

#define WHEELLY_VERSION "0.10.0"
#define WHEELLY_MESSAGES_VERSION "v0"

static const unsigned WIRE_CLOCK = 400000;

#define inrange(value, min, max) ((value) >= (min) && (value) <= (max))

/*
   Serial line buffer
*/
static const unsigned long SERIAL_TIMEOUT = 2000ul;

/*
   WiFi module
*/
static WiFiModuleClass wiFiModule;

/*
   Motor sensors
*/
MotorSensor leftSensor(LEFT_PIN);
MotorSensor rightSensor(RIGHT_PIN);

/*
   Motor controllers
*/
MotorCtrl leftMotor(LEFT_FORW_PIN, LEFT_BACK_PIN, leftSensor);
MotorCtrl rightMotor(RIGHT_FORW_PIN, RIGHT_BACK_PIN, rightSensor);

/*
   Contact sensors
*/
ContactSensors contacts(FRONT_CONTACTS_PIN, REAR_CONTACTS_PIN);

/**
* Configuration store
*/
static ConfStore confStore;

/**
   LCD
*/
static DisplayClass Display;

/*
  Mqqt parameters
*/
static const unsigned long MQTT_RETRY_INTERVAL = 3000ul;
static boolean mqttConnected;
static String deviceId;
static String pubSensorTopic;
static String respCommandTopic;
static String errCommandTopic;
static String subCommandTopics;

static const unsigned long STATUS_INTERVAL = 500ul;

/*
Test variables
*/
static unsigned long testStartInstant;
static MotorTest leftMotorTest(leftMotor);
static MotorTest rightMotorTest(rightMotor);
static unsigned long leftPulses;
static unsigned long rightPulses;
static bool frontSensor;
static bool rearSensor;
static unsigned int frontDistance;
static unsigned int rearDistance;

static Timer statusTimer;

/**
Returns true if test is running
*/
static bool isTesting(void) {
  return leftMotorTest.testing() || leftMotorTest.testing();
}

/**
Sends the report to queue
*/
static void sendReport() {
  // Handles reply to remote controller
  if (mqttClient.connected()) {
    const unsigned long t0 = millis();
    char status[256];
    const bool testing = isTesting();
    sprintf(status, "%lu,%d,%d,%d,%ld,%ld,%d,%d,%u,%u",
            testing ? t0 - testStartInstant : 0l,
            testing,
            leftMotorTest.power(), rightMotorTest.power(),
            leftPulses, rightPulses,
            frontSensor, rearSensor,
            frontDistance, rearDistance);
    mqttClient.send(pubSensorTopic, status);
  }
}

/*
Handles status timer
*/
static void handleStatus(void*, const unsigned long) {
  if (!isTesting()) {
    sendReport();
  }
}

/*
Sends command reply
*/
static void sendCommandReply(const String& topic, const String& message) {
  if (mqttClient.connected()) {
    mqttClient.send(topic, message);
  }
}

/*
Handles mqtt message
*/
static void handleMqttMessage(const String& topic, const String& message) {
  ESP_LOGI(TAG, "Message %s %s", topic.c_str(), message.c_str());
  if (topic.endsWith("/test")) {
    startTest(message);
  }
}

static const String validateArguments(const int maxPower, const unsigned long stepUpInterval, const int stepUpPower,
                                      const unsigned long stepDownInterval, const int stepDownPower) {
  if (maxPower == 0 || abs(maxPower) > MAX_POWER) {
    return "Invalid power";
  }
  if (stepUpInterval == 0 || stepDownInterval == 0) {
    return "Invalid interval";
  }
  if ((maxPower > 0 && stepUpPower <= 0)
      || (maxPower < 0 && stepUpPower >= 0)) {
    return "Invalid step up power";
  }
  if ((maxPower > 0 && stepDownPower >= 0)
      || (maxPower < 0 && stepDownPower <= 0)) {
    return "Invalid step down power";
  }
  return "";
}

/*
 Starts the test
*/
static bool startTest(const String& args) {
  if (isTesting()) {
    ESP_LOGE(TAG, "Wrong args %s", args.c_str());
    sendCommandReply(errCommandTopic, "Test is already running");
    return false;
  }
  int count;
  int leftMaxPower;
  unsigned long leftStepUpInterval;
  int leftStepUpPower;
  unsigned long leftStepDownInterval;
  int leftStepDownPower;
  int rightMaxPower;
  unsigned long rightStepUpInterval;
  int rightStepUpPower;
  unsigned long rightStepDownInterval;
  int rightStepDownPower;
  int n = sscanf(args.c_str(), "%d,%lu,%d,%lu,%d,%d,%lu,%d,%lu,%d%n",
                 &leftMaxPower,
                 &leftStepUpInterval,
                 &leftStepUpPower,
                 &leftStepDownInterval,
                 &leftStepDownPower,
                 &rightMaxPower,
                 &rightStepUpInterval,
                 &rightStepUpPower,
                 &rightStepDownInterval,
                 &rightStepDownPower,
                 &count);
  if (n != 10 || count != args.length()) {
    ESP_LOGE(TAG, "Wrong args n=%d count=%d [%s]", n, count, args.c_str());
    sendCommandReply(errCommandTopic, "Wrong args " + args);
    return false;
  }

  // Valudate arguments
  String argError = validateArguments(leftMaxPower, leftStepUpInterval, leftStepUpPower, leftStepDownInterval, leftStepDownPower);
  if (argError != "") {
    ESP_LOGE(TAG, "%s [%s]", argError.c_str(), args.c_str());
    sendCommandReply(errCommandTopic, argError + " " + args);
    return false;
  }
  argError = validateArguments(rightMaxPower, rightStepUpInterval, rightStepUpPower, rightStepDownInterval, rightStepDownPower);
  if (argError != "") {
    ESP_LOGE(TAG, "%s [%s]", argError.c_str(), args.c_str());
    sendCommandReply(errCommandTopic, argError + " " + args);
    return false;
  }

  testStartInstant = millis();
  leftPulses = rightPulses = 0;
  leftMotorTest.start(testStartInstant, leftMaxPower,
                      leftStepUpInterval, leftStepUpPower,
                      leftStepDownInterval, leftStepDownPower);
  rightMotorTest.start(testStartInstant, rightMaxPower,
                       rightStepUpInterval, rightStepUpPower,
                       rightStepDownInterval, rightStepDownPower);

  sendCommandReply(respCommandTopic, args);
  return true;
}

static void handlePowerChange(void*) {
  sendReport();
}

/*
   Set up
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

  Display.begin();
  Display.clear();

  ESP_LOGI(TAG, "Initialise confStore");
  confStore.begin();

  ESP_LOGI(TAG, "Initialise wifi module");
  const ConfigRecord& config = confStore.config();
  wiFiModule.begin(config);

  // Initializes the device id and the queue names
  uint64_t mac = ESP.getEfuseMac();
  uint64_t mac1 = 0;
  mac1 |= (mac & 0xffff000000000000ul);
  mac1 |= (mac & 0x0000ff0000000000ul) >> 40;
  mac1 |= (mac & 0x000000ff00000000ul) >> 24;
  mac1 |= (mac & 0x00000000ff000000ul) >> 8;
  mac1 |= (mac & 0x0000000000ff0000ul) << 8;
  mac1 |= (mac & 0x000000000000ff00ul) << 24;
  mac1 |= (mac & 0x00000000000000fful) << 40;

  deviceId = String(mac1, HEX);
  pubSensorTopic = "sens/wheelly/" + deviceId + "/" + WHEELLY_MESSAGES_VERSION + "/test";
  respCommandTopic = "cmd/wheelly/" + deviceId + "/" + WHEELLY_MESSAGES_VERSION + "/test/resp";
  errCommandTopic = "cmd/wheelly/" + deviceId + "/" + WHEELLY_MESSAGES_VERSION + "/test/err";
  subCommandTopics = "cmd/wheelly/" + deviceId + "/" + WHEELLY_MESSAGES_VERSION + "/+";

  ESP_LOGI(TAG, "Initialise mqtt client");
  mqttClient.begin(config.mqttBrokerHost, config.mqttBrokerPort, deviceId, config.mqttUser, config.mqttPsw,
                   subCommandTopics, MQTT_RETRY_INTERVAL);

  mqttClient.onMessage(handleMqttMessage);

  ESP_LOGI(TAG, "Start wifi module");
  wiFiModule.onChange(handleOnChange);
  wiFiModule.start();
  ESP_LOGI(TAG, "Startup sequence completed");

  leftSensor.onSample(handleLeftSensor);
  rightSensor.onSample(handleRightSensor);

  leftMotor.automatic(false);
  leftMotor.begin();

  rightMotor.automatic(false);
  rightMotor.begin();

  contacts.onChanged(handleContacts);
  contacts.begin();

  ESP_LOGI(TAG, "Starting status timer");
  statusTimer.interval(STATUS_INTERVAL);
  statusTimer.continuous(true);
  statusTimer.onNext(handleStatus);
  statusTimer.start();

  leftMotorTest.onPowerChange(handlePowerChange);
  rightMotorTest.onPowerChange(handlePowerChange);

  ESP_LOGI(TAG, "Wheelly measures started device=%s", deviceId.c_str());
  Display.clear();
  Display.print("Wheelly measures started.");
}

/*
   Main loop
*/
void loop() {
  unsigned long t0 = millis();
  Display.polling(t0);
  wiFiModule.polling(t0);
  contacts.polling(t0);
  leftMotor.polling(t0);
  rightMotor.polling(t0);
  mqttClient.polling(t0);
  if (mqttConnected != mqttClient.connected()) {
    mqttConnected = mqttClient.connected();
  }
  leftMotorTest.pooling(t0);
  rightMotorTest.pooling(t0);
  statusTimer.polling(t0);
}

/*
   Handles the wifi module state change
*/
static void handleOnChange(void*, WiFiModuleClass& module) {
  char bfr[256];
  boolean connected = module.connected();
  if (connected) {
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
  ESP_LOGI(TAG, "%s", bfr);
  Display.clear();
  Display.showWiFiInfo(bfr);
}

static void handleLeftSensor(void*, const int dPulse, const unsigned long clockTime, MotorSensor& sensor) {
  leftPulses += dPulse;
  sendReport();
}

static void handleRightSensor(void*, const int dPulse, const unsigned long clockTime, MotorSensor& sensor) {
  rightPulses += dPulse;
  sendReport();
}

static void handleContacts(void*, ContactSensors& sensors) {
  frontSensor = sensors.frontClear();
  rearSensor = sensors.rearClear();
  leftMotorTest.stop();
  rightMotorTest.stop();
  sendReport();
}
