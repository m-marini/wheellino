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

#ifndef Wheelly_h
#define Wheelly_h

#include "Arduino.h"

#include "Contacts.h"
#include "mpu6050mm.h"
#include "MotionCtrl.h"
#include "Display.h"
#include "ProxySensor.h"

#define WHEELLY_VERSION "0.9.0"
#define WHEELLY_MESSAGES_VERSION "v0"

/*
   Wheelly controller.
   Handles all the behavior of wheelly robot:
   - commands from wifi modules
   - scanning of proxy sensors
   - contact sensors
   - mpu sensors
   - motor sensors
   - drives the motors
   - operational timing processes
*/
class Wheelly {
private:
  String _id;
  DisplayClass _display;
  MotionCtrlClass _motionCtrl;
  MPU6050Class _mpu;
  ContactSensors _contactSensors;
  ProxySensor _proxySensor;
  String _pubSensorTopicPrefix;
  String _subCommandTopics;

  Timer _ledTimer;
  Timer _statsTimer;

  boolean _onLine;
  int _yaw;
  unsigned long _mpuTimeout;
  uint8_t _mpuError;

  unsigned long _sendInterval;
  unsigned long _lastSend;

  boolean _ledActive;

  unsigned long _counter;
  unsigned long _statsTime;

  unsigned long _echoTime;
  unsigned long _echoDelay;
  int _echoDirection;
  int _echoYaw;
  float _echoXPulses;
  float _echoYPulses;

  unsigned long _supplyTimeout;
  unsigned long _supplySampleTimeout;
  unsigned long _supplyTime;
  int _supplyVoltage;
  long _supplyTotal;
  int _supplySamples;

  void (*_onReply)(void* context, const String& topic, const String& data);
  void* _context;

  const boolean canMoveForward(void) const;
  const boolean canMoveBackward(void) const;
  const int forwardBlockDistanceTime() const;

  void handleProxyData(void);
  void handleStats(void);
  void handleLed(const unsigned long n);
  void handleMpuData(void);
  void handleChangedContacts(void);

  const boolean handleScanCmd(const unsigned long time, const String& topic, const String& args);
  const boolean handleMoveCmd(const unsigned long time, const String& topic, const String& args);
  const boolean handleCiCmd(const unsigned long time, const String& topic, const String& args);
  const boolean handleCcCmd(const unsigned long time, const String& topic, const String& args);
  const boolean handleCsCmd(const unsigned long time, const String& topic, const String& args);
  const boolean handleTcsCmd(const unsigned long time, const String& topic, const String& args);
  const boolean handleFxCmd(const unsigned long time, const String& topic, const String& args);
  const boolean handleQcCmd(const unsigned long time, const String& topic, const String& args);

  /*
       Sends the status of wheelly
    */
  void sendMotion(const unsigned long t0);
  void sendProxy(void);
  void sendContacts(void);
  void sendSupply(void);
  void sampleSupply(void);
  /**
       Averages the supply measures
    */
  void averageSupply(void);

  /**
    * Sends the sensor data
    * @param the suffixTopic
    *  @param data the data reply
    */
  void sendSensorData(const String& topic, const String& data);

  /**
    * Sends the command data reply
    * @param the suffixTopic
    * @param data the data reply
    */
  void sendCommandReply(const String& topic, const String& data);

  /**
       Scans proximity to the direction

       @param angle the angle (DEG)
       @param t0 the scanning instant
    */
  void scan(const int angle, const unsigned long t0 = millis());

  /**
       Moves the robot to the direction at speed

       @param direction the direction (DEG)
       @param speed the speed (pps)
    */
  void move(const int direction, const int speed);

  /**
       Moves the robot to the direction at speed
    */
  void halt(void) {
    _motionCtrl.halt();
  }

  /**
       Queries and sends the status
    */
  void queryStatus(void);

  /**
       Configures motion controller
       @param params the motion controller parameters
    */
  void configMotionController(const int* params) {
    _motionCtrl.configController(params);
  }

  /**
       Configures motor sensors
       @param tau the decay value of motor sensors
    */
  void configMotorSensors(const int tau) {
    _motionCtrl.tau(tau);
  }

  /**
       Configures feedback motor controller
       @param p the feedback motor controller parameters
       @param left true for left motor controller
    */
  void configFeedbackMotorController(const long* p, const boolean left) {
    (left ? _motionCtrl.leftMotor() : _motionCtrl.rightMotor()).muConfig(p);
  }

  /**
       Configures tcs motor controller
       @param p the tcs motor controller parameters
       @param left true for left motor controller
    */
  void configTcsMotorController(const int* p, const boolean left) {
    (left ? _motionCtrl.leftMotor() : _motionCtrl.rightMotor()).tcsConfig(p);
  }

  /**
       Configures intervals [send interval, scan interval]
       @param p the right motor controller parameters
    */
  void configIntervals(const int* p);

  /**
       Returns the motion controller
    */
  MotionCtrlClass& motionCtrl(void) {
    return _motionCtrl;
  }

  /**
       Resets wheelly
    */
  void reset(void);

public:
  /**
       Creates wheelly controller
    */
  Wheelly();

  /**
    * Returns the wheelly id (mac address)
    */
  const String& id(void) const {
    return _id;
  }

  /**
    * Returns the command subscription topics
    */
  const String& subCommandTopics(void) const {
    return _subCommandTopics;
  }

  /**
       Initializes wheelly controller
       Return true if successfully initialized
    */
  boolean begin(void);

  /**
       Sets on reply call back
    */
  void onReply(void (*callback)(void* context, const String& topic, const String& data), void* context = NULL) {
    _onReply = callback;
    _context = context;
  }

  /**
       Pools wheelly controller
    */
  void polling(const unsigned long t0 = millis());

  /**
       Execute a command
       Returns true if command ok
       @param t0 the current time
       @param topic the command topic
       @param args the arguments
    */
  const boolean execute(const unsigned long t0, const String& topic, const String& args);

  /**
       Sets on line status
       @param onLine true if mqtt clinet connected
    */
  void onLine(const boolean onLine);

  /**
       Set connected state
       @param state true if connected
    */
  void connected(const boolean state) {
    _display.connected(state);
  }

  /**
       Sets activity
    */
  void activity(const unsigned long t0 = millis()) {
    _display.activity(t0);
  }

  /**
       Displays the message
       @param message the message
    */
  void display(const char* message) {
    _display.showWiFiInfo(message);
  }
};

#endif
