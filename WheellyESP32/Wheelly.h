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

#include <Arduino.h>
#include <ArduinoJson.h>

#include "Contacts.h"
#include "mpu6050mm.h"
#include "MotionCtrl.h"
#include "Display.h"
#include "Lidar.h"
#include "LidarServo.h"
#include "Timer.h"

#define WHEELLY_VERSION "0.12.0"
#define WHEELLY_MESSAGES_VERSION "v2"

enum HeadStatus {
  FIX_DIRECTION = 0,
  FRONT_TRACKING = 1,
  REAR_TRACKING = 2,
};

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
  LidarServo _servo;
  Lidar _lidar;
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

  unsigned long _lidarTime;
  uint16_t _frontDistance;
  uint16_t _rearDistance;
  int _lidarYaw;
  int _lidarDirection;
  int _lidarTargetDirection;
  float _lidarXPulses;
  float _lidarYPulses;
  HeadStatus _headStatus;
  float _xHeadTarget;
  float _yHeadTarget;
  int _antiGimbalRadius;
  Timer _headTrackingTimer;
  Timer _headCmdTimer;

  unsigned long _supplyTimeout;
  unsigned long _supplySampleTimeout;
  unsigned long _supplyTime;
  int _supplyVoltage;
  long _supplyTotal;
  int _supplySamples;
  int _minHeadDir;
  int _maxHeadDir;

  void (*_onReply)(void* context, const String& topic, const String& data);
  void* _context;

  const bool canMoveForward(void) const;
  const bool canMoveBackward(void) const;

  void handleLidarRange(const uint16_t frontDistance, const uint16_t rearDistance);
  void handleStats(void);
  void handleLed(const unsigned long n);
  void handleHeadTimeout(void);
  void handleMpuData(void);
  void handleChangedContacts(void);

  /*
    Validates a configuration parameter and return true if not valid
  */
  const bool validateCfg(JsonDocument& cfg, const JsonDocument& doc, const String& key, const int minValue, const int maxValue, const String& topic);

  const bool handleScanCmd(const unsigned long time, const String& topic, const String& args);
  const bool handleHtCmd(const unsigned long time, const String& topic, const String& args);
  const bool handleRoCmd(const unsigned long time, const String& topic, const String& args);
  const bool handleMvCmd(const unsigned long time, const String& topic, const String& args);
  const bool handleQcCmd(const unsigned long time, const String& topic, const String& args);
  const bool handleCfCmd(const unsigned long time, const String& topic, const String& args);

  /*
    Returns the json configuration
  */
  JsonDocument& jsonConfig(JsonDocument& doc);

  /*
    Applies the json configuration
  */
  void applyJsonConfig(const JsonDocument& doc);

  /*
       Sends the status of wheelly
    */
  void sendMotion(const unsigned long t0);
  void sendLidar(void);
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
    Start tracking the head to the target point
  
    @param frontTrack true if front track otherwise rear track
    @param xTarget the x target coordinate (pulses)
    @param yTarget the y target coordinate (pulses)
    @param t0 the scanning instant
  */
  void headTrack(const boolean frontTrack, const float xTarget, const float yTarget, const unsigned long t0);

  /**
    Tracks the head toward the target
  */
  void trackingHead(void);

  /**
       Rotate the robot to the given direction

       @param direction the direction (DEG)
    */
  void rotate(const int direction);

  /*
   Moves forward the robot to the target position

   @param xTarget the x target (pulses)
   @param yTarget the y target (pulses)
  */
  void forward(const float xTarget, const float yTarget);

  /*
   Moves backward the robot to the target position

   @param xTarget the x target (pulses)
   @param yTarget the y target (pulses)
  */
  void backward(const float xTarget, const float yTarget);

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
  * Returns true if head is traking a target
  */
  const HeadStatus headStatus(void) const {
    return _headStatus;
  }

  /*
  * Returns the x head target (pulses)
  */
  const float xHeadTarget(void) const {
    return _xHeadTarget;
  }

  /*
  * Returns the y head target (pulses)
  */
  const float yHeadTarget(void) const {
    return _yHeadTarget;
  }

  /**
  * Returns the head anti gimbal radius (pulses)
  */
  const int antiGimbalRadius(void) const {
    return _antiGimbalRadius;
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
