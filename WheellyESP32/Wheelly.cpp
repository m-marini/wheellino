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

#include "Wheelly.h"
#include "pins.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <esp_log.h>
static const char* TAG = "Wheelly";

/*
   Send Interval
*/
#define DEFAULT_SEND_INTERVAL 500ul

/*
   Proxy sensor servo
*/
#define DEFAULT_SCAN_INTERVAL 1000ul

/*
   Proxy sensor servo
*/
#define DEFAULT_ANTI_GIMBAL_RADIUS 20

/*
   Current version
*/
static const char version[] = WHEELLY_VERSION;

/*
   The gyroscope
*/
static const uint8_t MPU_TIMEOUT_ERROR = 8;
static const unsigned long MPU_INTERVAL = 1000ul;

/*
   Status led
*/
static const unsigned long FAST_LED_INTERVAL = 100;
static const unsigned long SLOW_LED_INTERVAL = 300;

/*
   Statistics
*/
static const unsigned long STATS_INTERVAL = 10000ul;

/*
   Proximity sensor
   Proximity distance scanner
*/
static const uint16_t STOP_DISTANCE = 200;  // 200 mm


/*
   Proxy sensor servo
*/
static const int NO_SCAN_DIRECTIONS = 10;
static const int SERVO_OFFSET = 0;
static const int SERVO_DIRECTION_LIMIT = 90;
static const unsigned long HEAD_COMMAND_TIMEOUT = 3000ul;
static const unsigned long HEAD_TRACKING_INTERVAL = 100ul;
static const int HEAD_DISTANCE = 6;  // 30mm = 5.7 pulses

/*
   Voltage levels
*/
static const unsigned long SUPPLY_SAMPLE_INTERVAL = 100;
static const unsigned long SUPPLY_INTERVAL = 5000;
static const int SAMPLE_BATCH = 5;
static const int MIN_VOLTAGE_VALUE = 1896;
static const int MAX_VOLTAGE_VALUE = 2621;

/*
   Motion
*/
static const int MAX_SPEED = 60;

/*
   Creates wheelly controller
*/
Wheelly::Wheelly()
  : _motionCtrl(LEFT_FORW_PIN, LEFT_BACK_PIN, RIGHT_FORW_PIN, RIGHT_BACK_PIN, LEFT_PIN, RIGHT_PIN),
    _sendInterval(DEFAULT_SEND_INTERVAL),
    _contactSensors(FRONT_CONTACTS_PIN, REAR_CONTACTS_PIN),
    _lidar(FRONT_LIDAR_PIN, REAR_LIDAR_PIN),
    _minHeadDir(-SERVO_DIRECTION_LIMIT),
    _maxHeadDir(SERVO_DIRECTION_LIMIT),
    _servo(SERVO_PIN),
    _antiGimbalRadius(DEFAULT_ANTI_GIMBAL_RADIUS) {
  // Computes device id from mac address
  uint64_t mac = ESP.getEfuseMac();
  uint64_t mac1 = 0;
  mac1 |= (mac & 0xffff000000000000ul);
  mac1 |= (mac & 0x0000ff0000000000ul) >> 40;
  mac1 |= (mac & 0x000000ff00000000ul) >> 24;
  mac1 |= (mac & 0x00000000ff000000ul) >> 8;
  mac1 |= (mac & 0x0000000000ff0000ul) << 8;
  mac1 |= (mac & 0x000000000000ff00ul) << 24;
  mac1 |= (mac & 0x00000000000000fful) << 40;

  _id = String(mac1, HEX);
  _pubSensorTopicPrefix = "sens/wheelly/" + _id + "/" + WHEELLY_MESSAGES_VERSION;
  _subCommandTopics = "cmd/wheelly/" + _id + "/" + WHEELLY_MESSAGES_VERSION + "/+";
}

/*
   Initializes wheelly controller
   Returns true if successfully initialized
*/
boolean Wheelly::begin(void) {
  ESP_LOGI(TAG, "Begin");
  /* Setup display */
  _display.begin();

  /* Setup error led */
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, true);
  _ledTimer.onNext([](void* context, const unsigned long n) {
    ((Wheelly*)context)->handleLed(n);
  },
                   this);
  _ledTimer.interval(SLOW_LED_INTERVAL);
  _ledTimer.continuous(true);
  _ledTimer.start();

  /* Setup supplier sensor */
  pinMode(VOLTAGE_PIN, INPUT);

  /* Setup statistic timer */
  _statsTimer.onNext([](void* context, const unsigned long) {
    ((Wheelly*)context)->handleStats();
  },
                     this);
  _statsTimer.interval(STATS_INTERVAL);
  _statsTimer.continuous(true);
  _statsTimer.start();
  _statsTime = millis();

  /* Setup lidar servo */
  _servo.offset(SERVO_OFFSET);
  _servo.begin();

  // Setup head tracking timer
  _headTrackingTimer.onNext([](void* context, const unsigned long n) {
    ((Wheelly*)context)->trackingHead();
  },
                            this);
  _headTrackingTimer.interval(HEAD_TRACKING_INTERVAL);
  _headTrackingTimer.continuous(true);
  _headTrackingTimer.start();

  // Setup head command timer
  _headCmdTimer.onNext([](void* context, const unsigned long n) {
    ((Wheelly*)context)->handleHeadTimeout();
  },
                       this);
  _headCmdTimer.interval(HEAD_COMMAND_TIMEOUT);
  _headCmdTimer.continuous(false);

  // Setup lidar
  _lidar.interval(DEFAULT_SCAN_INTERVAL);
  _lidar.onRange([](void* context, Lidar& lidar, const uint16_t frontDistance, const uint16_t rearDistance) {
    ((Wheelly*)context)->handleLidarRange(frontDistance, rearDistance);
  },
                 this);
  _lidar.begin();

  /* Setup contact sensors */
  _contactSensors.onChanged([](void* context, ContactSensors& sensors) {
    ((Wheelly*)context)->handleChangedContacts();
  },
                            this);
  _contactSensors.begin();

  /* Setup mpu */
  _display.clear();
  _display.print("Init Mpu...");

  /* Setup Motion controller */
  _motionCtrl.begin();

  /* Setup mpu */
  _mpu.onData([](void* context) {
    ((Wheelly*)context)->handleMpuData();
  },
              this);
  _mpu.onError([](void* context, const char* error) {
    ((Wheelly*)context)->sendSensorData("er", error);
  },
               this);

  _mpu.begin();
  _display.clear();
  _display.print("Calibrating Mpu...");
  _mpu.calibrate();
  _mpuTimeout = millis() + MPU_INTERVAL;

  _display.clear();

  digitalWrite(STATUS_LED_PIN, false);
  return true;
}

/*
   Pools wheelly controller
*/
void Wheelly::polling(const unsigned long t0) {
  /* Increments cps counter */
  _counter++;

  _statsTimer.polling(t0);

  /* Polls MPU */
  _mpu.polling(t0);
  _mpuError = _mpu.rc();
  if (t0 >= _mpuTimeout) {
    _mpuError = MPU_TIMEOUT_ERROR;
    ESP_LOGE(TAG, "!! mpu timeout");
  }

  /* Polls sensors */
  _headCmdTimer.polling(t0);
  _headTrackingTimer.polling(t0);
  _lidar.polling(t0);
  _servo.polling(t0);
  _contactSensors.polling(t0);

  /* Polls motion controller */
  _motionCtrl.polling(t0);

  /* Polls led */
  _ledActive = _mpuError != 0 || !canMoveForward() || !canMoveBackward() || !_onLine;
  _ledTimer.interval(
    _mpuError != 0 || (!canMoveForward() && !canMoveBackward()) || !_onLine
      ? FAST_LED_INTERVAL
      : SLOW_LED_INTERVAL);
  _ledTimer.polling(t0);

  if (t0 >= _supplySampleTimeout) {
    /* Polls for supplier sensor sample */
    sampleSupply();
    _supplySampleTimeout = t0 + SUPPLY_SAMPLE_INTERVAL;
  }

  if (t0 >= _supplyTimeout) {
    /* Polls for supplier sensor */
    averageSupply();
    sendSupply();
    _supplyTimeout = t0 + SUPPLY_INTERVAL;
  }

  if (t0 - _lastSend >= _sendInterval) {
    /* Polls for send motion */
    sendMotion(t0);
  }

  /* Refreshes LCD */
  _display.error(_mpuError);
  _display.move(!_motionCtrl.isHalt());
  _display.block(canMoveForward()
                   ? canMoveBackward()
                       ? NO_BLOCK
                       : BACKWARD_BLOCK
                 : canMoveBackward()
                   ? FORWARD_BLOCK
                   : FULL_BLOCK);
  _display.polling(t0);
}

void Wheelly::onLine(boolean onLine) {
  _onLine = onLine;
  if (onLine) {
    sendSensorData("hi", "hi");
  }
}

/**
   Queries and sends the status
*/
void Wheelly::queryStatus(void) {
  sendMotion(millis());
  sendLidar();
  sendContacts();
  sendSupply();
}

/*
   Moves the robot to the direction

   @param direction the direction (DEG)
   @param speed the speed (pps)
*/
void Wheelly::rotate(const int direction) {
  _motionCtrl.rotate(direction);
  if ((_motionCtrl.isForward() && !canMoveForward())
      || (_motionCtrl.isBackward() && !canMoveBackward())) {
    _motionCtrl.halt();
  }
}

/*
   Moves the robot to the target position

   @param xTarget the x target (pulses)
   @param yTarget the y target (pulses)
*/
void Wheelly::forward(const float xTarget, const float yTarget) {
  _motionCtrl.forward(xTarget, yTarget);
  if (_motionCtrl.isForward() && !canMoveForward()
      || _motionCtrl.isBackward() && !canMoveBackward()) {
    _motionCtrl.halt();
  }
}

/*
   Moves the robot to the target position

   @param xTarget the x target (pulses)
   @param yTarget the y target (pulses)
*/
void Wheelly::backward(const float xTarget, const float yTarget) {
  ESP_LOGD(TAG, "target %d,%d", xTarget, yTarget);
  _motionCtrl.backward(xTarget, yTarget);
  if (_motionCtrl.isForward() && !canMoveForward()
      || _motionCtrl.isBackward() && !canMoveBackward()) {
    _motionCtrl.halt();
  }
}

/*
   Resets wheelly
*/
void Wheelly::reset() {
  _motionCtrl.halt();
  _motionCtrl.reset(millis());
  _mpuError = 0;
}

/**
* Handles lidar positioned event
*/
void Wheelly::handleLidarRange(const uint16_t frontDistance, const uint16_t rearDistance) {
  ESP_LOGD(TAG, "%u mm, %u mm", frontDistance, rearDistance);
  boolean prevCanMove = canMoveForward();
  // Reads lidar measures
  _lidarTime = millis();
  _lidarYaw = _yaw;
  _lidarDirection = _servo.direction();
  _lidarXPulses = _motionCtrl.xPulses();
  _lidarYPulses = _motionCtrl.yPulses();
  _frontDistance = frontDistance;
  _rearDistance = rearDistance;

  /* Checks for obstacles */
  if (_motionCtrl.isForward() && !canMoveForward()
      || _motionCtrl.isBackward() && !canMoveBackward()) {
    /* Halt robot if cannot move */
    _motionCtrl.halt();
  }

  sendLidar();
  if (canMoveForward() != prevCanMove) {
    sendContacts();
  }

  /* Displays sensor data */
  if (_frontDistance == 0) {
    _display.distance(-1);
  } else {
    _display.distance(_frontDistance / 10);
  }
}

/*
   Handles sample event from distance sensor
*/
void Wheelly::handleMpuData() {
  _yaw = roundf(_mpu.yaw() * 180 / PI);
  _mpuTimeout = millis() + MPU_INTERVAL;
  _motionCtrl.angle(_yaw);
}

/**
   Handles changed contacts
*/
void Wheelly::handleChangedContacts(void) {
  ESP_LOGD(TAG, "Wheelly::handleChangedContacts");
  sendContacts();
}

/*
   Scans proximity to the direction
   @param angle the angle (DEG)
   @param t0 the scanning instant
*/
void Wheelly::scan(const int angle, const unsigned long t0) {
  _headStatus = HeadStatus::FIX_DIRECTION;
  _lidarTargetDirection = angle;
  _servo.direction(angle, t0);
  _headCmdTimer.start();
}

/**
  Start head tracking
*/
void Wheelly::headTrack(const boolean frontTrack, const float xTarget, const float yTarget, const unsigned long t0) {
  _headStatus = frontTrack ? HeadStatus::FRONT_TRACKING : HeadStatus::REAR_TRACKING;
  _xHeadTarget = xTarget;
  _yHeadTarget = yTarget;
  _headCmdTimer.start();
}

/**
  Handle head command timeout
*/
void Wheelly::handleHeadTimeout(void) {
  ESP_LOGD(TAG, "Wheelly::handleHeadTimeout");
  _headStatus = HeadStatus::FIX_DIRECTION;
  _lidarTargetDirection = 0;
  _servo.direction(0, millis());
}

/**
  Handles polling for head track

  @param t0 current time
*/
void Wheelly::trackingHead(void) {
  if (_headStatus == FIX_DIRECTION) {
    return;
  }
  ESP_LOGD(TAG, "Wheelly::trackingHead");
  // _yaw = robot direction
  // _servo.direction() = head direction
  // _motionCtrl.xPulses() = robot x position
  // _motionCtrl.yPulses() = robot y position

  float robotRad = (float)M_PI * _yaw / 180;
  // Compute head position

  float xHead = _motionCtrl.xPulses() + HEAD_DISTANCE * sinf(robotRad);
  float yHead = _motionCtrl.yPulses() + HEAD_DISTANCE * cosf(robotRad);
  ESP_LOGD(TAG, "  head=(%.1f, %.1f", xHead, yHead);

  float dx = _xHeadTarget - xHead;
  float dy = _yHeadTarget - yHead;

  float distance2 = dx * dx + dy * dy;

  ESP_LOGD(TAG, "  target=(%.1f, %.1)f", dx, dy);
  ESP_LOGD(TAG, "  d2=(%.1f, %.1f)", distance2);

  if (distance2 <= _antiGimbalRadius * _antiGimbalRadius) {
    // target too near do nothing
    return;
  }
  // compute direction
  float headRad = _headStatus == REAR_TRACKING ? atan2f(-dx, -dy) : atan2f(dx, dy);
  int headDeg = normalDeg((int)roundf(180 * (headRad - robotRad) / (float)M_PI));

  headDeg = clip(headDeg, _minHeadDir, _maxHeadDir);

  ESP_LOGD(TAG, "  head %d DEG", headDeg);

  _servo.direction(headDeg, millis());
}


/*
  Handles timeout event from statistics timer
*/
void Wheelly::handleStats(void) {
  const unsigned long t0 = millis();
  const unsigned long dt = (t0 - _statsTime);
  const unsigned long tps = _counter * 1000 / dt;
  _statsTime = t0;
  _counter = 0;
  char bfr[256];
  sprintf(bfr, "%ld %ld", t0, tps);
  sendSensorData("cs", bfr);
}

/*
  Handles timeout event from led timer
*/
void Wheelly::handleLed(const unsigned long n) {
  boolean ledStatus = _ledActive && (n % 2) == 0;
  digitalWrite(STATUS_LED_PIN, ledStatus);
}

/*
   Sends reply
*/
void Wheelly::sendSensorData(const String& topic, const String& data) {
  if (_onReply) {
    _onReply(_context, _pubSensorTopicPrefix + "/" + topic, data);
  }
}

/*
   Sends reply
*/
void Wheelly::sendCommandReply(const String& topic, const String& data) {
  if (_onReply) {
    _onReply(_context, topic, data);
  }
}

/**
   Samples the supply voltage
*/
void Wheelly::sampleSupply(void) {
  /* Samples the supply voltage */
  for (int i = 0; i < SAMPLE_BATCH; i++) {
    int sample = analogRead(VOLTAGE_PIN);
    _supplyTotal += sample;
    _supplySamples++;
  }
  int supply = (int)(_supplyTotal / _supplySamples);
}

/*
   Averages the supply measures
*/
void Wheelly::averageSupply(void) {
  if (_supplySamples > 0) {
    /* Averages the measures */
    _supplyTime = millis();
    _supplyVoltage = (int)(_supplyTotal / _supplySamples);

    ESP_LOGD(TAG, "supply=%d, samples=", _supplyVoltage, _supplySamples);

    _supplySamples = 0;
    _supplyTotal = 0;

    /* Computes the charging level */
    const int hLevel = min(max(
                             (int)map(_supplyVoltage, MIN_VOLTAGE_VALUE, MAX_VOLTAGE_VALUE, 0, 10),
                             0),
                           9);
    const int level = (hLevel + 1) / 2;

    ESP_LOGD(TAG, "hlevel=%d, level=%d", hLevel, level);

    /* Display the charging level */
    _display.supply(level);
  }
}

/*
   Sends the status of supply
*/
void Wheelly::sendSupply(void) {
  char bfr[256];
  /* sv time volt */
  sprintf(bfr, "%ld,%d",
          _supplyTime,
          _supplyVoltage);
  sendSensorData("sv", bfr);
}

/*
   Sends the motion of wheelly
*/
void Wheelly::sendMotion(const unsigned long t0) {
  char bfr[256];
  /* st time x y yaw lpps rpps err stat dir speed lspeed rspeed lpwr rpw xtarget ytarget */
  sprintf(bfr, "%ld,%.1f,%.1f,%d,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%.1f",
          millis(),
          (double)_motionCtrl.xPulses(),
          (double)_motionCtrl.yPulses(),
          _yaw,
          (double)_motionCtrl.leftPps(),
          (double)_motionCtrl.rightPps(),
          (const unsigned short)_mpuError,
          _motionCtrl.status(),
          _motionCtrl.direction(),
          0,
          _motionCtrl.leftMotor().speed(),
          _motionCtrl.rightMotor().speed(),
          _motionCtrl.leftMotor().pwm(),
          _motionCtrl.rightMotor().pwm(),
          _motionCtrl.xTarget(),
          _motionCtrl.yTarget());
  sendSensorData("mt", bfr);
  _lastSend = t0;
}

/*
   Sends the status of contacts
*/
void Wheelly::sendContacts(void) {

  char bfr[256];
  /* ct time frontSig rearSig canF canB */
  sprintf(bfr, "%ld,%d,%d,%d,%d",
          millis(),
          _contactSensors.frontClear(),
          _contactSensors.rearClear(),
          canMoveForward(),
          canMoveBackward());
  sendSensorData("ct", bfr);
}

/*
   Sends the status of proxy sensor
*/
void Wheelly::sendLidar(void) {
  char bfr[256];
  /* rg time frontDistance rearDistance x y yaw lidarDirection*/
  sprintf(bfr, "%ld,%u,%u,%.1f,%.1f,%d,%d,%d,%d,%.1f,%.1f",
          _lidarTime,
          _frontDistance,
          _rearDistance,
          _lidarXPulses,
          _lidarYPulses,
          _lidarYaw,
          _lidarDirection,
          _lidarTargetDirection,
          _headStatus,
          _xHeadTarget,
          _yHeadTarget);
  sendSensorData("rg", bfr);
}

/*
   Returns true if can move forward
*/
const boolean Wheelly::canMoveForward() const {
  return _contactSensors.frontClear() && (_frontDistance == 0 || _frontDistance > STOP_DISTANCE);
}

/*
   Returns true if can move backward
*/
const boolean Wheelly::canMoveBackward() const {
  return _contactSensors.rearClear();
}

/**
       Execute a command
       Returns true if command ok
       @param t0 the current time
       @param topic the command topic
       @param args the arguments
    */
const boolean Wheelly::execute(const unsigned long t0, const String& topic, const String& args) {
  ESP_LOGD(TAG, "Execute command %s %s", topic.c_str(), args.c_str());

  if (topic.endsWith("/ck")) {
    sendCommandReply(topic + "/res", args + "," + t0 + "," + millis());
    return true;
  } else if (topic.endsWith("/ha")) {
    halt();
    sendCommandReply(topic + "/res", args);
    return true;
  } else if (topic.endsWith("/sc")) {
    return handleScanCmd(t0, topic, args);
  } else if (topic.endsWith("/ht")) {
    return handleHtCmd(t0, topic, args);
  } else if (topic.endsWith("/ro")) {
    return handleRoCmd(t0, topic, args);
  } else if (topic.endsWith("/mv")) {
    return handleMvCmd(t0, topic, args);
  } else if (topic.endsWith("/cf")) {
    return handleCfCmd(t0, topic, args);
  } else if (topic.endsWith("/rs")) {
    reset();
    sendCommandReply(topic + "/res", args);
    return true;
  } else if (topic.endsWith("/vr")) {
    sendCommandReply(topic + "/res", version);
    return true;
  } else if (topic.endsWith("/qc")) {
    return handleQcCmd(t0, topic, args);
  } else if (topic.endsWith("/qs")) {
    queryStatus();
    sendCommandReply(topic + "/res", args);
    return true;
  } else {
    sendCommandReply(topic + "/err", "Wrong command " + args);
    return false;
  }
}

const boolean Wheelly::handleScanCmd(const unsigned long time, const String& topic, const String& args) {
  int direction;
  int count;
  if (sscanf(args.c_str(), "%d%n", &direction, &count) != 1 || count != args.length()) {
    ESP_LOGD(TAG, "Wrong args %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }
  if (!(direction >= _minHeadDir && direction <= _maxHeadDir)) {
    ESP_LOGD(TAG, "Wrong args %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  scan(direction, time);
  sendCommandReply(topic + "/res", args);
  return true;
}

const boolean Wheelly::handleHtCmd(const unsigned long time, const String& topic, const String& args) {
  int mode;
  float xTarget;
  float yTarget;
  int count;
  if (sscanf(args.c_str(), "%d,%f,%f%n", &mode, &xTarget, &yTarget, &count) != 3 || count != args.length()) {
    ESP_LOGD(TAG, "Wrong parse args %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }
  headTrack(mode == 0, xTarget, yTarget, time);

  sendCommandReply(topic + "/res", args);
  return true;
}

const boolean Wheelly::handleRoCmd(const unsigned long time, const String& topic, const String& args) {
  int direction;
  int count;
  if (sscanf(args.c_str(), "%d%n", &direction, &count) != 1 || count != args.length()) {
    ESP_LOGD(TAG, "Wrong parse args %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }
  if (!(direction >= -180 && direction <= 179)) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  rotate(direction);
  sendCommandReply(topic + "/res", args);
  return true;
}

const boolean Wheelly::handleMvCmd(const unsigned long time, const String& topic, const String& args) {
  int mode;
  float xTarget;
  float yTarget;
  int count;
  if (sscanf(args.c_str(), "%d,%f,%f%n", &mode, &xTarget, &yTarget, &count) != 3 || count != args.length()) {
    ESP_LOGD(TAG, "Wrong parse args %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }
  if (mode == 0) {
    forward(xTarget, yTarget);
  } else {
    backward(xTarget, yTarget);
  }
  sendCommandReply(topic + "/res", args);
  return true;
}

/*handleCf
  Returns the json configuration
*/
JsonDocument& Wheelly::jsonConfig(JsonDocument& doc) {
  const motionConfig_t& cfg = _motionCtrl.config();
  const tcsParams_t& motorCfg = _motionCtrl.leftMotor().tcs();
  doc["asr"] = motorCfg.asr;
  doc["lambdaFactor"] = motorCfg.lambdaFactor;
  doc["maxPulseInterval"] = motorCfg.maxPulseInterval;
  doc["delayedInterval"] = motorCfg.delayedInterval;
  doc["tau"] = _motionCtrl.tau();
  doc["minRotRange"] = cfg.minRotRange;
  doc["maxRotRange"] = cfg.maxRotRange;
  doc["maxRotPps"] = cfg.maxRotPps;
  doc["maxSpeed"] = cfg.maxSpeed;
  doc["haltDistance"] = cfg.haltDistance;
  doc["decelerateDistance"] = cfg.decelerateDistance;
  doc["minHeadDir"] = _minHeadDir;
  doc["maxHeadDir"] = _maxHeadDir;
  doc["sendInterval"] = _sendInterval;
  doc["scanInterval"] = _lidar.interval();
  doc["antiGimbalRadius"] = _antiGimbalRadius;

  return doc;
}

/*
  Applies the json configuration
*/
void Wheelly::applyJsonConfig(const JsonDocument& doc) {
  const motionConfig_t cfg = {
    .minRotRange = doc["minRotRange"],
    .maxRotRange = doc["maxRotRange"],
    .maxRotPps = doc["maxRotPps"],
    .maxSpeed = doc["maxSpeed"],
    .haltDistance = doc["haltDistance"],
    .decelerateDistance = doc["decelerateDistance"]
  };
  _motionCtrl.config(cfg);

  const tcsParams_t tcs = {
    .asr = doc["asr"],
    .maxPulseInterval = doc["maxPulseInterval"],
    .lambdaFactor = doc["lambdaFactor"],
    .delayedInterval = doc["delayedInterval"]
  };
  _motionCtrl.leftMotor().tcs(tcs);
  _motionCtrl.rightMotor().tcs(tcs);

  _motionCtrl.tau(doc["tau"]);
  _minHeadDir = doc["minHeadDir"];
  _maxHeadDir = doc["maxHeadDir"];
  _sendInterval = doc["sendInterval"];
  _lidar.interval(doc["scanInterval"]);
  _antiGimbalRadius = doc["antiGimbalRadius"];
}

/*
  Handle qc command (query config)
*/
const boolean Wheelly::handleQcCmd(const unsigned long time, const String& topic, const String& args) {
  StaticJsonDocument<256> doc;
  jsonConfig(doc);

  String json;
  serializeJson(doc, json);
  sendCommandReply(topic + "/res", json);
  return true;
}

/*
  Validates a configuration parameter and return true if valid
*/
const bool Wheelly::validateCfg(JsonDocument& cfg, const JsonDocument& doc, const String& key, const int minValue, const int maxValue, const String& topic) {
  if (doc[key].is<int>()) {
    int value = doc[key];
    if (!(value >= minValue && value <= maxValue)) {
      char msg[256];
      sprintf(msg, "Wrong %s %d", key.c_str(), value);
      ESP_LOGE(TAG, "%s", msg);
      sendCommandReply(topic + "/err", msg);
      return false;
    }
    cfg[key] = doc[key];
  }
  return true;
}

/*
  Handle cf command (config)
*/
const boolean Wheelly::handleCfCmd(const unsigned long time, const String& topic, const String& args) {
  StaticJsonDocument<256> doc;
  int count;
  // Read current configuration
  StaticJsonDocument<256> cfg;
  jsonConfig(cfg);

  // Parse json argument
  auto error = deserializeJson(doc, args);
  if (error) {
    ESP_LOGE(TAG, "Wrong json %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong json " + args);
    return false;
  }

  // Validate configuration
  if (!(validateCfg(cfg, doc, "tau", 1, 10000, topic)
        && validateCfg(cfg, doc, "asr", 1, 16383, topic)
        && validateCfg(cfg, doc, "lambdaFactor", 0, 100, topic)
        && validateCfg(cfg, doc, "maxPulseInterval", 1, 10000, topic)
        && validateCfg(cfg, doc, "delayedInterval", 1, 10000, topic)
        && validateCfg(cfg, doc, "minRotRange", 0, 180, topic)
        && validateCfg(cfg, doc, "maxRotRange", 0, 180, topic)
        && validateCfg(cfg, doc, "maxRotPps", 0, 20, topic)
        && validateCfg(cfg, doc, "maxSpeed", 0, 60, topic)
        && validateCfg(cfg, doc, "haltDistance", 0, 1000, topic)
        && validateCfg(cfg, doc, "decelerateDistance", 0, 1000, topic)
        && validateCfg(cfg, doc, "sendInterval", 1, 60000, topic)
        && validateCfg(cfg, doc, "scanInterval", 1, 60000, topic)
        && validateCfg(cfg, doc, "minHeadDir", -90, 90, topic)
        && validateCfg(cfg, doc, "maxHeadDir", -90, 90, topic)
        && validateCfg(cfg, doc, "antiGimbalRadius", 1, 60000, topic))) {
    return false;
  }

  // Applies changed configuration
  applyJsonConfig(cfg);
  sendCommandReply(topic + "/res", args);
  ESP_LOGI(TAG, "Configuration %s", args.c_str());
  return true;
}
