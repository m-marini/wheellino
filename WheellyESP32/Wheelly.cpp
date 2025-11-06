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
   Send Interval
*/
static const unsigned long DEFAULT_SEND_INTERVAL = 500ul;

/*
   Statistics
*/
static const unsigned long STATS_INTERVAL = 10000ul;

/*
   Proximity sensor
   Proximity distance scanner
*/
//static const unsigned long STOP_ECHO_TIME = (20ul * 5887 / 100);  // 20 cm
//static const unsigned long MAX_ECHO_TIME = (400ul * 5887 / 100);  // 400 cm
//static const unsigned long MIN_ECHO_TIME = (3ul * 5887 / 100);    // 3 cm
//static const int DISTANCE_TICK = 5;
static const uint16_t STOP_DISTANCE = 200;  // 200 mm

/*
   Proxy sensor servo
*/
static const unsigned long DEFAULT_SCAN_INTERVAL = 1000ul;
static const unsigned long SCANNER_RESET_INTERVAL = 1000ul;
static const int NO_SCAN_DIRECTIONS = 10;
static const int SERVO_OFFSET = 0;

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
    _servo(SERVO_PIN) {
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

  // Setup lidar
  _lidar.interval(_sendInterval);
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
   Moves the robot to the direction at speed

   @param direction the direction (DEG)
   @param speed the speed (pps)
*/
void Wheelly::move(const int direction, const int speed) {
  _motionCtrl.move(direction, speed);
  if (_motionCtrl.isForward() && !canMoveForward()
      || _motionCtrl.isBackward() && !canMoveBackward()) {
    _motionCtrl.halt();
  }
}

/**
   Configures intervals [send interval, scan interval]
   @param p the intervals
*/
void Wheelly::configIntervals(const int* intervals) {
  _sendInterval = intervals[0];
  _lidar.interval(intervals[1]);
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
  _servo.direction(angle, t0);
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
  /* st time x y yaw lpps rpps err halt dir speed lspeed rspeed lpwr rpw */
  sprintf(bfr, "%ld,%.1f,%.1f,%d,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d",
          millis(),
          (double)_motionCtrl.xPulses(),
          (double)_motionCtrl.yPulses(),
          _yaw,
          (double)_motionCtrl.leftPps(),
          (double)_motionCtrl.rightPps(),
          (const unsigned short)_mpuError,
          _motionCtrl.isHalt() ? 1 : 0,
          _motionCtrl.direction(),
          _motionCtrl.speed(),
          _motionCtrl.leftMotor().speed(),
          _motionCtrl.rightMotor().speed(),
          _motionCtrl.leftMotor().power(),
          _motionCtrl.rightMotor().power());
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
  sprintf(bfr, "%ld,%u,%u,%.1f,%.1f,%d,%d",
          _lidarTime,
          _frontDistance,
          _rearDistance,
          _lidarXPulses,
          _lidarYPulses,
          _lidarYaw,
          _lidarDirection);
  sendSensorData("rg", bfr);
}

/*
   Returns true if can move forward
*/
const boolean Wheelly::canMoveForward() const {
  return !(_frontDistance > 0 && _frontDistance <= STOP_DISTANCE) && _contactSensors.frontClear();
}

/*
   Returns true if can move backward
*/
const boolean Wheelly::canMoveBackward() const {
  return _contactSensors.rearClear();
}

/**
   Replies the configuration
*/
const boolean Wheelly::handleQcCmd(const unsigned long time, const String& topic, const String& args) {
  char bfr[512];

  MotorCtrl& leftMotor = _motionCtrl.leftMotor();
  MotorCtrl& rightMotor = _motionCtrl.leftMotor();
  sprintf(bfr, "%d,%d,%d,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld",
          _motionCtrl.minRotRange(),
          _motionCtrl.maxRotRange(),
          _motionCtrl.maxRotPps(),
          _motionCtrl.sensors().tau(),
          leftMotor.p0Forw(),
          leftMotor.p1Forw(),
          leftMotor.pxForw(),
          leftMotor.p0Back(),
          leftMotor.p1Back(),
          leftMotor.pxBack(),
          leftMotor.ax(),
          leftMotor.alpha(),
          leftMotor.muForw(),
          leftMotor.muBack(),
          rightMotor.p0Forw(),
          rightMotor.p1Forw(),
          rightMotor.pxForw(),
          rightMotor.p0Back(),
          rightMotor.p1Back(),
          rightMotor.pxBack(),
          rightMotor.ax(),
          rightMotor.alpha(),
          rightMotor.muForw(),
          rightMotor.muBack());
  sendCommandReply(topic + "/res", bfr);
  return true;
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
  } else if (topic.endsWith("/mv")) {
    return handleMoveCmd(t0, topic, args);
  } else if (topic.endsWith("/ci")) {
    return handleCiCmd(t0, topic, args);
  } else if (topic.endsWith("/cc")) {
    return handleCcCmd(t0, topic, args);
  } else if (topic.endsWith("/cs")) {
    return handleCsCmd(t0, topic, args);
  } else if (topic.endsWith("/tcsl")) {
    return handleTcsCmd(t0, topic, args);
  } else if (topic.endsWith("/tcsr")) {
    return handleTcsCmd(t0, topic, args);
  } else if (topic.endsWith("/fl")) {
    return handleFxCmd(t0, topic, args);
  } else if (topic.endsWith("/fr")) {
    return handleFxCmd(t0, topic, args);
  } else if (topic.endsWith("/rs")) {
    reset();
    sendCommandReply(topic + "/res", args);
    return true;
  } else if (topic.endsWith("/vr")) {
    sendCommandReply(topic + "/res", version);
    return true;
  } else if (topic.endsWith("/qs")) {
    queryStatus();
    sendCommandReply(topic + "/res", args);
    return true;
  } else if (topic.endsWith("/qc")) {
    return handleQcCmd(t0, topic, args);
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
  if (!(direction >= -90 && direction <= 90)) {
    ESP_LOGD(TAG, "Wrong args %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  scan(direction, time);
 sendCommandReply(topic + "/res", args);
  return true;
}

const boolean Wheelly::handleMoveCmd(const unsigned long time, const String& topic, const String& args) {
  int direction;
  int speed;
  int count;
  if (sscanf(args.c_str(), "%d,%d%n", &direction, &speed, &count) != 2 || count != args.length()) {
    ESP_LOGD(TAG, "Wrong parse args %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }
  if (!(direction >= -180 && direction <= 179 && speed >= -MAX_SPEED && speed <= MAX_SPEED)) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  move(direction, speed);
  sendCommandReply(topic + "/res", args);
  return true;
}

const boolean Wheelly::handleCiCmd(const unsigned long time, const String& topic, const String& args) {
  int params[2];
  int count;
  if (sscanf(args.c_str(), "%d,%d%n", &params[0], &params[1], &count) != 2 || count != args.length()) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  if (!(params[0] >= 1 && params[0] <= 60000 && params[1] >= 1 && params[1] <= 60000)) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  configIntervals(params);
  sendCommandReply(topic + "/res", args);
  return true;
}

const boolean Wheelly::handleCcCmd(const unsigned long time, const String& topic, const String& args) {
  int params[3];
  int count;

  if (sscanf(args.c_str(), "%d,%d,%d%n", &params[0], &params[1], &params[2], &count) != 3 || count != args.length()) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  if (!(params[0] >= 0 && params[0] <= 180
        && params[1] >= 0 && params[1] <= 180
        && params[2] >= 0 && params[2] <= 20)) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  configMotionController(params);
  sendCommandReply(topic + "/res", args);
  return true;
}

const boolean Wheelly::handleCsCmd(const unsigned long time, const String& topic, const String& args) {
  int tau;
  int count;

  if (sscanf(args.c_str(), "%d%n", &tau, &count) != 1 || count != args.length()) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  if (!(tau >= 1 && tau <= 10000)) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  configMotorSensors(tau);
  sendCommandReply(topic + "/res", args);
  return true;
}

const boolean Wheelly::handleTcsCmd(const unsigned long time, const String& topic, const String& args) {
  int params[8];
  int count;

  if (sscanf(args.c_str(), "%d,%d,%d,%d,%d,%d,%d,%d%n",
             &params[0], &params[1], &params[2], &params[3], &params[4], &params[5], &params[6], &params[7],
             &count)
        != 8
      || count != args.length()) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  if (!(params[0] >= 0 && params[0] <= 1000
        && params[1] >= 0 && params[1] <= 1000
        && params[2] >= 0 && params[2] <= 1000
        && params[3] >= -1000 && params[3] <= 0
        && params[4] >= -1000 && params[4] <= 0
        && params[5] >= -1000 && params[5] <= 0
        && params[6] >= 0 && params[6] <= 16383
        && params[7] >= 0 && params[7] <= 100)) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  configTcsMotorController(params, topic.endsWith("/tcsl"));
  sendCommandReply(topic + "/res", args);
  return true;
}

const boolean Wheelly::handleFxCmd(const unsigned long time, const String& topic, const String& args) {
  long params[2];
  int count;

  if (sscanf(args.c_str(), "%ld,%ld%n", &params[0], &params[1], &count) != 2 || count != args.length()) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  if (!(params[0] >= 0 && params[0] <= 2000000L
        && params[1] >= 0 && params[1] <= 2000000L)) {
    ESP_LOGD(TAG, "Wrong args values %s %s", topic.c_str(), args.c_str());
    sendCommandReply(topic + "/err", "Wrong args " + args);
    return false;
  }

  configFeedbackMotorController(params, topic.endsWith("/fl"));
  sendCommandReply(topic + "/res", args);
  return true;
}
