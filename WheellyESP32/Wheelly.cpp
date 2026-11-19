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
#include <esp_mac.h>
#include <pb.h>
#include <proto/pb_decode.h>

//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <esp_log.h>
static const char* TAG = "Wheelly";

#include "Wheelly.h"
#include "pins.h"
#include "MqttClient.h"
#include "Display.h"
#include "Contacts.h"
#include "mpu6050mm.h"
#include "MotionCtrl.h"
#include "Lidar.h"
#include "LidarServo.h"
#include "Timer.h"

#include "src/robot.pb.h"

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

#define TOPIC_LEN 129

typedef void (*command_cb_t)(const char* topic, byte* msg, unsigned int len);

typedef struct {
  const char* suffix;
  command_cb_t handler;
} command_route_t;

Wheelly wheelly;

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
//static const int MIN_VOLTAGE_VALUE = 1896;
//static const int MAX_VOLTAGE_VALUE = 2621;
static const int MIN_VOLTAGE_VALUE = 790;
static const int MAX_VOLTAGE_VALUE = 1092;
static char _id[33];
static char _pubSensorTopicPrefix[TOPIC_LEN];
static char _subCommandTopics[TOPIC_LEN];

static boolean _onLine;
static int _yaw;
static unsigned long _mpuTimeout;
static uint8_t _mpuError;

static unsigned long _sendInterval;
static unsigned long _lastSend;

static boolean _ledActive;

static unsigned long _counter;
static unsigned long _statsTime;

static unsigned long _lidarTime;
static uint16_t _frontDistance;
static uint16_t _rearDistance;
static int _lidarYaw;
static int _lidarDirection;
static int _lidarTargetDirection;
static float _lidarXPulses;
static float _lidarYPulses;
static HeadStatus _headStatus;
static float _xHeadTarget;
static float _yHeadTarget;
static int _antiGimbalRadius;

static unsigned long _supplyTimeout;
static unsigned long _supplySampleTimeout;
static unsigned long _supplyTime;
static int _supplyVoltage;
static long _supplyTotal;
static int _supplySamples;
static int _minHeadDir;
static int _maxHeadDir;

static DisplayClass _display;
static MPU6050Class _mpu;
static LidarServo _servo(SERVO_PIN);
static Lidar _lidar(FRONT_LIDAR_PIN, REAR_LIDAR_PIN);
static ContactSensors _contactSensors(FRONT_CONTACTS_PIN, REAR_CONTACTS_PIN);
static MotionCtrlClass _motionCtrl(LEFT_FORW_PIN, LEFT_BACK_PIN, RIGHT_FORW_PIN, RIGHT_BACK_PIN, LEFT_PIN, RIGHT_PIN);
static Timer _ledTimer;
static Timer _statsTimer;
static Timer _headTrackingTimer;
static Timer _headCmdTimer;

/*******************************
   Sensor section
********************************/

/**
 * @brief Checks whether the robot is permitted to move forward.
 * 
 * Evaluates both the front contact sensors and the forward distance sensor to ensure 
 * the path ahead is unobstructed. Forward movement is allowed only if no front 
 * contact is detected and the measured obstacle distance exceeds `STOP_DISTANCE` 
 * (or reads `0`, indicating no obstacle within sensor range).
 * 
 * @return `true` if the forward path is clear and movement is safe; `false` otherwise.
 */
static bool canMoveForward() {
  return _contactSensors.frontClear() && (_frontDistance == 0 || _frontDistance > STOP_DISTANCE);
}

/**
 * @brief Checks whether the robot is permitted to move backward.
 * 
 * Evaluates the rear contact sensors to verify that no physical collision or 
 * obstruction is currently detected behind the robot.
 * 
 * @return `true` if the rear path is clear and reverse movement is safe; `false` otherwise.
 */
static const bool canMoveBackward() {
  return _contactSensors.rearClear();
}

/**
 * @brief Samples the analogue supply voltage pin and accumulates readings.
 * 
 * Performs `SAMPLE_BATCH` consecutive analogue-to-digital conversions on `VOLTAGE_PIN`, 
 * adding each raw reading to the running sum `_supplyTotal` and incrementing the 
 * sample counter `_supplySamples` for subsequent averaging.
 */
static void sampleSupply(void) {
  /* Samples the supply voltage */
  for (int i = 0; i < SAMPLE_BATCH; i++) {
    int sample = analogRead(VOLTAGE_PIN);
    _supplyTotal += sample;
    _supplySamples++;
  }
}

/**
 * @brief Calculates the average supply voltage and updates the battery level indicator.
 * 
 * If at least one voltage sample has been accumulated, records the current system 
 * timestamp, computes the mean supply voltage from the running total, and resets 
 * the sample accumulators. Maps the averaged voltage onto a 6-level charging scale 
 * (0 to 5) and updates the display indicator accordingly.
 */
static void averageSupply(void) {
  if (_supplySamples > 0) {
    /* Averages the measures */
    _supplyTime = millis();
    _supplyVoltage = (int)(_supplyTotal / _supplySamples);

    ESP_LOGD(TAG, "supply=%d, samples=", _supplyVoltage, _supplySamples);

    _supplySamples = 0;
    _supplyTotal = 0;

    /* Computes the charging level (6 levels) */
    const int hLevel = constrain((int)map(_supplyVoltage, MIN_VOLTAGE_VALUE, MAX_VOLTAGE_VALUE, 0, 10), 0, 10);
    const int level = (hLevel + 1) / 2;

    ESP_LOGD(TAG, "hlevel=%d, level=%d", hLevel, level);

    /* Display the charging level */
    _display.supply(level);
  }
}

/**
 * @brief Immediately halts all robot locomotion.
 * 
 * Commands the internal motion controller to stop any active forward, backward, 
 * or rotational movement immediately.
 */
static void halt(void) {
  _motionCtrl.halt();
}

/*
   Scans proximity to the direction
   @param angle the angle (DEG)
   @param t0 the scanning instant
*/
static void scan(const int angle, const unsigned long t0) {
  _headStatus = HeadStatus::FIX_DIRECTION;
  _lidarTargetDirection = angle;
  _servo.direction(angle, t0);
  _headCmdTimer.start();
}


/**
 * @brief Starts head tracking towards a specified target point.
 * 
 * Sets the operational head status to tracking mode (front or rear), stores the 
 * target coordinates, and initialises the internal command timer.
 * 
 * @param frontTrack Direction flag: `true` to enable front tracking (`FRONT_TRACKING`), 
 *                   `false` to enable rear tracking (`REAR_TRACKING`).
 * @param xTarget    Target X-coordinate in the reference space.
 * @param yTarget    Target Y-coordinate in the reference space.
 */
static void headTrack(const boolean frontTrack, const float xTarget, const float yTarget) {
  _headStatus = frontTrack ? HeadStatus::FRONT_TRACKING : HeadStatus::REAR_TRACKING;
  _xHeadTarget = xTarget;
  _yHeadTarget = yTarget;
  _headCmdTimer.start();
}

/**
 * @brief Rotates the robot towards a specified direction while enforcing movement safety checks.
 * 
 * Commands the motion controller to initiate a rotation towards the target angle. 
 * Immediately verifies whether the resulting motion direction (forward or backward) 
 * is permitted by the robot's safety constraints; if obstructed or disallowed, 
 * the motion controller is halted immediately.
 * 
 * @param direction Target rotation angle or heading (in degrees).
 */
static void rotate(const int direction) {
  _motionCtrl.rotate(direction);
  if ((_motionCtrl.isForward() && !canMoveForward())
      || (_motionCtrl.isBackward() && !canMoveBackward())) {
    _motionCtrl.halt();
  }
}

/**
 * @brief Moves the robot forward towards a specified target point whilst enforcing safety checks.
 * 
 * Commands the motion controller to drive forward towards the given coordinates. 
 * Immediately verifies whether the resulting motion direction (forward or backward) 
 * is permitted by the robot's obstacle and contact sensors; if the path is obstructed 
 * or disallowed, the motion controller is halted immediately.
 * 
 * @param xTarget Target X-coordinate in the reference space.
 * @param yTarget Target Y-coordinate in the reference space.
 */
static void forward(const float xTarget, const float yTarget) {
  _motionCtrl.forward(xTarget, yTarget);
  if (_motionCtrl.isForward() && !canMoveForward()
      || _motionCtrl.isBackward() && !canMoveBackward()) {
    _motionCtrl.halt();
  }
}

/**
 * @brief Moves the robot backward towards a specified target point whilst enforcing safety checks.
 * 
 * Logs the target coordinates at debug level and commands the motion controller to 
 * drive in reverse towards the given position. Immediately verifies whether the 
 * resulting motion direction (forward or backward) is permitted by the robot's 
 * safety sensors; if obstructed or disallowed, the motion controller is halted immediately.
 * 
 * @param xTarget Target X-coordinate in the reference space.
 * @param yTarget Target Y-coordinate in the reference space.
 */
static void backward(const float xTarget, const float yTarget) {
  ESP_LOGD(TAG, "target %d,%d", xTarget, yTarget);
  _motionCtrl.backward(xTarget, yTarget);
  if (_motionCtrl.isForward() && !canMoveForward()
      || _motionCtrl.isBackward() && !canMoveBackward()) {
    _motionCtrl.halt();
  }
}

/**
 * @brief Timer callback that updates the head servo orientation to track the active target.
 * 
 * Exits immediately if the head is currently in `FIX_DIRECTION` mode. Otherwise, 
 * calculates the head's global position from the robot's odometry and heading (`_yaw`), 
 * determines the squared Euclidean distance to the target, and skips actuation if 
 * the target lies within the `_antiGimbalRadius` dead zone. When outside the radius, 
 * computes the relative bearing for either front or rear tracking, normalises and 
 * clamps the angle within `[_minHeadDir, _maxHeadDir]`, and commands the servo.
 * 
 * @param context Opaque user-defined context pointer passed by the timer scheduler 
 *                (currently unused).
 * @param n       Timer tick count or elapsed interval parameter passed by the timer 
 *                scheduler (currently unused).
 */
static void trackingHead(void* context, const unsigned long n) {
  if (_headStatus == FIX_DIRECTION) {
    return;
  }
  ESP_LOGD(TAG, "Wheelly::trackingHead");
  // _yaw = robot direction
  // _servo.direction() = head direction
  // _motionCtrl.xPulses() = robot x position
  // _motionCtrl.yPulses() = robot y position1135

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

/**
 * @brief Resets the robot controller to its default idle state.
 * 
 * Immediately halts any active locomotion, resets the motion controller with the 
 * current system timestamp, centres the scanner to 0 degrees, and clears any 
 * accumulated MPU (Motion Processing Unit) error flags or counters.
 */
static void reset(void) {
  _motionCtrl.halt();
  _motionCtrl.reset(millis());
  scan(0, millis());
  _mpuError = 0;
}

/***********************
  Old send message section
*************************/

/*
   Sends reply
*/
static void sendCommandReply(const String& topic, const String& data) {
  mqttClient.send(topic.c_str(), (uint8_t*)data.c_str(), data.length());
  wheelly.activity();
}

/***********************
  Send message section
*************************/

/**
 * @brief Publishes a raw binary message to a specified MQTT topic and records system activity.
 *
 * This helper function forwards the given byte payload to the underlying MQTT client
 * and invokes @c wheelly.activity() to activate the activity icon on the display,
 * providing visual feedback of outgoing network traffic.
 *
 * @param[in] topic   Null-terminated C-string specifying the destination MQTT topic.
 * @param[in] payload Pointer to the raw byte buffer containing the message payload.
 * @param[in] length  Exact size of the payload buffer in bytes.
 */
static void sendMessage(const char* topic, byte* payload, unsigned int length) {
  mqttClient.send(topic, payload, length);
  wheelly.activity();
}

/**
 * @brief Appends a suffix to a base MQTT topic, publishes a raw binary message, and activates the display activity icon.
 *
 * This overloaded helper concatenates @p topic and @p suffix into a fixed-size stack buffer
 * of length @c TOPIC_LEN before forwarding the message to the base @c sendMessage() overload.
 * If the combined topic exceeds @c TOPIC_LEN - 1 characters, it will be safely truncated.
 *
 * @param[in] topic   Null-terminated C-string specifying the base MQTT topic.
 * @param[in] suffix  Null-terminated C-string to append to the base topic (e.g. "/res" or "/err").
 * @param[in] payload Pointer to the raw byte buffer containing the message payload.
 * @param[in] length  Exact size of the payload buffer in bytes.
 */
static void sendMessage(const char* topic, const char* suffix, byte* payload, unsigned int length) {
  char fullTopic[TOPIC_LEN];
  snprintf(fullTopic, sizeof(fullTopic), "%s%s", topic, suffix);
  sendMessage(fullTopic, payload, length);
}

/**
 * @brief Publishes a binary response message to an MQTT command topic and activates the display activity icon.
 *
 * This convenience wrapper appends the standard @c "/res" suffix to @p topic and delegates
 * transmission to @c sendMessage(), providing visual feedback of outgoing traffic on the display.
 *
 * @param[in] topic   Null-terminated C-string specifying the base command MQTT topic.
 * @param[in] payload Pointer to the raw byte buffer containing the response payload.
 * @param[in] length  Exact size of the payload buffer in bytes.
 */
static void sendResponse(const char* topic, byte* payload, unsigned int length) {
  sendMessage(topic, "/res", payload, length);
}

/**
 * @brief Serialises a Protobuf CommandError message, publishes it to the error subtopic,
 *        and activates the display activity icon.
 *
 * This helper constructs a @c wheelly_protobuf_robot_CommandError message from the given
 * command @p id and human-readable @p msg, encodes it into a fixed-size stack buffer via
 * Nanopb, and transmits the resulting binary payload to @p topic with the @c "/err" suffix
 * appended. If encoding fails, an error is logged via @c ESP_LOGE and no message is sent.
 *
 * @param[in] topic Null-terminated C-string specifying the base command MQTT topic.
 * @param[in] id    Unique 64-bit identifier of the command that triggered the error.
 * @param[in] msg   Null-terminated C-string containing the error description (safely truncated
 *                  to fit @c protoMsg.message).
 */
static void sendError(const char* topic, uint64_t id, const char* msg) {
  wheelly_protobuf_robot_CommandError protoMsg = {
    .id = id
  };
  snprintf(protoMsg.message, sizeof(protoMsg.message), "%s", msg);
  uint8_t buffer[96];  // Buffer per i byte serializzati
  pb_ostream_t streamOut = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Encode message
  bool status = pb_encode(&streamOut, wheelly_protobuf_robot_CommandError_fields, &protoMsg);
  if (!status) {
    ESP_LOGE(TAG, "Encode error %s", PB_GET_ERROR(&streamOut));
    return;
  }
  size_t len = streamOut.bytes_written;
  sendMessage(topic, "/err", buffer, len);
}

/**
 * @brief Serialises a Protobuf CommandAck message, publishes it to the response subtopic,
 *        and activates the display activity icon.
 *
 * This helper constructs a @c wheelly_protobuf_robot_CommandAck message with the given
 * command @p id, encodes it into a fixed-size 16-byte stack buffer via Nanopb, and
 * transmits the resulting binary payload via @c sendResponse() (appending @c "/res" to
 * @p topic). If encoding fails, an error is logged via @c ESP_LOGE and no message is sent.
 *
 * @param[in] topic Null-terminated C-string specifying the base command MQTT topic.
 * @param[in] id    Unique 64-bit identifier of the acknowledged command.
 */
static void sendCommandAck(const char* topic, uint64_t id) {
  wheelly_protobuf_robot_CommandAck msg = {
    .id = id
  };
  uint8_t buffer[16];  // Buffer per i byte serializzati
  pb_ostream_t streamOut = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Encode message
  bool status = pb_encode(&streamOut, wheelly_protobuf_robot_CommandAck_fields, &msg);
  if (!status) {
    ESP_LOGE(TAG, "Encode error %s", PB_GET_ERROR(&streamOut));
    return;
  }
  size_t len = streamOut.bytes_written;
  sendResponse(topic, buffer, len);
}

/**
 * @brief Publishes serialised sensor telemetry data to an MQTT topic.
 * 
 * Constructs the full MQTT publication topic by prepending `_pubSensorTopicPrefix` 
 * to the specified sub-topic, transmits the binary payload via the MQTT client, 
 * and records network activity on the `wheelly` controller.
 * 
 * @param topic  Null-terminated sub-topic string identifying the specific sensor channel 
 *               (e.g. `"rg"`).
 * @param buffer Pointer to the byte buffer containing the Protobuf-encoded telemetry payload.
 * @param size   Length of the payload buffer in bytes.
 */
static void sendSensorData(const char* topic, const uint8_t* buffer, const size_t size) {
  char top[128];
  snprintf(top, sizeof(top), "%s/%s", _pubSensorTopicPrefix, topic);
  mqttClient.send(top, buffer, size);
  wheelly.activity();
}

/**
 * @brief Serialises and publishes the current LiDAR and head-tracking telemetry.
 * 
 * Populates a Nanopb `LidarTelemetry` message with the latest distance measurements, 
 * odometry pulses, robot orientation, head direction, head status, and target 
 * coordinates. Encodes the message into a binary buffer and dispatches it on the 
 * `"rg"` (range) sensor data channel.
 */
static void sendLidar(void) {
  wheelly_protobuf_robot_LidarTelemetry message = {
    .timestamp = _lidarTime,
    .frontDistance = _frontDistance,
    .rearDistance = _rearDistance,
    .x = _lidarXPulses,
    .y = _lidarYPulses,
    .robotYaw = _lidarYaw,
    .headDirection = _lidarDirection,
    .headStatus = (wheelly_protobuf_robot_HeadStatusId)_headStatus,
    .headTarget = _lidarTargetDirection,
    .xTarget = _xHeadTarget,
    .yTarget = _yHeadTarget
  };

  uint8_t buffer[64];  // Buffer per i byte serializzati
  pb_ostream_t streamOut = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Encode message
  bool status = pb_encode(&streamOut, wheelly_protobuf_robot_LidarTelemetry_fields, &message);
  if (!status) {
    ESP_LOGE(TAG, "Encode error %s", PB_GET_ERROR(&streamOut));
    return;
  }

  size_t len = streamOut.bytes_written;
  sendSensorData("rg", buffer, len);
}

/**
 * @brief Serialises and publishes the current motion and motor telemetry.
 * 
 * Populates a Nanopb `MotionTelemetry` message with odometry pulses, robot heading, 
 * measured and target wheel speeds (PPS), motor PWM power levels, motion status, 
 * target coordinates, and IMU failure status. Encodes the message into a binary 
 * buffer, dispatches it on the `"mt"` (motion) sensor data channel, and updates 
 * the last transmission timestamp.
 * 
 * @param t0 Reference timestamp (in milliseconds) recorded as `_lastSend` upon 
 *           successful transmission.
 */
static void sendMotion(const unsigned long t0 = millis()) {
  wheelly_protobuf_robot_MotionTelemetry message = {
    .timestamp = millis(),
    .x = _motionCtrl.xPulses(),
    .y = _motionCtrl.yPulses(),
    .robotDeg = _yaw,
    .leftPps = _motionCtrl.leftPps(),
    .rightPps = _motionCtrl.rightPps(),
    .imuFailure = _mpuError,
    .statusId = (wheelly_protobuf_robot_MotionStatusId)_motionCtrl.status(),
    .targetDeg = _motionCtrl.direction(),
    .xTarget = _motionCtrl.xTarget(),
    .yTarget = _motionCtrl.yTarget(),
    .leftTargetPps = _motionCtrl.leftMotor().speed(),
    .rightTargetPps = _motionCtrl.rightMotor().speed(),
    .leftPower = _motionCtrl.leftMotor().pwm(),
    .rightPower = _motionCtrl.rightMotor().pwm()
  };
  uint8_t buffer[128];  // Buffer per i byte serializzati
  pb_ostream_t streamOut = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Encode message
  bool status = pb_encode(&streamOut, wheelly_protobuf_robot_MotionTelemetry_fields, &message);
  if (!status) {
    ESP_LOGE(TAG, "Encode error %s", PB_GET_ERROR(&streamOut));
    return;
  }

  size_t len = streamOut.bytes_written;
  sendSensorData("mt", buffer, len);
  _lastSend = t0;
}

/**
 * @brief Serialises and publishes the current contact sensor and movement permission telemetry.
 * 
 * Populates a Nanopb `ContactsTelemetry` message with the current timestamp, front and 
 * rear contact sensor states, and directional movement flags (`canMoveForward` and 
 * `canMoveBackward`). Encodes the message into a binary buffer and dispatches it on 
 * the `"ct"` (contacts) sensor data channel.
 */
static void sendContacts(void) {
  wheelly_protobuf_robot_ContactsTelemetry message = {
    .timestamp = millis(),
    .frontSensor = _contactSensors.frontClear(),
    .rearSensor = _contactSensors.rearClear(),
    .canMoveForward = canMoveForward(),
    .canMoveBackward = canMoveBackward()
  };

  uint8_t buffer[32];  // Buffer per i byte serializzati
  pb_ostream_t streamOut = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Encode message
  bool status = pb_encode(&streamOut, wheelly_protobuf_robot_ContactsTelemetry_fields, &message);
  if (!status) {
    ESP_LOGE(TAG, "Encode error %s", PB_GET_ERROR(&streamOut));
    return;
  }

  size_t len = streamOut.bytes_written;
  sendSensorData("ct", buffer, len);
}

/**
 * @brief Serialises and publishes the current power supply telemetry.
 * 
 * Populates a Nanopb `SupplyTelemetry` message with the measurement timestamp and 
 * supply voltage reading. Encodes the message into a binary buffer and dispatches 
 * it on the `"sv"` (supply voltage) sensor data channel.
 */
static void sendSupply(void) {
  wheelly_protobuf_robot_SupplyTelemetry message = {
    .timestamp = _supplyTime,
    .voltage = _supplyVoltage
  };

  uint8_t buffer[32];
  pb_ostream_t streamOut = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Encode message
  bool status = pb_encode(&streamOut, wheelly_protobuf_robot_SupplyTelemetry_fields, &message);
  if (!status) {
    ESP_LOGE(TAG, "Encode error %s", PB_GET_ERROR(&streamOut));
    return;
  }

  size_t len = streamOut.bytes_written;
  sendSensorData("sv", buffer, len);
}

/**
 * @brief Serialises and publishes a periodic heartbeat ("hi") message.
 * 
 * Populates a Nanopb `HiMessage` with the current system timestamp, encodes it into 
 * a binary buffer, and dispatches it on the `"hi"` sensor data channel to signal 
 * that the robot controller is active and connected.
 */
static void sendHi(void) {
  wheelly_protobuf_robot_HiMessage message = {
    .timestamp = millis(),
  };

  uint8_t buffer[32];
  pb_ostream_t streamOut = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Encode message
  bool status = pb_encode(&streamOut, wheelly_protobuf_robot_HiMessage_fields, &message);
  if (!status) {
    ESP_LOGE(TAG, "Encode error %s", PB_GET_ERROR(&streamOut));
    return;
  }

  size_t len = streamOut.bytes_written;
  sendSensorData("hi", buffer, len);
}

/**
 * @brief Queries and publishes the current operational status of all robot subsystems.
 * 
 * Triggers an immediate telemetry dispatch across all primary sensor and actuator 
 * channels by publishing motion metrics, LiDAR scan data, contact sensor states, 
 * and power supply readings.
 */
static void queryStatus(void) {
  sendMotion(millis());
  sendLidar();
  sendContacts();
  sendSupply();
}

/**
 * @brief Callback invoked when new front and rear LiDAR range measurements are available.
 * 
 * Records the latest distance readings alongside the current timestamp, robot heading, 
 * head orientation, and odometry pulse counts. Evaluates obstacle safety constraints 
 * and immediately halts locomotion if the active movement direction becomes obstructed. 
 * Publishes updated LiDAR telemetry, dispatches a contact state update if forward 
 * movement permission has changed, and refreshes the distance readout on the display.
 * 
 * @param context    Opaque user-defined context pointer passed by the LiDAR driver 
 *                   (currently unused).
 * @param lidar      Reference to the `Lidar` sensor instance that generated the event 
 *                   (currently unused).
 * @param frontRange Measured distance to the nearest forward obstacle in millimetres 
 *                   (`0` indicates no obstacle detected within sensor range).
 * @param rearRange  Measured distance to the nearest rear obstacle in millimetres 
 *                   (`0` indicates no obstacle detected within sensor range).
 */
static void handleLidarRange(void* context, Lidar& lidar, const uint16_t frontRange, const uint16_t rearRange) {
  ESP_LOGD(TAG, "%u mm, %u mm", frontDistance, rearDistance);
  boolean prevCanMove = canMoveForward();
  // Reads lidar measures
  _lidarTime = millis();
  _lidarYaw = _yaw;
  _lidarDirection = _servo.direction();
  _lidarXPulses = _motionCtrl.xPulses();
  _lidarYPulses = _motionCtrl.yPulses();
  _frontDistance = frontRange;
  _rearDistance = rearRange;

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

/**
 * @brief Timer callback that drives the status LED blinking behaviour.
 * 
 * Evaluates whether the status LED is enabled (`_ledActive`) and toggles the 
 * physical output pin on even timer ticks (`n % 2 == 0`), producing a regular 
 * blink pattern whilst active and keeping the LED off otherwise.
 * 
 * @param context Opaque user-defined context pointer passed by the timer scheduler 
 *                (currently unused).
 * @param n       Monotonically increasing timer tick counter used to alternate 
 *                the LED state.
 */
static void handleLed(void* context, const unsigned long n) {
  bool ledStatus = _ledActive && (n % 2) == 0;
  digitalWrite(STATUS_LED_PIN, ledStatus);
}

/**
 * @brief Callback invoked when new yaw orientation data is sampled from the MPU.
 * 
 * Converts the yaw angle reported by `_mpu` from radians into rounded degrees, 
 * updates the MPU watchdog timeout deadline (`_mpuTimeout`), and forwards the 
 * new heading angle to the motion controller.
 * 
 * @param context Opaque user-defined context pointer passed by the sensor driver 
 *                (currently unused).
 */
static void handleMpuData(void* context) {
  _yaw = roundf(_mpu.yaw() * 180 / PI);
  _mpuTimeout = millis() + MPU_INTERVAL;
  _motionCtrl.angle(_yaw);
}

/**
 * @brief Timer callback that computes and publishes main-loop execution frequency telemetry.
 * 
 * Calculates the elapsed time since the previous measurement (`dt`), derives the 
 * execution rate in ticks per second (`tps`) from `_counter`, and resets both the 
 * timestamp and cycle counter. Serialises the resulting metrics into a Nanopb 
 * `StatsTelemetry` message and dispatches it on the `"cs"` (cycle stats) sensor 
 * data channel.
 * 
 * @param context Opaque user-defined context pointer passed by the timer scheduler 
 *                (currently unused).
 * @param n       Monotonically increasing timer tick counter (currently unused).
 */
static void handleStats(void* context, const unsigned long n) {
  const unsigned long t0 = millis();
  const unsigned long dt = (t0 - _statsTime);
  const unsigned long tps = _counter * 1000 / dt;
  _statsTime = t0;
  _counter = 0;

  wheelly_protobuf_robot_StatsTelemetry message = {
    .timestamp = t0,
    .stats = tps
  };

  uint8_t buffer[32];
  pb_ostream_t streamOut = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Encode message
  bool status = pb_encode(&streamOut, wheelly_protobuf_robot_StatsTelemetry_fields, &message);
  if (!status) {
    ESP_LOGE(TAG, "Encode error %s", PB_GET_ERROR(&streamOut));
    return;
  }

  size_t len = streamOut.bytes_written;
  sendSensorData("cs", buffer, len);
}

/**
 * @brief Callback invoked when a state change is detected on the contact sensors.
 * 
 * Logs the contact state change event at debug level and immediately dispatches 
 * updated contact sensor and movement permission telemetry via `sendContacts()`.
 * 
 * @param context Opaque user-defined context pointer passed by the sensor driver 
 *                (currently unused).
 * @param sensors Reference to the `ContactSensors` instance that triggered the event 
 *                (currently unused).
 */
static void handleChangedContacts(void* context, ContactSensors& sensors) {
  ESP_LOGD(TAG, "Wheelly::handleChangedContacts");
  sendContacts();
}

/**
 * @brief Timer callback invoked when the head-tracking command times out.
 * 
 * Logs the timeout event at debug level, resets the head operational state back to 
 * `HeadStatus::FIX_DIRECTION`, clears the target direction to 0 degrees, and 
 * commands the head servo to return to its centred position.
 * 
 * @param context Opaque user-defined context pointer passed by the timer scheduler 
 *                (currently unused).
 * @param n       Monotonically increasing timer tick counter (currently unused).
 */
static void handleHeadTimeout(void* context, const unsigned long n) {
  ESP_LOGD(TAG, "Wheelly::handleHeadTimeout");
  _headStatus = HeadStatus::FIX_DIRECTION;
  _lidarTargetDirection = 0;
  _servo.direction(0, millis());
}

/**************************
  Command hadlers section
***************************/

/**
 * @brief Decodes a raw binary Protobuf payload into a specified Nanopb message structure.
 * 
 * Wraps the input byte buffer in a Nanopb input stream and attempts to deserialise it 
 * according to the provided message descriptor. If decoding fails, the Nanopb stream 
 * error is logged alongside the originating topic and an error response is dispatched.
 * 
 * @tparam T       Nanopb-generated message struct type to populate.
 * @param topic    Null-terminated string representing the messaging topic on which 
 *                 the command was received.
 * @param payload  Pointer to the raw byte buffer containing the Protobuf-encoded data.
 * @param length   Size of the payload buffer in bytes.
 * @param fields   Pointer to the Nanopb message descriptor array (`*_fields`) matching type `T`.
 * @param[out] msg Reference to the target message structure where decoded fields will be stored.
 * 
 * @return `true` if the payload was successfully decoded into `msg`; `false` otherwise.
 */
template<typename T>
static bool decodeCommand(const char* topic, byte* payload, unsigned int length, const pb_msgdesc_t* fields, T& msg) {
  pb_istream_t stream = pb_istream_from_buffer(payload, length);
  if (!pb_decode(&stream, fields, &msg)) {
    ESP_LOGE(TAG, "Error decoding: %s", PB_GET_ERROR(&stream));
    return false;
  }
  return true;
}

/**
 * @brief Decodes and executes a Protobuf HaltCommand received via MQTT.
 *
 * This handler deserialises the incoming @p payload into a
 * @c wheelly_protobuf_robot_HaltCommand structure using Nanopb, invokes @c halt()
 * to stop the robot immediately, and sends a command acknowledgement via
 * @c sendCommandAck(). If decoding fails, an error is logged and no action is taken.
 *
 * @param[in] topic   Null-terminated C-string specifying the command MQTT topic.
 * @param[in] payload Pointer to the raw byte buffer containing the serialised HaltCommand.
 * @param[in] length  Exact size of the payload buffer in bytes.
 *
 * @return @c true if the command was decoded and executed successfully;
 *         @c false otherwise.
 */
static void handleHaCmd(const char* topic, byte* payload, unsigned int length) {
  wheelly_protobuf_robot_HaltCommand msg = wheelly_protobuf_robot_HaltCommand_init_zero;
  if (!decodeCommand<wheelly_protobuf_robot_HaltCommand>(topic, payload, length,
                                                         wheelly_protobuf_robot_HaltCommand_fields, msg)) {
    return;
  }
  halt();
  sendCommandAck(topic, msg.id);
}

/**
 * @brief Decodes and executes a Protobuf ScanCommand received via MQTT.
 *
 * This handler deserialises the incoming @p payload into a
 * @c wheelly_protobuf_robot_ScanCommand structure using Nanopb, validates that the
 * requested scan direction lies within the permitted bounds (@c _minHeadDir to
 * @c _maxHeadDir), triggers the scan routine, and sends a command acknowledgement
 * via @c sendCommandAck(). If decoding or validation fails, an error is logged and,
 * where a command ID is available, reported back via @c sendError().
 *
 * @param[in] topic   Null-terminated C-string specifying the command MQTT topic.
 * @param[in] payload Pointer to the raw byte buffer containing the serialised ScanCommand.
 * @param[in] length  Exact size of the payload buffer in bytes.
 *
 * @return @c true if the command was decoded, validated, and executed successfully;
 *         @c false otherwise.
 */
static void handleScCmd(const char* topic, byte* payload, unsigned int length) {
  wheelly_protobuf_robot_ScanCommand msg = wheelly_protobuf_robot_ScanCommand_init_zero;
  if (!decodeCommand<wheelly_protobuf_robot_ScanCommand>(topic, payload, length,
                                                         wheelly_protobuf_robot_ScanCommand_fields, msg)) {
    return;
  }

  if (!(msg.direction >= _minHeadDir && msg.direction <= _maxHeadDir)) {
    char err[96];
    snprintf(err, sizeof(err), "Direction (%d) must be >=%d and <=%d",
             msg.direction, _minHeadDir, _maxHeadDir);
    ESP_LOGE(TAG, "%s Error: %s", topic, err);
    sendError(topic, msg.id, err);
    return;
  }

  scan(msg.direction, millis());
  sendCommandAck(topic, msg.id);
}

/**
 * @brief Handles incoming head-tracking commands received via a messaging topic.
 * 
 * Decodes the raw binary payload into a Nanopb `TrackCommand` message. If decoding 
 * succeeds, it initiates head tracking towards the target coordinates, triggers a 
 * directional scan, and sends a command acknowledgement back on the specified topic.
 * 
 * @param topic   Null-terminated string representing the MQTT or messaging topic 
 *                on which the command was received.
 * @param payload Pointer to the raw byte buffer containing the Protobuf-encoded message.
 * @param length  Size of the payload buffer in bytes.
 * 
 * @return `true` if the payload was successfully decoded and the command was executed; 
 *         `false` if Protobuf decoding failed.
 */
static void handleHtCmd(const char* topic, byte* payload, unsigned int length) {
  wheelly_protobuf_robot_TrackCommand msg = wheelly_protobuf_robot_TrackCommand_init_zero;
  if (!decodeCommand<wheelly_protobuf_robot_TrackCommand>(topic, payload, length,
                                                          wheelly_protobuf_robot_TrackCommand_fields, msg)) {
    return;
  }

  headTrack(msg.frontTrack, msg.x, msg.y);

  sendCommandAck(topic, msg.id);
}

/**
 * @brief Handles incoming rotation commands received via a messaging topic.
 * 
 * Decodes the raw binary payload into a Nanopb `RotateCommand` message and validates
 * that the target direction falls within the permitted angular range (-180 to 179 degrees). 
 * If valid, it executes the rotation and dispatches a command acknowledgement; otherwise, 
 * it logs the failure and sends an error response.
 * 
 * @param topic   Null-terminated string representing the messaging topic on which 
 *                the command was received.
 * @param payload Pointer to the raw byte buffer containing the Protobuf-encoded message.
 * @param length  Size of the payload buffer in bytes.
 * 
 * @return `true` if the payload was successfully decoded, validated, and executed; 
 *         `false` if decoding failed or the direction value was out of range.
 */
static void handleRoCmd(const char* topic, byte* payload, unsigned int length) {
  wheelly_protobuf_robot_RotateCommand msg = wheelly_protobuf_robot_RotateCommand_init_zero;
  if (!decodeCommand<wheelly_protobuf_robot_RotateCommand>(topic, payload, length,
                                                           wheelly_protobuf_robot_RotateCommand_fields, msg)) {
    return;
  }

  if (!(msg.direction >= -180 && msg.direction <= 179)) {
    char err[60];
    snprintf(err, sizeof(err), "Invalid direction %d", msg.direction);
    sendError(topic, msg.id, err);
    ESP_LOGE(TAG, "%s Error: %s", topic, err);
    return;
  }

  rotate(msg.direction);

  sendCommandAck(topic, msg.id);
}

/**
 * @brief Handles incoming movement commands received via a messaging topic.
 * 
 * Decodes the raw binary payload into a Nanopb `MoveCommand` message. If decoding 
 * succeeds, it drives the robot either forward or backward towards the specified 
 * target coordinates depending on the `frontMove` flag, and dispatches a command 
 * acknowledgement back on the specified topic.
 * 
 * @param topic   Null-terminated string representing the messaging topic on which 
 *                the command was received.
 * @param payload Pointer to the raw byte buffer containing the Protobuf-encoded message.
 * @param length  Size of the payload buffer in bytes.
 * 
 * @return `true` if the payload was successfully decoded and the movement command 
 *         was executed; `false` if Protobuf decoding failed.
 */
static void handleMvCmd(const char* topic, byte* payload, unsigned int length) {
  wheelly_protobuf_robot_MoveCommand msg = wheelly_protobuf_robot_MoveCommand_init_zero;
  if (!decodeCommand<wheelly_protobuf_robot_MoveCommand>(topic, payload, length,
                                                         wheelly_protobuf_robot_MoveCommand_fields, msg)) {
    return;
  }

  if (msg.frontMove) {
    forward(msg.x, msg.y);
  } else {
    backward(msg.x, msg.y);
  }

  sendCommandAck(topic, msg.id);
}

/**
 * @brief Handles incoming reset commands received via a messaging topic.
 * 
 * Decodes the raw binary payload into a Nanopb `ResetCommand` message. If decoding 
 * succeeds, it resets the robot controller to its default idle state and dispatches 
 * a command acknowledgement back on the specified topic.
 * 
 * @param topic   Null-terminated string representing the messaging topic on which 
 *                the command was received.
 * @param payload Pointer to the raw byte buffer containing the Protobuf-encoded message.
 * @param length  Size of the payload buffer in bytes.
 * 
 * @return `true` if the payload was successfully decoded and the reset command 
 *         was executed; `false` if Protobuf decoding failed.
 */
static void handleRsCmd(const char* topic, byte* payload, unsigned int length) {
  wheelly_protobuf_robot_ResetCommand msg = wheelly_protobuf_robot_ResetCommand_init_zero;
  if (!decodeCommand<wheelly_protobuf_robot_ResetCommand>(topic, payload, length,
                                                          wheelly_protobuf_robot_ResetCommand_fields, msg)) {
    return;
  }

  reset();

  sendCommandAck(topic, msg.id);
}

/**
 * @brief Validates an integer configuration parameter against an inclusive range.
 * 
 * Checks whether the supplied configuration value lies within `[minValue, maxValue]`. 
 * If the value falls outside the permitted bounds, it formats a descriptive error 
 * message, logs it at error level, and dispatches an error response referencing the 
 * original command identifier.
 * 
 * @param topic    Null-terminated string representing the messaging topic associated 
 *                 with the configuration request.
 * @param id       Unique command identifier used to correlate the error response.
 * @param key      Null-terminated string naming the configuration parameter being validated.
 * @param value    Integer value to validate.
 * @param minValue Minimum permitted value (inclusive).
 * @param maxValue Maximum permitted value (inclusive).
 * 
 * @return `true` if `value` is within the valid range; `false` otherwise.
 */
static const bool validateCfg(const char* topic, const char* key, const uint64_t id, const int32_t value, const int32_t minValue, const int32_t maxValue) {
  if (!(value >= minValue && value <= maxValue)) {
    char msg[96];
    snprintf(msg, sizeof(msg), "%s (%d) must be >= %d and <= %d", key, value, minValue, maxValue);
    ESP_LOGE(TAG, "%s", msg);
    sendError(topic, id, msg);
    return false;
  }
  return true;
}

/**
 * @brief Validates an unsigned 32-bit integer configuration parameter against an inclusive range.
 * 
 * Checks whether the supplied configuration value lies within `[minValue, maxValue]`. 
 * If the value falls outside the permitted bounds, it formats a descriptive error 
 * message, logs it at error level, and dispatches an error response referencing the 
 * original command identifier.
 * 
 * @param topic    Null-terminated string representing the messaging topic associated 
 *                 with the configuration request.
 * @param id       Unique command identifier used to correlate the error response.
 * @param key      Null-terminated string naming the configuration parameter being validated.
 * @param value    Unsigned 32-bit integer value to validate.
 * @param minValue Minimum permitted value (inclusive).
 * @param maxValue Maximum permitted value (inclusive).
 * 
 * @return `true` if `value` is within the valid range; `false` otherwise.
 */
static const bool validateCfg(const char* topic, const char* key, const uint64_t id, const uint32_t value, const uint32_t minValue, const uint32_t maxValue) {
  if (!(value >= minValue && value <= maxValue)) {
    char msg[96];
    snprintf(msg, sizeof(msg), "%s (%d) must be >= %d and <= %d", key, value, minValue, maxValue);
    ESP_LOGE(TAG, "%s", msg);
    sendError(topic, id, msg);
    return false;
  }
  return true;
}

/**
 * @brief Handles incoming configuration commands received via a messaging topic.
 * 
 * Decodes the raw binary payload into a Nanopb `ConfigurationCommand` message and 
 * validates every configuration parameter against its permitted bounds. If all 
 * values are valid, it applies the updated motion, motor traction control (TCS), 
 * LiDAR, head, and telemetry parameters to the robot subsystems, and dispatches 
 * a command acknowledgement back on the specified topic.
 * 
 * @param topic   Null-terminated string representing the messaging topic on which 
 *                the command was received.
 * @param payload Pointer to the raw byte buffer containing the Protobuf-encoded message.
 * @param length  Size of the payload buffer in bytes.
 * 
 * @return `true` if the payload was successfully decoded, all parameters passed 
 *         validation, and the configuration was applied; `false` if decoding or 
 *         parameter validation failed.
 */
static void handleCfCmd(const char* topic, byte* payload, unsigned int length) {
  wheelly_protobuf_robot_ConfigurationCommand msg = wheelly_protobuf_robot_ConfigurationCommand_init_zero;
  if (!decodeCommand<wheelly_protobuf_robot_ConfigurationCommand>(topic, payload, length,
                                                                  wheelly_protobuf_robot_ConfigurationCommand_fields, msg)) {
    return;
  }
  // Validate configuration
  if (!msg.has_config) {
    sendError(topic, msg.id, "Missing config payload");
    return;
  }
  if (!(validateCfg(topic, "tau", msg.id, msg.config.tau, 1, 10000)
        && validateCfg(topic, "asr", msg.id, msg.config.asr, 1, 16383)
        && validateCfg(topic, "lambdaFactor", msg.id, msg.config.lambdaFactor, 0, 100)
        && validateCfg(topic, "maxPulseInterval", msg.id, msg.config.maxPulseInterval, 1, 10000)
        && validateCfg(topic, "delayedInterval", msg.id, msg.config.delayedInterval, 1, 10000)
        && validateCfg(topic, "minRotRange", msg.id, msg.config.minRotRange, 0, 180)
        && validateCfg(topic, "maxRotRange", msg.id, msg.config.maxRotRange, 0, 180)
        && validateCfg(topic, "maxRotPps", msg.id, msg.config.maxRotPps, 0, 20)
        && validateCfg(topic, "maxSpeed", msg.id, msg.config.maxSpeed, 0, 60)
        && validateCfg(topic, "haltDistance", msg.id, msg.config.haltDistance, 0, 1000)
        && validateCfg(topic, "decelerateDistance", msg.id, msg.config.decelerateDistance, 0, 1000)
        && validateCfg(topic, "sendInterval", msg.id, msg.config.sendInterval, 1, 60000)
        && validateCfg(topic, "scanInterval", msg.id, msg.config.scanInterval, 1, 60000)
        && validateCfg(topic, "minHeadDir", msg.id, msg.config.minHeadDir, -90, 90)
        && validateCfg(topic, "maxHeadDir", msg.id, msg.config.maxHeadDir, -90, 90)
        && validateCfg(topic, "antiGimbalRadius", msg.id, msg.config.antiGimbalRadius, 1, 1000))) {
    return;
  }
  if (msg.config.minHeadDir > msg.config.maxHeadDir) {
    char err[96];
    snprintf(err, sizeof(err), "minHeadDir (%d) must be <= maxHeadDir (%d)",
             msg.config.minHeadDir, msg.config.maxHeadDir);
    ESP_LOGE(TAG, "%s", err);
    sendError(topic, msg.id, err);
    return;
  }
  if (msg.config.minRotRange > msg.config.maxRotRange) {
    char err[96];
    snprintf(err, sizeof(err), "minRotRange (%d) must be <= maxRotRange (%d)",
             msg.config.minRotRange, msg.config.maxRotRange);
    ESP_LOGE(TAG, "%s", err);
    sendError(topic, msg.id, err);
    return;
  }

  ESP_LOGI(TAG, "               asr=%d", msg.config.asr);
  ESP_LOGI(TAG, "  maxPulseInterval=%d", msg.config.maxPulseInterval);
  ESP_LOGI(TAG, "      lambdaFactor=%d", msg.config.lambdaFactor);
  ESP_LOGI(TAG, "   delayedInterval=%d", msg.config.delayedInterval);
  ESP_LOGI(TAG, "               tau=%d", msg.config.tau);
  ESP_LOGI(TAG, "       minRotRange=%d", msg.config.minRotRange);
  ESP_LOGI(TAG, "       maxRotRange=%d", msg.config.maxRotRange);
  ESP_LOGI(TAG, "         maxRotPps=%d", msg.config.maxRotPps);
  ESP_LOGI(TAG, "          maxSpeed=%d", msg.config.maxSpeed);
  ESP_LOGI(TAG, "decelerateDistance=%d", msg.config.decelerateDistance);
  ESP_LOGI(TAG, "        minHeadDir=%d", msg.config.minHeadDir);
  ESP_LOGI(TAG, "        maxHeadDir=%d", msg.config.maxHeadDir);
  ESP_LOGI(TAG, "      sendInterval=%d", msg.config.sendInterval);
  ESP_LOGI(TAG, "      scanInterval=%d", msg.config.scanInterval);
  ESP_LOGI(TAG, "  antiGimbalRadius=%d", msg.config.antiGimbalRadius);

  // Applies changed configuration
  const motionConfig_t cfg = {
    .minRotRange = msg.config.minRotRange,
    .maxRotRange = msg.config.maxRotRange,
    .maxRotPps = msg.config.maxRotPps,
    .maxSpeed = msg.config.maxSpeed,
    .haltDistance = msg.config.haltDistance,
    .decelerateDistance = msg.config.decelerateDistance
  };
  _motionCtrl.config(cfg);

  const tcsParams_t tcs = {
    .asr = msg.config.asr,
    .maxPulseInterval = msg.config.maxPulseInterval,
    .lambdaFactor = msg.config.lambdaFactor,
    .delayedInterval = msg.config.delayedInterval
  };
  _motionCtrl.leftMotor().tcs(tcs);
  _motionCtrl.rightMotor().tcs(tcs);

  _motionCtrl.tau(msg.config.tau);
  _minHeadDir = msg.config.minHeadDir;
  _maxHeadDir = msg.config.maxHeadDir;
  _sendInterval = msg.config.sendInterval;
  _lidar.interval(msg.config.scanInterval);
  _antiGimbalRadius = msg.config.antiGimbalRadius;

  sendCommandAck(topic, msg.id);
}

/**
 * @brief Handles incoming query-configuration commands received via a messaging topic.
 * 
 * Decodes the raw binary payload into a Nanopb `QueryConfigurationCommand` message. 
 * If decoding succeeds, it gathers the active motion, motor traction control (TCS), 
 * LiDAR, head, and telemetry parameters from the robot subsystems, encodes them into 
 * a `ConfigurationResponse` Protobuf message, and dispatches the serialised payload 
 * back on the specified topic.
 * 
 * @param topic   Null-terminated string representing the messaging topic on which 
 *                the query command was received.
 * @param payload Pointer to the raw byte buffer containing the Protobuf-encoded request.
 * @param length  Size of the payload buffer in bytes.
 * 
 * @return `true` if the request was successfully decoded and the configuration response 
 *         was encoded and sent; `false` if Protobuf decoding or encoding failed.
 */
static void handleQcCmd(const char* topic, byte* payload, unsigned int length) {
  wheelly_protobuf_robot_QueryConfigurationCommand msg = wheelly_protobuf_robot_QueryConfigurationCommand_init_zero;
  if (!decodeCommand<wheelly_protobuf_robot_QueryConfigurationCommand>(topic, payload, length,
                                                                       wheelly_protobuf_robot_QueryConfigurationCommand_fields, msg)) {
    return;
  }

  const motionConfig_t& cfg = _motionCtrl.config();
  const tcsParams_t& motorCfg = _motionCtrl.leftMotor().tcs();
  wheelly_protobuf_robot_ConfigurationResponse response = {
    .id = msg.id,
    .has_config = true,
    .config = {
      .asr = motorCfg.asr,
      .maxPulseInterval = motorCfg.maxPulseInterval,
      .lambdaFactor = motorCfg.lambdaFactor,
      .delayedInterval = motorCfg.delayedInterval,
      .tau = _motionCtrl.tau(),
      .minRotRange = cfg.minRotRange,
      .maxRotRange = cfg.maxRotRange,
      .maxRotPps = cfg.maxRotPps,
      .maxSpeed = cfg.maxSpeed,
      .haltDistance = cfg.haltDistance,
      .decelerateDistance = cfg.decelerateDistance,
      .minHeadDir = _minHeadDir,
      .maxHeadDir = _maxHeadDir,
      .sendInterval = _sendInterval,
      .scanInterval = _lidar.interval(),
      .antiGimbalRadius = _antiGimbalRadius }
  };

  uint8_t buffer[128];  // Buffer per i byte serializzati
  pb_ostream_t streamOut = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Encode message
  bool status = pb_encode(&streamOut, wheelly_protobuf_robot_CommandAck_fields, &msg);
  if (!status) {
    ESP_LOGE(TAG, "Encode error %s", PB_GET_ERROR(&streamOut));
    return;
  }
  size_t len = streamOut.bytes_written;
  sendResponse(topic, buffer, len);
}

/**
 * @brief Handles incoming query-status commands received via a messaging topic.
 * 
 * Decodes the raw binary payload into a Nanopb `QueryStatusCommand` message. If 
 * decoding succeeds, it triggers the status query routine to gather and publish 
 * the robot's current operational state, and dispatches a command acknowledgement 
 * back on the specified topic.
 * 
 * @param topic   Null-terminated string representing the messaging topic on which 
 *                the command was received.
 * @param payload Pointer to the raw byte buffer containing the Protobuf-encoded message.
 * @param length  Size of the payload buffer in bytes.
 * 
 * @return `true` if the payload was successfully decoded and the status query command 
 *         was executed; `false` if Protobuf decoding failed.
 */
static void handleQsCmd(const char* topic, byte* payload, unsigned int length) {
  wheelly_protobuf_robot_QueryStatusCommand msg = wheelly_protobuf_robot_QueryStatusCommand_init_zero;
  if (!decodeCommand<wheelly_protobuf_robot_QueryStatusCommand>(topic, payload, length,
                                                                wheelly_protobuf_robot_QueryStatusCommand_fields, msg)) {
    return;
  }

  queryStatus();

  sendCommandAck(topic, msg.id);
}

/**
 * @brief Handles incoming firmware version query commands received via a messaging topic.
 * 
 * Decodes the raw binary payload into a Nanopb `VersionCommand` message. If decoding 
 * succeeds, it populates a `VersionResponse` message with the command identifier and 
 * the current `WHEELLY_VERSION` string, serialises it into a binary buffer, and 
 * dispatches the response back on the specified topic.
 * 
 * @param topic   Null-terminated string representing the messaging topic on which 
 *                the command was received.
 * @param payload Pointer to the raw byte buffer containing the Protobuf-encoded message.
 * @param length  Size of the payload buffer in bytes.
 * 
 * @return `true` if the command was successfully decoded and the version response 
 *         was encoded and sent; `false` if decoding or encoding failed.
 */
static void handleVrCmd(const char* topic, byte* payload, unsigned int length) {
  wheelly_protobuf_robot_VersionCommand msg = wheelly_protobuf_robot_VersionCommand_init_zero;
  if (!decodeCommand<wheelly_protobuf_robot_VersionCommand>(topic, payload, length,
                                                            wheelly_protobuf_robot_VersionCommand_fields, msg)) {
    return;
  }

  wheelly_protobuf_robot_VersionResponse response = {
    .id = msg.id
  };

  snprintf(response.version, sizeof(response.version), "%s", WHEELLY_VERSION);

  uint8_t buffer[64];  // Buffer per i byte serializzati
  pb_ostream_t streamOut = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Encode message
  bool status = pb_encode(&streamOut, wheelly_protobuf_robot_VersionResponse_fields, &msg);
  if (!status) {
    ESP_LOGE(TAG, "Encode error %s", PB_GET_ERROR(&streamOut));
    return;
  }
  size_t len = streamOut.bytes_written;
  sendResponse(topic, buffer, len);
}

/**
 * @brief Checks whether a null-terminated C-string ends with a specified suffix.
 *
 * This helper compares the trailing characters of @p str against @p suffix without
 * allocating any dynamic memory. If either pointer is null, or if @p suffix is longer
 * than @p str, the function safely returns @c false.
 *
 * @param[in] str    Null-terminated C-string to inspect.
 * @param[in] suffix Null-terminated C-string representing the expected suffix.
 *
 * @return @c true if @p str ends with @p suffix; @c false otherwise.
 */
static bool endsWith(const char* str, const char* suffix) {
  if (!str || !suffix) { return false; }
  size_t strLen = strlen(str);
  size_t suffixLen = strlen(suffix);
  if (suffixLen > strLen) { return false; }
  return strcmp(str + (strLen - suffixLen), suffix) == 0;
}


/**
 * @brief Routes incoming MQTT command messages to their corresponding handlers.
 * 
 * Iterates through the static routing table to match the suffix of the incoming 
 * MQTT topic against supported command endpoints (such as halt, scan, head-track, 
 * rotate, move, configuration, query, reset, and version). When a matching route 
 * is found, it invokes the associated handler and returns immediately. If no route 
 * matches the topic suffix, it logs an unrecognised command error and dispatches 
 * an error response.
 * 
 * @param topicArg Null-terminated string representing the full MQTT topic on which 
 *                 the message was received.
 * @param payload  Pointer to the raw byte buffer containing the Protobuf-encoded message.
 * @param length   Size of the payload buffer in bytes.
 */
static void handleMqttMessage(const char* topicArg, byte* payload, unsigned int length) {
  static command_route_t routes[] = {
    { "/ha", handleHaCmd },
    { "/sc", handleScCmd },
    { "/ht", handleHtCmd },
    { "/ro", handleRoCmd },
    { "/mv", handleMvCmd },
    { "/cf", handleCfCmd },
    { "/qc", handleQcCmd },
    { "/qs", handleQsCmd },
    { "/rs", handleRsCmd },
    { "/vr", handleVrCmd }
  };

  for (const auto& route : routes) {
    if (endsWith(topicArg, route.suffix)) {
      route.handler(topicArg, payload, length);
      return;
    }
  }
  const char* err = "Unrecognized command";
  sendError(topicArg, 0, err);
  ESP_LOGE(TAG, "%s Error: %s", topicArg, err);
}

/**
 * @brief Constructs a new `Wheelly` robot controller instance and initialises device identifiers.
 * 
 * Sets the default operational parameters for telemetry transmission intervals, 
 * head servo angular limits, and the anti-gimbal radius. Reads the hardware eFuse 
 * MAC address from the ESP32, reverses its 6-byte order into standard big-endian 
 * representation, formats it as a hexadecimal device identifier (`_id`), and 
 * builds the MQTT sensor publication prefix and command subscription wildcard topics.
 */
Wheelly::Wheelly() {
  _sendInterval = DEFAULT_SEND_INTERVAL;
  _minHeadDir = -SERVO_DIRECTION_LIMIT;
  _maxHeadDir = SERVO_DIRECTION_LIMIT;
  _antiGimbalRadius = DEFAULT_ANTI_GIMBAL_RADIUS;

  // Computes device id from mac address
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(_id, sizeof(_id), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  snprintf(_pubSensorTopicPrefix, sizeof(_pubSensorTopicPrefix), "sens/wheelly/%s/" WHEELLY_MESSAGES_VERSION, _id);
  snprintf(_subCommandTopics, sizeof(_subCommandTopics), "cmd/wheelly/%s/" WHEELLY_MESSAGES_VERSION "/+", _id);
}

/**
    * Returns the wheelly id (mac address)
    */
const char* Wheelly::id(void) const {
  return _id;
}

/**
    * Returns the command subscription topics
    */
const char* Wheelly::subCommandTopics(void) const {
  return _subCommandTopics;
}


/**
  * Returns true if head is traking a target
  */
const HeadStatus Wheelly::headStatus(void) const {
  return _headStatus;
}


/*
  * Returns the x head target (pulses)
  */
const float Wheelly::xHeadTarget(void) const {
  return _xHeadTarget;
}


/*
  * Returns the y head target (pulses)
  */
const float Wheelly::yHeadTarget(void) const {
  return _yHeadTarget;
}

/**
  * Returns the head anti gimbal radius (pulses)
  */
const int Wheelly::antiGimbalRadius(void) const {
  return _antiGimbalRadius;
}


void Wheelly::connected(const boolean state) {
  _display.connected(state);
}

/**
       Displays the message
       @param message the message
    */
void Wheelly::display(const char* message) {
  _display.showWiFiInfo(message);
}

/**
       Sets activity
    */
void Wheelly::activity(const unsigned long t0) {
  _display.activity(t0);
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
  _ledTimer.onNext(handleLed);
  _ledTimer.interval(SLOW_LED_INTERVAL);
  _ledTimer.continuous(true);
  _ledTimer.start();

  /* Setup supplier sensor */
  pinMode(VOLTAGE_PIN, INPUT);

  /* Setup statistic timer */
  _statsTimer.onNext(handleStats);
  _statsTimer.interval(STATS_INTERVAL);
  _statsTimer.continuous(true);
  _statsTimer.start();
  _statsTime = millis();

  /* Setup lidar servo */
  _servo.offset(SERVO_OFFSET);
  _servo.begin();

  // Setup head tracking timer
  _headTrackingTimer.onNext(trackingHead);

  _headTrackingTimer.interval(HEAD_TRACKING_INTERVAL);
  _headTrackingTimer.continuous(true);
  _headTrackingTimer.start();

  // Setup head command timer
  _headCmdTimer.onNext(handleHeadTimeout);
  _headCmdTimer.interval(HEAD_COMMAND_TIMEOUT);
  _headCmdTimer.continuous(false);

  // Setup lidar
  _lidar.interval(DEFAULT_SCAN_INTERVAL);
  _lidar.onRange(handleLidarRange);
  _lidar.begin();

  /* Setup contact sensors */
  _contactSensors.onChanged(handleChangedContacts);
  _contactSensors.begin();

  /* Setup mpu */
  _display.clear();
  _display.print("Init Mpu...");

  /* Setup Motion controller */
  _motionCtrl.begin();

  /* Setup mpu */
  _mpu.onData(handleMpuData);
  _mpu.onError([](void* context, const char* error) {
    sendMotion();
  });

  _mpu.begin();
  _display.clear();
  _display.print("Calibrating Mpu...");
  _mpu.calibrate();
  _mpuTimeout = millis() + MPU_INTERVAL;

  mqttClient.onMessage(handleMqttMessage);


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

/**
 * @brief Updates the online connectivity state of the robot controller.
 * 
 * Sets the internal `_onLine` flag to reflect whether the messaging connection 
 * is active. When transitioning to an online state (`true`), it immediately 
 * transmits a `"hi"` presence message to announce availability.
 * 
 * @param onLine Connection status flag: `true` if the controller is connected and 
 *               online; `false` otherwise.
 */
void Wheelly::onLine(boolean onLine) {
  _onLine = onLine;
  if (onLine) {
    sendHi();
  }
}
