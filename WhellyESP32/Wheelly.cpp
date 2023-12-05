#include "Wheelly.h"
#include "pins.h"

//#define DEBUG
#include "debug.h"

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
static const unsigned long STOP_ECHO_TIME = (20ul * 5887 / 100); // 20 cm
static const unsigned long MAX_ECHO_TIME = (400ul * 5887 / 100); // 400 cm
static const unsigned long MIN_ECHO_TIME = (3ul * 5887 / 100); // 3 cm
static const int DISTANCE_TICK = 5;

/*
   Proxy sensor servo
*/
static const unsigned long DEFAULT_SCAN_INTERVAL = 1000ul;
static const unsigned long SCANNER_RESET_INTERVAL = 1000ul;
static const int NO_SCAN_DIRECTIONS = 10;
static const int SERVO_OFFSET = -4;
static const int FRONT_DIRECTION = 90;

/*
   Voltage levels
*/
static const int MIN_VOLTAGE_VALUE = 1896;
static const int MAX_VOLTAGE_VALUE = 2621;

/*
   Creates wheelly controller
*/
Wheelly::Wheelly() :
  _motionCtrl(LEFT_FORW_PIN, LEFT_BACK_PIN, RIGHT_FORW_PIN, RIGHT_BACK_PIN, LEFT_PIN, RIGHT_PIN),
  _sendInterval(DEFAULT_SEND_INTERVAL),
  _contactSensors(FRONT_CONTACTS_PIN, REAR_CONTACTS_PIN),
  _proxySensor(SERVO_PIN, TRIGGER_PIN, ECHO_PIN) {
}

/*
   Initializes wheelly controller
   Returns true if successfully initialized
*/
boolean Wheelly::begin(void) {
  /* Setup display */
  _display.begin();

  /* Setup error led */
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, true);
  _ledTimer.onNext([](void *context, const unsigned long n) {
    ((Wheelly*)context)->handleLed(n);
  }, this);
  _ledTimer.interval(SLOW_LED_INTERVAL);
  _ledTimer.continuous(true);
  _ledTimer.start();

  /* Setup supplier sensor */
  pinMode(VOLTAGE_PIN, INPUT);

  /* Setup statistic timer */
  _statsTimer.onNext([](void *context, const unsigned long) {
    ((Wheelly*)context)->handleStats();
  }, this);
  _statsTimer.interval(STATS_INTERVAL);
  _statsTimer.continuous(true);
  _statsTimer.start();
  _statsTime = millis();

  /* Setup send timer */
  _sendTimer.onNext([](void *context, const unsigned long) {
    ((Wheelly*)context)->sendStatus();
  }, this);
  _sendTimer.interval(_sendInterval);
  _sendTimer.continuous(true);
  _sendTimer.start();

  /* Setup proxy sensor */
  _proxySensor.onDataReady([](void *context, ProxySensor & sensor) {
    ((Wheelly*)context)->handleProxyData();
  }, this);
  _proxySensor.interval(_sendInterval);
  _proxySensor.offset(SERVO_OFFSET);
  _proxySensor.begin();

  /* Setup contact sensors */
  _contactSensors.onChanged([](void *context, ContactSensors & sensors) {
    ((Wheelly*)context)->handleChangedContacts();
  }, this);
  _contactSensors.begin();

  /* Setup mpu */
  _display.clear();
  _display.print("Init Mpu...");

  /* Setup Motion controller */
  _motionCtrl.begin();

  /* Setup mpu */
  _mpu.onData([](void *context) {
    ((Wheelly*)context)->handleMpuData();
  }, this);
  _mpu.onError([](void *context, const char* error) {
    ((Wheelly*)context)->sendReply(error);
  }, this);

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

  _proxySensor.polling(t0);
  _statsTimer.polling(t0);

  _contactSensors.polling(t0);

  _mpu.polling(t0);
  _mpuError = _mpu.rc();
  if (t0 >= _mpuTimeout) {
    _mpuError = MPU_TIMEOUT_ERROR;
    DEBUG_PRINTLN("!! mpu timeout");
  }

  _ledActive = _mpuError != 0 || !canMoveForward() || !canMoveBackward() || !_onLine;
  _ledTimer.interval(
    _mpuError != 0 || (!canMoveForward() && !canMoveBackward()) || !_onLine
    ? FAST_LED_INTERVAL : SLOW_LED_INTERVAL);
  _ledTimer.polling(t0);

  _motionCtrl.polling(t0);
  _sendTimer.polling(t0);

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
void Wheelly::configIntervals(const int *intervals) {
  _sendTimer.interval(intervals[0]);
  _proxySensor.interval(intervals[1]);
}

/*
   Resets wheelly
*/
void Wheelly::reset() {
  _motionCtrl.halt();
  _motionCtrl.reset(millis());
  _mpuError = 0;
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
  DEBUG_PRINTLN("//Wheelly::handleChangedContacts");
  sendStatus();
}

/*
   Handles sample event from distance sensor
*/
void Wheelly::handleProxyData() {
  const unsigned long echoTime = _proxySensor.echoDelay();
  _distanceTime = echoTime > MIN_ECHO_TIME ? echoTime : 0;
  DEBUG_PRINT("//Wheelly::handleEchoSample ");
  DEBUG_PRINTLN(echoTime);
  if (_motionCtrl.isForward() && !canMoveForward()
      || _motionCtrl.isBackward() && !canMoveBackward()) {
    /* Halt robot if cannot move */
    _motionCtrl.halt();
  }
  sendStatus();

  if (_distanceTime == 0) {
    _display.distance(-1);
  } else {
    int distance = (int)(echoTime * 100 / 5887);
    distance = (2 * distance + DISTANCE_TICK) / DISTANCE_TICK / 2;
    distance *= DISTANCE_TICK;
    _display.distance(distance);
  }
}

/*
   Scans proximity to the direction
   @param angle the angle (DEG)
   @param t0 the scanning instant
*/
void Wheelly::scan(const int angle, const unsigned long t0) {
  _proxySensor.direction(angle, t0);
}

/*
  Handles timeout event from statistics timer
*/
void Wheelly::handleStats() {
  const unsigned long t0 = millis();
  const unsigned long dt = (t0 - _statsTime);
  const unsigned long tps = _counter * 1000 / dt;
  _statsTime = t0;
  _counter = 0;
  char bfr[256];
  sprintf(bfr, "cs %ld %ld", t0, tps);
  sendReply(bfr);
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
void Wheelly::sendReply(const char* data) {
  if (_onReply) {
    _onReply(_context, data);
  }
}

/*
   Sends the status of wheelly
*/
void Wheelly::sendStatus() {
  const int voltageValue = analogRead(VOLTAGE_PIN);
  const int v = voltageValue;
  const int hLevel = min(max(
                           (int)map(v, MIN_VOLTAGE_VALUE, MAX_VOLTAGE_VALUE, 0, 10),
                           0), 9);
  const int level = (hLevel + 1) / 2;
  DEBUG_PRINT("// Wheelly::sendStatus v=");
  DEBUG_PRINT(v);
  DEBUG_PRINT(" , hlevel=");
  DEBUG_PRINT(hLevel);
  DEBUG_PRINT(" , level=");
  DEBUG_PRINT(level);
  DEBUG_PRINTLN();
  _display.supply(level);

  char bfr[256];
  /* st time x y yaw servo distance lpps rpps frontSig rearSig volt canF canB err halt dir speed nextScan */
  sprintf(bfr, "st %ld %.1f %.1f %d %d %ld %.1f %.1f %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
          millis(),
          (double)_motionCtrl.xPulses(),
          (double)_motionCtrl.yPulses(),
          _yaw,
          _proxySensor.echoDirection(),
          _distanceTime,
          (double)_motionCtrl.leftPps(),
          (double)_motionCtrl.rightPps(),
          _contactSensors.frontClear(),
          _contactSensors.rearClear(),
          voltageValue,
          canMoveForward(),
          canMoveBackward(),
          (const unsigned short)_mpuError,
          _motionCtrl.isHalt() ? 1 : 0,
          _motionCtrl.direction(),
          _motionCtrl.speed(),
          _proxySensor.direction(),
          _motionCtrl.leftMotor().speed(),
          _motionCtrl.rightMotor().speed(),
          _motionCtrl.leftMotor().power(),
          _motionCtrl.rightMotor().power()
         );
  sendReply(bfr);
  _sendTimer.restart();
}

/*
   Returns true if can move forward
*/
const boolean Wheelly::canMoveForward() const {
  return forwardBlockDistanceTime() > STOP_ECHO_TIME && _contactSensors.frontClear();
}

/*
   Returns true if can move backward
*/
const boolean Wheelly::canMoveBackward() const {
  return _contactSensors.rearClear();
}

/*
   Returns the block echo time
*/
const int Wheelly::forwardBlockDistanceTime() const {
  return (_distanceTime > 0 && _distanceTime <= MAX_ECHO_TIME) ? _distanceTime : MAX_ECHO_TIME;
}
