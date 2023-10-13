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
static const unsigned long LED_INTERVAL = 200;

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
static const unsigned long STOP_ECHO_TIME = (20ul * 5887 / 100);
static const unsigned long MAX_ECHO_TIME = (400ul * 5887 / 100);
static const int DISTANCE_TICK = 5;

/*
   Proxy sensor servo
*/
static const unsigned long DEFAULT_SCAN_INTERVAL = 1000ul;
static const unsigned long SCANNER_RESET_INTERVAL = 1000ul;
static const int NO_SCAN_DIRECTIONS = 10;
static const int SERVO_OFFSET = 4;
static const int FRONT_DIRECTION = 90;

/*
   Creates wheelly controller
*/
Wheelly::Wheelly() :
  _motionCtrl(LEFT_FORW_PIN, LEFT_BACK_PIN, RIGHT_FORW_PIN, RIGHT_BACK_PIN, LEFT_PIN, RIGHT_PIN),
  _scanInterval(DEFAULT_SCAN_INTERVAL),
  _sendInterval(DEFAULT_SEND_INTERVAL),
  _sr04(TRIGGER_PIN, ECHO_PIN),
  _contactSensors(FRONT_CONTACTS_PIN, REAR_CONTACTS_PIN) {
}

/*
   Initializes wheelly controller
   Returns true if successfully initialized
*/
boolean Wheelly::begin(void) {
  // Setup display
  _display.begin();

  // Setup error led
  pinMode(STATUS_LED_PIN, OUTPUT);
  _ledTimer.onNext([](void *context, const unsigned long n) {
    ((Wheelly*)context)->handleLed(n);
  }, this);
  _ledTimer.interval(LED_INTERVAL);
  _ledTimer.continuous(true);
  _ledTimer.start();

  // Setup supplier sensor
  pinMode(VOLTAGE_PIN, INPUT);

  // Setup statistic timer
  _statsTimer.onNext([](void *context, const unsigned long) {
    ((Wheelly*)context)->handleStats();
  }, this);
  _statsTimer.interval(STATS_INTERVAL);
  _statsTimer.continuous(true);
  _statsTimer.start();
  _statsTime = millis();

  // Setup send timer
  _sendTimer.onNext([](void *context, const unsigned long) {
    ((Wheelly*)context)->sendStatus();
  }, this);
  _sendTimer.interval(_sendInterval);
  _sendTimer.continuous(true);
  _sendTimer.start();

  // Setup SR04 proximity sensor
  _sr04.begin();
  //  SR04.noSamples(NO_SAMPLES);
  _sr04.onSample([](void *context, const unsigned long time) {
    ((Wheelly*)context)->handleEchoSample(time);
  }, this);

  // Setup proxy servo
  _scanTimer.onNext([](void *context, const unsigned long) {
    ((Wheelly*)context)->handleAutoScan();
  }, this);
  _scanTimer.interval(_scanInterval);
  _proxyServo.attach(SERVO_PIN);
  _proxyServo.offset(SERVO_OFFSET);
  _proxyServo.onReached([](void *context, const int) {
    ((Wheelly*)context)->handleServoReached();
  }, this);
  _proxyServo.angle(FRONT_DIRECTION);

  // Setup contact sensors
  _contactSensors.onChanged([](void *context, ContactSensors & sensors) {
    ((Wheelly*)context)->handleChangedContacts();
  }, this);
  _contactSensors.begin();

  // Setup mpu
  _display.clear();
  _display.print("Init Mpu...");

  // Setup Motion controller
  _motionCtrl.begin();

  // Setup mpu
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

  return true;
}

/*
   Pools wheelly controller
*/
void Wheelly::polling(const unsigned long t0) {
  // Increments cps counter
  _counter++;

  _scanTimer.polling(t0);
  _proxyServo.polling(t0);
  _sr04.polling(t0);
  _statsTimer.polling(t0);

  _contactSensors.polling(t0);

  _mpu.polling(t0);
  _mpuError = _mpu.rc();
  if (t0 >= _mpuTimeout) {
    _mpuError = MPU_TIMEOUT_ERROR;
    DEBUG_PRINTLN("!! mpu timeout");
  }

  _ledActive = _mpuError != 0 || (!canMoveForward() && !canMoveBackward());
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
  _sendInterval = intervals[0];
  _scanInterval = intervals[1];
  _sendTimer.interval(_sendInterval);
  _scanTimer.interval(_scanInterval);
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

  if (_lastYaw != _yaw) {
    sendStatus();
  }
}

/*
   Handles servo auto scan
*/
void Wheelly::handleAutoScan(void) {
  DEBUG_PRINTLN(F("//Wheelly::handleAutoScan"));
  _proxyServo.angle(_nextScan);
}

/**
   Handles changed contacts
*/
void Wheelly::handleChangedContacts(void) {
  DEBUG_PRINTLN("//Wheelly::handleChangedContacts");
  sendStatus();
}

/**
   Handles position reached event from scan servo
*/
void Wheelly::handleServoReached(void) {
  DEBUG_PRINTLN(F("//Wheelly::handleServoReached"));
  _sr04.start();
}

/*
   Handles sample event from distance sensor
*/
void Wheelly::handleEchoSample(const unsigned long echoTime) {
  _distanceTime = echoTime;
  DEBUG_PRINT("//Wheelly::handleEchoSample ");
  DEBUG_PRINTLN(echoTime);
  if (_motionCtrl.isForward() && !canMoveForward()
      || _motionCtrl.isBackward() && !canMoveBackward()) {
    // Halt robot if cannot move
    _motionCtrl.halt();
  }
  sendStatus();
  const unsigned long now = millis();
  if (now >= _resetTime) {
    _nextScan = FRONT_DIRECTION;
  }

  if (_distanceTime == 0) {
    _display.distance(-1);
  } else {
    int distance = (int)(echoTime * 100 / 5887);
    distance = (2 * distance + DISTANCE_TICK) / DISTANCE_TICK / 2;
    distance *= DISTANCE_TICK;
    _display.distance(distance);
  }
  _scanTimer.start();
}

/*
   Scans proximity to the direction
   @param angle the angle (DEG)
   @param t0 the scanning instant
*/
void Wheelly::scan(const int angle, const unsigned long t0) {
  _nextScan = 90 - angle;
  _resetTime = t0 + SCANNER_RESET_INTERVAL;
  _scanTimer.stop();
  _proxyServo.angle(_nextScan);
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
  digitalWrite(STATUS_LED_PIN, _ledActive && (n % 2) == 0);
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
  char bfr[256];
  _lastYaw = _yaw;
  // st time x y yaw servo distance lpps rpps frontSig rearSig volt canF canB err halt dir speed nextScan
  sprintf(bfr, "st %ld %.1f %.1f %d %d %ld %.1f %.1f %d %d %d %d %d %d %d %d %d %d",
          millis(),
          (double)_motionCtrl.xPulses(),
          (double)_motionCtrl.yPulses(),
          _yaw,
          90 - _proxyServo.angle(),
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
          (90 - _nextScan));
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
