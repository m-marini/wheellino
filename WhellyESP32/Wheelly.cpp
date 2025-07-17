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

//#define DEBUG
#include "debug.h"

/*
   Current version
*/
static const char version[] = "0.8.1";

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
static const unsigned long STOP_ECHO_TIME = (20ul * 5887 / 100);  // 20 cm
static const unsigned long MAX_ECHO_TIME = (400ul * 5887 / 100);  // 400 cm
static const unsigned long MIN_ECHO_TIME = (3ul * 5887 / 100);    // 3 cm
static const int DISTANCE_TICK = 5;

/*
   Proxy sensor servo
*/
static const unsigned long DEFAULT_SCAN_INTERVAL = 1000ul;
static const unsigned long SCANNER_RESET_INTERVAL = 1000ul;
static const int NO_SCAN_DIRECTIONS = 10;
static const int SERVO_OFFSET = -4;

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
    _proxySensor(SERVO_PIN, TRIGGER_PIN, ECHO_PIN) {

  _commandInterpreter.onError([](void* ctx, const char* msg) {
    ((Wheelly*)ctx)->sendReply(msg);
  },
                              (void*)this);

  /* ha command */
  _commandInterpreter.addCommand(
    "ha", [](void* ctx, const unsigned long, const char* msg) {
      ((Wheelly*)ctx)->halt();
    },
    (void*)this);

  /* qs command */
  _commandInterpreter.addCommand(
    "qs", [](void* ctx, const unsigned long, const char* msg) {
      ((Wheelly*)ctx)->queryStatus();
    },
    (void*)this);

  /* rs command */
  _commandInterpreter.addCommand(
    "rs", [](void* ctx, const unsigned long, const char* msg) {
      ((Wheelly*)ctx)->reset();
    },
    (void*)this);

  /* qc command */
  _commandInterpreter.addCommand(
    "qc", [](void* ctx, const unsigned long, const char* msg) {
      ((Wheelly*)ctx)->queryConfig();
    },
    (void*)this);

  /* vr command */
  _commandInterpreter.addCommand(
    "vr", [](void* ctx, const unsigned long, const char* msg) {
      // vr command
      char bfr[256];
      strcpy(bfr, "// vr ");
      strcat(bfr, version);
      ((Wheelly*)ctx)->sendReply(bfr);
    },
    (void*)this);

  /* sc command */
  _commandInterpreter.addIntCommand(
    "sc", [](void* ctx, const unsigned long, const char*, const int* argv) {
      ((Wheelly*)ctx)->scan(argv[0]);
    },
    (void*)this, 1, -90, 90);

  /* mv command */
  _commandInterpreter.addIntCommand(
    "mv", [](void* ctx, const unsigned long, const char*, const int* argv) {
      ((Wheelly*)ctx)->move(argv[0], argv[1]);
    },
    (void*)this, 2, -180, 179, -MAX_SPEED, MAX_SPEED);

  /* cc command */
  _commandInterpreter.addIntCommand(
    "cc", [](void* ctx, const unsigned long, const char* cmd, const int* argv) {
      ((Wheelly*)ctx)->configMotionController(argv);
      char bfr[256];
      strcpy(bfr, "// ");
      strcat(bfr, cmd);
      ((Wheelly*)ctx)->sendReply(bfr);
    },
    (void*)this, 3, 0, 180, 0, 180, 0, 20);

  /* cs command */
  _commandInterpreter.addIntCommand(
    "cs", [](void* ctx, const unsigned long, const char* cmd, const int* argv) {
      ((Wheelly*)ctx)->configMotorSensors(argv[0]);
      char bfr[256];
      strcpy(bfr, "// ");
      strcat(bfr, cmd);
      ((Wheelly*)ctx)->sendReply(bfr);
    },
    (void*)this, 1, 1, 10000);

  /* ci command */
  _commandInterpreter.addIntCommand(
    "ci", [](void* ctx, const unsigned long, const char* cmd, const int* argv) {
      ((Wheelly*)ctx)->configIntervals(argv);
      char bfr[256];
      strcpy(bfr, "// ");
      strcat(bfr, cmd);
      ((Wheelly*)ctx)->sendReply(bfr);
    },
    (void*)this, 2, 1, 60000, 1, 60000);

  /* tcsl command */
  _commandInterpreter.addIntCommand(
    "tcsl", [](void* ctx, const unsigned long, const char* cmd, const int* argv) {
      ((Wheelly*)ctx)->configTcsMotorController(argv, true);
      char bfr[256];
      strcpy(bfr, "// ");
      strcat(bfr, cmd);
      ((Wheelly*)ctx)->sendReply(bfr);
    },
    (void*)this, 8, 0, 1000, 0, 1000, 0, 1000, -1000, 0, -1000, 0, -1000, 0, 0, 16383, 0, 100);

  /* tcsr command */
  _commandInterpreter.addIntCommand(
    "tcsr", [](void* ctx, const unsigned long, const char* cmd, const int* argv) {
      ((Wheelly*)ctx)->configTcsMotorController(argv, false);
      char bfr[256];
      strcpy(bfr, "// ");
      strcat(bfr, cmd);
      ((Wheelly*)ctx)->sendReply(bfr);
    },
    (void*)this, 8, 0, 1000, 0, 1000, 0, 1000, -1000, 0, -1000, 0, -1000, 0, 0, 16383, 0, 100);

  /* fl command */
  _commandInterpreter.addLongCommand(
    "fl", [](void* ctx, const unsigned long, const char* cmd, const long* argv) {
      ((Wheelly*)ctx)->configFeedbackMotorController(argv, true);
      char bfr[256];
      strcpy(bfr, "// ");
      strcat(bfr, cmd);
      ((Wheelly*)ctx)->sendReply(bfr);
    },
    (void*)this, 2, 0L, 2000000L, 0L, 2000000L);

  /* fr command */
  _commandInterpreter.addLongCommand(
    "fr", [](void* ctx, const unsigned long, const char* cmd, const long* argv) {
      ((Wheelly*)ctx)->configFeedbackMotorController(argv, false);
      char bfr[256];
      strcpy(bfr, "// ");
      strcat(bfr, cmd);
      ((Wheelly*)ctx)->sendReply(bfr);
    },
    (void*)this, 2, 0L, 2000000L, 0L, 2000000L);

  /* ck command */
  _commandInterpreter.addStrCommand(
    "ck", [](void* ctx, const unsigned long t0, const char* cmd) {
      char bfr[256];
      sprintf(bfr, "%s %ld %ld", cmd, t0, millis());
      ((Wheelly*)ctx)->sendReply(bfr);
      DEBUG_PRINTLN(bfr);
    },
    (void*)this);
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

  /* Setup proxy sensor */
  _proxySensor.onDataReady([](void* context, ProxySensor& sensor) {
    ((Wheelly*)context)->handleProxyData();
  },
                           this);
  _proxySensor.interval(_sendInterval);
  _proxySensor.offset(SERVO_OFFSET);
  _proxySensor.begin();

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
    ((Wheelly*)context)->sendReply(error);
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
    DEBUG_PRINTLN("!! mpu timeout");
  }

  /* Polls sensors */
  _proxySensor.polling(t0);
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
   Queries and sends the status
*/
void Wheelly::queryStatus(void) {
  //sendStatus(millis());
  sendMotion(millis());
  sendProxy();
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
  DEBUG_PRINTLN("// Wheelly::handleChangedContacts");
  //sendStatus(millis());
  sendContacts();
}

/*
   Handles sample event from distance sensor
*/
void Wheelly::handleProxyData(void) {
  /* Reads proxy data */
  boolean prevCanMove = canMoveForward();
  const unsigned long echoDelay = _proxySensor.echoDelay();
  _echoDelay = echoDelay > MIN_ECHO_TIME ? echoDelay : 0;
  _echoDirection = _proxySensor.echoDirection();
  _echoTime = _proxySensor.echoTime();
  _echoYaw = _yaw;
  _echoXPulses = _motionCtrl.xPulses();
  _echoYPulses = _motionCtrl.yPulses();
  DEBUG_PRINT("//Wheelly::handleEchoSample ");
  DEBUG_PRINTLN(echoDelay);

  /* Checks for obstacles */
  if (_motionCtrl.isForward() && !canMoveForward()
      || _motionCtrl.isBackward() && !canMoveBackward()) {
    /* Halt robot if cannot move */
    _motionCtrl.halt();
  }
  /* Sends sensor data */
  //sendStatus(millis());
  sendProxy();
  if (canMoveForward() != prevCanMove) {
    sendContacts();
  }

  /* Displays sensor data */
  if (_echoTime == 0) {
    _display.distance(-1);
  } else {
    // Converts delay (us) to distance (cm)
    int distance = (int)(_echoDelay * 100 / 5887);
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
void Wheelly::handleStats(void) {
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

    DEBUG_PRINT("// Wheelly::averageSupply : supply=");
    DEBUG_PRINT(_supplyVoltage);
    DEBUG_PRINT(", samples=");
    DEBUG_PRINT(_supplySamples);
    DEBUG_PRINTLN();

    _supplySamples = 0;
    _supplyTotal = 0;

    /* Computes the charging level */
    const int hLevel = min(max(
                             (int)map(_supplyVoltage, MIN_VOLTAGE_VALUE, MAX_VOLTAGE_VALUE, 0, 10),
                             0),
                           9);
    const int level = (hLevel + 1) / 2;

    DEBUG_PRINT("// Wheelly::sendSupply v=");
    DEBUG_PRINT(v);
    DEBUG_PRINT(" , hlevel=");
    DEBUG_PRINT(hLevel);
    DEBUG_PRINT(" , level=");
    DEBUG_PRINT(level);
    DEBUG_PRINTLN();

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
  sprintf(bfr, "sv %ld %d",
          _supplyTime,
          _supplyVoltage);
  sendReply(bfr);
}

/*
   Sends the motion of wheelly
*/
void Wheelly::sendMotion(const unsigned long t0) {
  char bfr[256];
  /* st time x y yaw lpps rpps err halt dir speed lspeed rspeed lpwr rpw */
  sprintf(bfr, "mt %ld %.1f %.1f %d %.1f %.1f %d %d %d %d %d %d %d %d",
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
  sendReply(bfr);
  _lastSend = t0;
}

/*
   Sends the status of contacts
*/
void Wheelly::sendContacts(void) {

  char bfr[256];
  /* ct time frontSig rearSig canF canB */
  sprintf(bfr, "ct %ld %d %d %d %d",
          millis(),
          _contactSensors.frontClear(),
          _contactSensors.rearClear(),
          canMoveForward(),
          canMoveBackward());
  sendReply(bfr);
}

/*
   Sends the status of proxy sensor
*/
void Wheelly::sendProxy(void) {
  char bfr[256];
  /* px time direction delay x y yaw */
  sprintf(bfr, "px %ld %d %ld %.1f %.1f %d",
          _echoTime,
          _echoDirection,
          _echoDelay,
          _echoXPulses,
          _echoYPulses,
          _echoYaw);
  sendReply(bfr);
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
  return (_echoDelay > 0 && _echoDelay <= MAX_ECHO_TIME) ? _echoDelay : MAX_ECHO_TIME;
}

/**
   Replies the configuration
*/
void Wheelly::queryConfig(void) {
  char bfr[256];

  MotorCtrl& leftMotor = _motionCtrl.leftMotor();
  sprintf(bfr, "// tcsl %d %d %d %d %d %d %d %d",
          leftMotor.p0Forw(),
          leftMotor.p1Forw(),
          leftMotor.pxForw(),
          leftMotor.p0Back(),
          leftMotor.p1Back(),
          leftMotor.pxBack(),
          leftMotor.ax(),
          leftMotor.alpha());
  sendReply(bfr);
  sprintf(bfr, "// fl %ld %ld",
          leftMotor.muForw(),
          leftMotor.muBack());
  sendReply(bfr);

  MotorCtrl& rightMotor = _motionCtrl.leftMotor();
  sprintf(bfr, "// tcsr %d %d %d %d %d %d %d %d",
          rightMotor.p0Forw(),
          rightMotor.p1Forw(),
          rightMotor.pxForw(),
          rightMotor.p0Back(),
          rightMotor.p1Back(),
          rightMotor.pxBack(),
          rightMotor.ax(),
          rightMotor.alpha());
  sendReply(bfr);
  sprintf(bfr, "// fr %ld %ld",
          rightMotor.muForw(),
          rightMotor.muBack());
  sendReply(bfr);

  sprintf(bfr, "// cc %d %d %d",
          _motionCtrl.minRotRange(),
          _motionCtrl.maxRotRange(),
          _motionCtrl.maxRotPps());
  sendReply(bfr);

  sprintf(bfr, "// cs %lu",
          _motionCtrl.sensors().tau());
  sendReply(bfr);
}
