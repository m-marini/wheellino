/*
   Measure the motor speed.

   Sending command 'start' the robot will move randomly
   and after the test interval (1s) will print the selected motor powers
   and the motor speed measured (pps).
*/

//#define DEBUG
#include "debug.h"

#include "pins.h"

#include "WiFiModule.h"
#include "TelnetServer.h"
#include "Timer.h"
#include "MotorCtrl.h"
#include "Contacts.h"
#include "Display.h"

/*
   Serial config
*/
#define SERIAL_BPS  115200
#define MAX_POWER   255
#define BUFFER_SIZE 2048

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
   Telnet server
*/
static TelnetServerClass telnetServer;

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
   LCD
*/
static DisplayClass Display;

/*
   Variables
*/
static boolean running;
static int leftPwr;
static int rightPwr;
static int maxLeftPwr;
static int maxRightPwr;
static unsigned long startTime;
static unsigned long duration;

struct Record {
  unsigned long time;
  int power;
  int pulses;
};

static Record leftMeasures[BUFFER_SIZE];
static Record rightMeasures[BUFFER_SIZE];
static size_t leftIndex;
static size_t rightIndex;

/*
   Set up
*/
void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println();

  delay(100);

  Display.begin();
  Display.clear();

  Serial.println();

  wiFiModule.onChange(handleOnChange);
  wiFiModule.begin();

  telnetServer.onClient(handleOnClient);
  telnetServer.onLineReady(handleLineReady);

  leftSensor.onSample(handleLeftSensor);
  rightSensor.onSample(handleRightSensor);

  leftMotor.automatic(false);
  leftMotor.begin();

  rightMotor.automatic(false);
  rightMotor.begin();

  contacts.onChanged(handleContacts);
  contacts.begin();

  Serial.print("Wheelly measures started.");
  Serial.println();
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
  telnetServer.polling(t0);
  contacts.polling(t0);
  leftMotor.polling(t0);
  rightMotor.polling(t0);
  pollTest(t0);
}

/**
   Polls the test
*/
static void pollTest(const unsigned long t0) {
  if (!running || t0 < startTime) {
    return;
  }
  const unsigned long dt = t0 - startTime;

  DEBUG_PRINT("// pollTest t0: ");
  DEBUG_PRINT(t0);
  DEBUG_PRINT(", startTime: ");
  DEBUG_PRINT(startTime);
  DEBUG_PRINT(", dt: ");
  DEBUG_PRINT(dt);
  DEBUG_PRINTLN();

  if (dt > duration) {
    leftPwr = rightPwr = 0;
    leftMotor.power(0);
    rightMotor.power(0);
    running = false;

    char *msg = "completed";
    Display.move(false);
    Serial.println(msg);
    telnetServer.println(msg);
    dumpData("l", leftIndex, leftMeasures);
    dumpData("r", rightIndex, rightMeasures);
    return;
  }
  int left;
  int right;
  if (dt <= duration / 2) {
    left = map(dt, 0, duration / 2, 0, maxLeftPwr);
    right = map(dt, 0, duration / 2, 0, maxRightPwr);
  } else {
    left = map(dt, duration / 2, duration, maxLeftPwr, 0);
    right = map(dt, duration / 2, duration, maxRightPwr, 0);
  }
  if (leftPwr != left) {
    leftPwr = left;
    leftMotor.power(left);
  }
  if (rightPwr != right) {
    rightPwr = right;
    rightMotor.power(right);
  }
}

/*
   Handles the wifi module state change
*/
static void handleOnChange(void *, WiFiModuleClass & module) {
  char bfr[256];
  if (module.connected()) {
    telnetServer.begin();
    strcpy(bfr, wiFiModule.ssid());
    strcat(bfr, " - IP: ");
    strcat(bfr, wiFiModule.ipAddress().toString().c_str());
  } else if (module.connecting()) {
    strcpy(bfr, wiFiModule.ssid());
    strcat(bfr, " connecting...");
  } else {
    strcpy(bfr, "Disconnected");
  }
  Serial.println(bfr);
  Display.clear();
  Display.print(bfr);
}

/*
  Handles the wifi client connection, disconnection
*/
static void handleOnClient(void*, TelnetServerClass & telnet) {
  boolean hasClient = telnet.hasClient();
  Serial.println(hasClient
                 ? "Client connected"
                 : "Client disconnected");
  Display.connected(hasClient);
  if (hasClient) {
    telnet.println("Wheelly measures ready");
  }
}

/*
   Handles line ready from wifi
   @param line the line
*/
static void handleLineReady(void *, const char* line) {
  Serial.print(line);
  Serial.println();
  Display.activity();
  execute(line);
}

/*
   Returns true if wrong number of arguments
*/
static const boolean parseCmdArgs(const char* command, const int from, const int argc, int *argv) {
  DEBUG_PRINT("parseCmdArgs[");
  DEBUG_PRINT(command);
  DEBUG_PRINT("]");
  DEBUG_PRINTLN();

  const char* s0 = command + from;
  const char* s1 = s0;
  char parm[100];
  for (int i = 0; i < argc - 1; i++) {
    s1 = strchr(s0, ' ');
    DEBUG_PRINT("parseCmdArgs");
    if (!s1 || s1 == s0) {
      // Missing argument
      char error[256];
      sprintf(error, "!! Found %d arguments, expected %d: %s",
              i + 1,
              argc,
              command);

      Serial.println(error);
      telnetServer.println(error);
      return false;
    }
    DEBUG_PRINT("s0=[");
    DEBUG_PRINT(s0);
    DEBUG_PRINT("], s1=[");
    DEBUG_PRINT(s1);
    DEBUG_PRINT("], s1-s0=");
    DEBUG_PRINT(s1 - s0);
    DEBUG_PRINTLN();

    strncpy(parm, s0, s1 - s0);
    parm[s1 - s0] = 0;
    DEBUG_PRINT("parm=[");
    DEBUG_PRINT(parm);
    DEBUG_PRINT("]");
    DEBUG_PRINTLN();

    *argv++ = atoi(parm);
    s0 = s1 + 1;
  }
  DEBUG_PRINT("s0=[");
  DEBUG_PRINT(s0);
  DEBUG_PRINT("]");
  DEBUG_PRINTLN();
  *argv = atoi(s0);
  return true;
}

/*
   Returns true if value is invalid
*/
static const boolean validateIntArg(const int value, const int minValue, const int maxValue, const char* cmd, const int arg) {
  if (!inrange(value, minValue, maxValue)) {
    char error[256];
    sprintf(error, "!! Wrong arg[%d]: value %d must be between %d, %d range: %s",
            arg + 1,
            value,
            minValue,
            maxValue,
            cmd);
    Serial.println(error);
    telnetServer.println(error);
    return false;
  }
  return true;
}

/*
  Executes a command.
  Returns true if command executed
  @param command the command
*/
static const boolean execute(const char* command) {
  char cmd[256];
  // Left trim
  while (isspace(*command)) {
    command++;
  }
  strcpy(cmd, command);

  // Right trim
  for (int i = strlen(cmd) - 1; i >= 0 && isspace(cmd[i]); i--) {
    cmd[i] = 0;
  }

  DEBUG_PRINT("// processCommand: ");
  DEBUG_PRINTLN(cmd);
  if (strcmp(cmd, "stop") == 0) {
    // stop command
    return handleStopCommand();
  } else if (strncmp(cmd, "start ", 6) == 0) {
    // start command
    return handleStartCommand(cmd);
  } else {
    char msg[256];
    strcpy(msg, "!! Wrong command: ");
    strcat(msg, cmd);
    Serial.println(msg);
    telnetServer.println(msg);
    return false;
  }
}

static const boolean handleStartCommand(const char* cmd) {
  if (!running) {
    int params[3];
    if (!parseCmdArgs(cmd, 6, 3, params)) {
      return false;
    }
    // Validate duration
    if (!validateIntArg(params[0], 1, 60000, cmd, 0)) {
      return false;
    }
    // Validate left power
    if (!validateIntArg(params[1], -MAX_POWER, MAX_POWER, cmd, 1)) {
      return false;
    }
    // Validate right power
    if (!validateIntArg(params[2], -MAX_POWER, MAX_POWER, cmd, 2)) {
      return false;
    }
    startTest(params[0], params[1], params[2]);
    return true;
  } else {
    char* msg = "!! Measures already running";
    Serial.println(msg);
    telnetServer.println(msg);
    return false;
  }
}

static const boolean handleStopCommand() {
  if (running) {
    stopTest("// Stopped");
    return true;
  } else {
    char* msg = "!! Measures not running";
    Serial.println(msg);
    telnetServer.println(msg);
    return false;
  }
}

static void dumpData(const char* id, const size_t n, const Record * records) {
  char msg[256];
  sprintf(msg, "%ss %ld", id, n);
  Serial.println(msg);
  telnetServer.println(msg);

  for (size_t i = 0; i < n; i++, records++) {
    sprintf(msg, "%sd %ld %ld %d %d", id, i,
            records->time - startTime,
            records->pulses,
            records->power);
    DEBUG_PRINTLN(msg);
    telnetServer.println(msg);
  }
  sprintf(msg, "%se", id);
  DEBUG_PRINTLN(msg);
  telnetServer.println(msg);
}

static void startTest(const unsigned long durationParam, const int leftPwrParam, const int rightPwrParam) {
  duration = durationParam;
  maxLeftPwr = leftPwrParam;
  maxRightPwr = rightPwrParam;
  leftIndex = 0;
  rightIndex = 0;

  leftPwr = rightPwr = 0;
  startTime = millis();
  running = true;

  leftMotor.power(0);
  rightMotor.power(0);

  char* msg = "started";
  Serial.println(msg);
  telnetServer.println(msg);
  Display.move(true);
}

static void stopTest(const char* msg) {
  running = false;
  leftMotor.power(0);
  rightMotor.power(0);

  Display.move(false);
  Serial.println(msg);
  telnetServer.println(msg);
}

static void handleLeftSensor(void*, const int dPulse, const unsigned long clockTime, MotorSensor & sensor) {
  if (running) {
    addRecord(leftIndex, leftMeasures, clockTime, leftPwr, dPulse);
  }
}

static void handleRightSensor(void*, const int dPulse, const unsigned long clockTime, MotorSensor & sensor) {
  if (running) {
    addRecord(rightIndex, rightMeasures, clockTime, rightPwr, dPulse);
  }
}

static void handleContacts(void *, ContactSensors & sensors) {
  boolean front = !sensors.frontClear();
  boolean rear = !sensors.rearClear();
  block_t block = front
                  ? rear ? FULL_BLOCK : FORWARD_BLOCK
                  : rear ? BACKWARD_BLOCK : NO_BLOCK;

  Display.block(block);
  if (running && (front || rear)) {
    // Stop by contact
    char msg[256];
    sprintf(msg, "!! Stopped for %s, %s contacts",
            front ? "front" : "",
            rear ? "rear" : "");
    stopTest(msg);
  }
}

static void addRecord(size_t& index, Record* records, const unsigned long t0, const int power, const int dPulses) {
  if (index >= BUFFER_SIZE) {
    stopTest("!! Buffer overflow");
  } else {
    records[index].pulses = dPulses;
    records[index].power = power;
    records[index].time = t0;
    index++;
  }
}
