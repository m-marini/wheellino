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
#include "Tests.h"

/*
   Serial config
*/
#define SERIAL_BPS  115200
#define MAX_POWER   255

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
static RecordList leftRecords;
static RecordList rightRecords;

/*
  Tests
*/
static FrictionTest frictionTest(leftSensor, rightSensor,
                                 leftMotor, rightMotor, contacts,
                                 leftRecords, rightRecords);

static PowerTest powerTest(leftSensor, rightSensor,
                           leftMotor, rightMotor, contacts,
                           leftRecords, rightRecords);

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

  powerTest.onCompletion([](void *) {
    handleTestCompletion();
  });

  powerTest.onStop([](void *, const char* reason) {
    handleTestStop(reason);
  });

  frictionTest.onCompletion([](void *) {
    handleTestCompletion();
  });

  frictionTest.onStop([](void *, const char* reason) {
    handleTestStop(reason);
  });

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
  powerTest.polling(t0);
  frictionTest.polling(t0);
}

/**
   Power test completion
*/
static void handleTestCompletion(void) {
  Display.move(false);
  char *msg = "completed";
  Serial.println(msg);
  telnetServer.println(msg);
  dumpData("l", leftRecords);
  dumpData("r", rightRecords);
}


/**
   Power test completion
*/
static void handleTestStop(const char * reason) {
  Display.move(false);
  Serial.println(reason);
  telnetServer.println(reason);
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
  if (strcmp(cmd, "ha") == 0) {
    // stop command
    return handleHaltCommand();
  } else if (strncmp(cmd, "pw ", 3) == 0) {
    // start command
    return handleStartPowerCommand(cmd);
  } else if (strncmp(cmd, "fr ", 3) == 0) {
    // start command
    return handleStartFrictionCommand(cmd);
  } else {
    char msg[256];
    strcpy(msg, "!! Wrong command: ");
    strcat(msg, cmd);
    Serial.println(msg);
    telnetServer.println(msg);
    return false;
  }
}


static const boolean handleStartPowerCommand(const char* cmd) {
  if (powerTest.isRunning() || frictionTest.isRunning()) {
    char* msg = "!! Measures already running";
    Serial.println(msg);
    telnetServer.println(msg);
    return false;
  }
  int params[3];
  if (!parseCmdArgs(cmd, 3, 3, params)) {
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
  powerTest.start(millis(), params[0], params[1], params[2]);
  char* msg = "started";
  Serial.println(msg);
  telnetServer.println(msg);
  Display.move(true);
  return true;
}

static const boolean handleStartFrictionCommand(const char* cmd) {
  if (powerTest.isRunning() || frictionTest.isRunning()) {
    char* msg = "!! Measures already running";
    Serial.println(msg);
    telnetServer.println(msg);
    return false;
  }
  int params[4];
  if (!parseCmdArgs(cmd, 3, 4, params)) {
    return false;
  }
  // Validate interval
  if (!validateIntArg(params[0], 1, 1000, cmd, 0)) {
    return false;
  }
  // Validate duration
  if (!validateIntArg(params[1], 1, 1000, cmd, 1)) {
    return false;
  }
  // Validate left power
  if (!validateIntArg(params[2], -1, 1, cmd, 2)) {
    return false;
  }
  // Validate right power
  if (!validateIntArg(params[3], -1, 1, cmd, 3)) {
    return false;
  }
  frictionTest.start(millis(), params[0], params[1], params[2], params[3]);
  char* msg = "started";
  Serial.println(msg);
  telnetServer.println(msg);
  Display.move(true);
  return true;
}

static const boolean handleHaltCommand() {
  if (powerTest.isRunning()) {
    powerTest.stop("// Stopped");
    return true;
  }
  if (frictionTest.isRunning()) {
    frictionTest.stop("// Stopped");
    return true;
  }

  char* msg = "!! Measures not running";
  Serial.println(msg);
  telnetServer.println(msg);
  return false;
}

static void dumpData(const char* id, const RecordList& records) {
  size_t n = records.size();
  char msg[256];
  sprintf(msg, "%ss %ld", id, n);
  Serial.println(msg);
  telnetServer.println(msg);

  const Record* ptr = records.records();
  const unsigned long startTime = ptr->time;
  for (size_t i = 0; i < n; i++, ptr++) {
    sprintf(msg, "%sd %ld %ld %d %d", id, i,
            ptr->time - startTime,
            ptr->pulses,
            ptr->power);
    DEBUG_PRINTLN(msg);
    telnetServer.println(msg);
  }
  sprintf(msg, "%se", id);
  DEBUG_PRINTLN(msg);
  telnetServer.println(msg);
}

static void handleLeftSensor(void*, const int dPulse, const unsigned long clockTime, MotorSensor & sensor) {
  powerTest.processLeftPulses(clockTime, dPulse);
  frictionTest.processLeftPulses(clockTime, dPulse);
}

static void handleRightSensor(void*, const int dPulse, const unsigned long clockTime, MotorSensor & sensor) {
  powerTest.processRightPulses(clockTime, dPulse);
  frictionTest.processRightPulses(clockTime, dPulse);
}

static void handleContacts(void *, ContactSensors & sensors) {
  boolean front = !sensors.frontClear();
  boolean rear = !sensors.rearClear();
  block_t block = front
                  ? rear ? FULL_BLOCK : FORWARD_BLOCK
                  : rear ? BACKWARD_BLOCK : NO_BLOCK;

  Display.block(block);
  powerTest.processContacts(front, rear);
  frictionTest.processContacts(front, rear);
}
