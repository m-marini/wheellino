/*
   Measure the motor speed.

   Sending command 'start' the robot will move randomly
   and after the test interval (1s) will print the selected motor powers
   and the motor speed measured (pps).
*/

//#define DEBUG
#include "debug.h"

#include "WiFiModule.h"
#include "TelnetServer.h"
#include "Timer.h"
#include "MotorCtrl.h"
#include "Contacts.h"
#include "pins.h"

/*
   Serial config
*/
#define SERIAL_BPS  115200
#define MAX_POWER   255
#define BUFFER_SIZE 1024

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
   Timers
*/
static Timer testTimer;

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

/*
   Variables
*/
static boolean running;
static int leftPwr;
static int rightPwr;
static int rightPulses[BUFFER_SIZE];
static int leftPulses[BUFFER_SIZE];
static unsigned long rightTime[BUFFER_SIZE];
static unsigned long leftTime[BUFFER_SIZE];
static size_t leftIndex;
static size_t rightIndex;
static unsigned long startTime;

/*
   Set up
*/
void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println();

  delay(100);
  Serial.println();

  wiFiModule.onChange(handleOnChange);
  wiFiModule.begin();

  telnetServer.onClient(handleOnClient);
  telnetServer.onLineReady(handleLineReady);

  testTimer.onNext(handleTestTimer);

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
}

/*
   Main loop
*/
void loop() {
  unsigned long t0 = millis();
  wiFiModule.polling(t0);
  telnetServer.polling(t0);
  contacts.polling(t0);
  leftMotor.polling(t0);
  rightMotor.polling(t0);
  testTimer.polling(t0);
}

/*
   Handles the wifi module state change
*/
static void handleOnChange(void *, WiFiModuleClass& module) {
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
}

/*
  Handles the wifi client connection, disconnection
*/
static void handleOnClient(void*, TelnetServerClass& telnet) {
  boolean hasClient = telnet.hasClient();
  Serial.println(hasClient
                 ? "Client connected"
                 : "Client disconnected");
  if (hasClient) {
    telnet.println("Wheelly measures ready");
  }
}

/*
   Handles line ready from wifi
   @param line the line
*/
static void handleLineReady(void *, const char* line) {
  Serial.print("<- ");
  Serial.print(line);
  Serial.println();
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

static void handleTestTimer(void*, const unsigned long) {
  leftMotor.power(0);
  rightMotor.power(0);
  running = false;

  char *msg = "completed";
  Serial.println(msg);
  telnetServer.println(msg);
  dumpData("l", leftIndex, leftPulses, leftTime);
  dumpData("r", rightIndex, rightPulses, rightTime);
}

static void dumpData(const char* id, const size_t n, const int* pulses, const unsigned long* times) {
  char msg[256];
  sprintf(msg, "%ss %ld", id, n);
  Serial.println(msg);
  telnetServer.println(msg);

  for (size_t i = 0; i < n; i++) {
    sprintf(msg, "%sd %ld %ld %d", id, i, times[i] - startTime, pulses[i]);
    Serial.println(msg);
    telnetServer.println(msg);
  }
  sprintf(msg, "%se", id);
  Serial.println(msg);
  telnetServer.println(msg);
}

static void startTest(const unsigned long duration, const int leftPwrParam, const int rightPwrParam) {
  running = true;
  leftPwr = leftPwrParam;
  rightPwr = rightPwrParam;
  leftIndex = 0;
  rightIndex = 0;

  leftMotor.power(leftPwr);
  rightMotor.power(rightPwr);

  testTimer.interval(duration);
  testTimer.start();
  startTime = millis();

  char* msg = "started";
  Serial.println(msg);
  telnetServer.println(msg);
}

static void stopTest(const char* msg) {
  running = false;
  testTimer.stop();
  leftMotor.power(0);
  rightMotor.power(0);

  Serial.println(msg);
  telnetServer.println(msg);
}

static void handleLeftSensor(void*, const int dPulse, const unsigned long clockTime, MotorSensor& sensor) {
  if (leftIndex >= BUFFER_SIZE) {
    stopTest("!! Left buffer overflow");
  } else {
    leftPulses[leftIndex] = dPulse;
    leftTime[leftIndex] = clockTime;
    leftIndex++;
  }
}

static void handleRightSensor(void*, const int dPulse, const unsigned long clockTime, MotorSensor& sensor) {
  if (rightIndex >= BUFFER_SIZE) {
    stopTest("!! Right buffer overflow");
  } else {
    rightPulses[rightIndex] = dPulse;
    rightTime[rightIndex] = clockTime;
    rightIndex++;
  }
}

static void handleContacts(void *, ContactSensors& sensors) {
  boolean front = !sensors.frontClear();
  boolean rear = !sensors.rearClear();
  if (running && (front || rear)) {
    // Stop by contact
    char msg[256];
    sprintf(msg, "!! Stopped for %s, %s contacts",
            front ? "front" : "",
            rear ? "rear" : "");
    stopTest(msg);
  }
}
