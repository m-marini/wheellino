/*
   Measure the motor speed.

   Sending command 'start' the robot will move randomly
   and after the test interval (1s) will print the selected motor powers
   and the motor speed measured (pps).
*/
#include <Wire.h>

#define HALT_INTERVAL     500ul
#define DEFAULT_MEASURE_INTERVAL 200ul
#define DEFAULT_NUM_STEPS 20
#define DEFAULT_MAX_POWER 127

//#define DEBUG
#include "debug.h"
#include "Timer.h"
#include "MotionSensor.h"
#include "MotorCtrl.h"

/*
   Pins
*/
#define LEFT_PIN        2
#define RIGHT_FORW_PIN  3
#define BLOCK_LED_PIN   4
#define RIGHT_BACK_PIN  5
#define LEFT_BACK_PIN   6
#define ECHO_PIN        7
#define TRIGGER_PIN     8
#define SERVO_PIN       9
#define RIGHT_PIN       10
#define LEFT_FORW_PIN   11
#define PROXY_LED_PIN   12
#define VOLTAGE_PIN     A3

/*
   Serial config
*/
#define SERIAL_BPS  115200

/*
   Command parser definitions
*/
#define LINE_SIZE 100

/*
   Intervals
*/
#define LED_INTERVAL      50ul

/*
   Dividers
*/
#define LED_PULSE_DIVIDER  31
#define LED_FAST_PULSE_DIVIDER  7
#define BLOCK_PULSE_DIVIDER  11

/*
   Command parser
*/
char line[LINE_SIZE];

/*
   Variables
*/
int counter;
int numSteps = DEFAULT_NUM_STEPS;
int maxPower = DEFAULT_MAX_POWER;
unsigned long measureInterval = 300;
int left;
int right;
bool running;

/*
   Timers
*/

Timer ledTimer;
Timer measureTimer;
Timer waitTimer;

/*
   Statistics
*/
MotionSensor sensors(LEFT_PIN, RIGHT_PIN);
MotorCtrl leftMotor(LEFT_FORW_PIN, LEFT_BACK_PIN);
MotorCtrl rightMotor(RIGHT_FORW_PIN, RIGHT_BACK_PIN);

/*
*/
void serialFlush() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

/*
   Set up
*/
void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PROXY_LED_PIN, OUTPUT);
  pinMode(BLOCK_LED_PIN, OUTPUT);
  pinMode(LEFT_FORW_PIN, OUTPUT);
  pinMode(LEFT_BACK_PIN, OUTPUT);
  pinMode(RIGHT_FORW_PIN, OUTPUT);
  pinMode(RIGHT_BACK_PIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PROXY_LED_PIN, LOW);
  digitalWrite(BLOCK_LED_PIN, LOW);
  digitalWrite(LEFT_FORW_PIN, LOW);
  digitalWrite(LEFT_BACK_PIN, LOW);
  digitalWrite(RIGHT_FORW_PIN, LOW);
  digitalWrite(RIGHT_BACK_PIN, LOW);

  Serial.println();

  /*
    Init led
  */
  digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i < 3; i++) {
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
  }
  ledTimer.onNext(handleLedTimer)
  .interval(LED_INTERVAL)
  .continuous(true)
  .start();

  measureTimer.onNext(handleMeasureTimer)
  .interval(DEFAULT_MEASURE_INTERVAL);

  waitTimer.onNext(handleWaitTimer)
  .interval(HALT_INTERVAL);

  sensors.begin();

  //leftMotor.begin().setCorrection(leftXCorrection, leftYCorrection);
  //rightMotor.begin().setCorrection(rightXCorrection, rightYCorrection);

  // Final setup
  Serial.println(F("ha"));
  serialFlush();
}

/*
   Main loop
*/
void loop() {
  unsigned long now = millis();
  sensors.polling(now);
  ledTimer.polling(now);
  measureTimer.polling(now);
  waitTimer.polling(now);
  pollSerialPort();
}

/*
   Poll the seraial port
*/
void pollSerialPort() {
  if (Serial.available()) {
    unsigned long time = millis();
    size_t n = Serial.readBytesUntil('\n', line, sizeof(line) - 1);
    line[n] = 0;
    processCommand(time);
  }
}

void handleWaitTimer(void *, unsigned long) {
  sensors.reset();
  measureTimer.start();
}

void handleMeasureTimer(void *, unsigned long) {
  long leftPulses = sensors.leftPulses();
  long rightPulses = sensors.rightPulses();
  long leftSpeed = leftPulses * 1000 / (long)measureInterval;
  long rightSpeed = rightPulses * 1000 / (long)measureInterval;
  Serial.print(F("sa "));
  Serial.print(left);
  Serial.print(F(" "));
  Serial.print(right);
  Serial.print(F(" "));
  Serial.print(leftSpeed);
  Serial.print(F(" "));
  Serial.print(rightSpeed);
  Serial.println();
  counter++;
  int phase = counter / numSteps;
  int step = counter % numSteps;
  switch (phase) {
    case 0:
      // Positive ramp up
      left = right = map(step, 0, numSteps - 1, 0, maxPower);
      break;
    case 1:
      // Positive ramp down
      left = right = map(step, 0, numSteps - 1, maxPower, 0);
      break;
    case 2:
      // Negative ramp down
      left = right = map(step, 0, numSteps - 1, 0, -maxPower);
      break;
    case 3:
      // Negative ramp up
      left = right = map(step, 0, numSteps - 1, -maxPower, 0);
      break;
    case 4:
      // CW ramp up
      left = map(step, 0, numSteps - 1, 0, maxPower);
      right = -left;
      break;
    case 5:
      // CW ramp down
      left = map(step, 0, numSteps - 1, maxPower, 0);
      right = -left;
      break;
    case 6:
      // CCW ramp up
      left = map(step, 0, numSteps - 1, 0, -maxPower);
      right = -left;
      break;
    case 7:
      // CCW ramp down
      left = map(step, 0, numSteps - 1, -maxPower, 0);
      right = -left;
      break;
    default:
      stop();
      return;
  }
  leftMotor.speed(left);
  rightMotor.speed(right);
  sensors.reset();
  sensors.setDirection(left, right);
  if (left == 0 && right == 0) {
    waitTimer.start();
  } else {
    measureTimer.start();
  }
}

/*
   Handles the led timer
*/
void handleLedTimer(void *, unsigned long n) {
  unsigned long ledDivider = !running ? LED_PULSE_DIVIDER : LED_FAST_PULSE_DIVIDER;
  digitalWrite(LED_BUILTIN, (n % ledDivider) == 0 ? HIGH : LOW);
}

/*
  Process a command from serial port
*/
void processCommand(unsigned long time) {
  DEBUG_PRINT(F("// processCommand: "));
  DEBUG_PRINTLN(line);
  strtrim(line, line);
  if (strcmp(line, "start") == 0) {
    start();
  } else if (strncmp(line, "setup ", 6) == 0) {
    setup(line);
  } else if (strcmp(line, "stop") == 0) {
    stop();
  } else if (strncmp(line, "//", 2) == 0
             || strncmp(line, "!!", 2) == 0
             || strlen(line) == 0) {
    // Ignore comments, errors, empty line
  } else {
    Serial.println(F("!! Wrong command"));
  }
}

void setup(char *cmd) {
  String args = cmd + 6;
  int s0 = 0;
  int s1 = args.indexOf(' ', s0);
  if (s1 <= 0) {
    sendMissingArgument(0, cmd);
    return;
  }
  int noSteps = args.substring(s0, s1).toInt();

  s0 = s1 + 1;
  s1 = args.indexOf(' ', s0);
  if (s1 <= 0) {
    sendMissingArgument(1, cmd);
    return;
  }
  int maxPow = args.substring(s0, s1).toInt();

  s0 = s1 + 1;
  int interval = args.substring(s0).toInt();

  // Validates arguments
  if (!(noSteps > 0 && noSteps <= 100)) {
    sendWrongArgument(0, cmd);
    return;
  }
  if (!(maxPow > 0 && maxPow <= 255)) {
    sendWrongArgument(1, cmd);
    return;
  }
  if (!(interval > 0 && interval <= 10000)) {
    sendWrongArgument(2, cmd);
    return;
  }

  maxPower = maxPow;
  numSteps = noSteps;
  measureTimer.interval(interval);
  sendCommandAck(cmd);
}

void start() {
  if (!running) {
    running = true;
    counter = 0;
    waitTimer.start();
  }
}

void sendCommandAck(char * cmd) {
  Serial.print(F("// "));
  Serial.print(cmd);
  Serial.println();
}

void sendMissingArgument(int i, char *cmd) {
  Serial.print(F("!! Missing argument "));
  Serial.print(i);
  Serial.print(F(": "));
  Serial.print(cmd);
  Serial.println();
}

void sendWrongArgument(int i, char *cmd) {
  Serial.print(F("!! Wrong argument "));
  Serial.print(i);
  Serial.print(F(": "));
  Serial.print(cmd);
  Serial.println();
}

void stop() {
  if (running) {
    running = false;
    waitTimer.stop();
    measureTimer.stop();
    leftMotor.speed(0);
    rightMotor.speed(0);
  }
}

/*

*/
char *strtrim(char *out, const char *from) {
  while (isSpace(*from)) {
    from++;
  }
  const char *to = from + strlen(from) - 1;
  while (to >= from && isSpace(*to)) {
    to--;
  }
  char *s = out;
  while (from <= to) {
    *s++ = *from++;
  }
  *s = '\0';
  return out;
}
