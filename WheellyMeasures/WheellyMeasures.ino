
#include <Wire.h>

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
   Multiplexer outputs
*/

/*
   Distances
*/
#define STOP_DISTANCE 20
#define WARN_DISTANCE 60
#define MAX_DISTANCE  400


/*
    Obsatcle pulses
*/
#define MIN_OBSTACLE_PULSES 3
#define MAX_OBSTACLE_PULSES 12

/*
   Scanner constants
*/
#define NO_SAMPLES          5
#define FRONT_SCAN_INDEX    (NO_SCAN_DIRECTIONS / 2)
#define NO_SCAN_DIRECTIONS  (sizeof(scanDirections) / sizeof(scanDirections[0]))
#define CAN_MOVE_MIN_DIR    (FRONT_SCAN_INDEX - 1)
#define CAN_MOVE_MAX_DIR    (FRONT_SCAN_INDEX + 1)
#define SERVO_OFFSET        4

/*
   Command parser definitions
*/
#define LINE_SIZE 100

/*
   Intervals
*/
#define LED_INTERVAL      50ul
#define MEASURE_INTERVAL  1000ul
#define WAIT_INTERVAL     100ul

/*
   Dividers
*/
#define LED_PULSE_DIVIDER  31
#define LED_FAST_PULSE_DIVIDER  7
#define BLOCK_PULSE_DIVIDER  11

/*
   Voltage scale
*/
#define VOLTAGE_SCALE (5.0 * 3 / 1023)

/*
   Command parser
*/
char line[LINE_SIZE];

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

int counter;
float left;
float right;
bool running;

const float leftXCorrection[] = { -1,  -0.06055, 0, 0.02311, 1};
const float leftYCorrection[] = { -1, -0.30432, 0, 0.12577, 1};
const float rightXCorrection[] = { -1, -0.03759, 0, 0.02041, 1};
const float rightYCorrection[] = { -1, -0.2667, 0, 0.12648, 1};

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
  .interval(MEASURE_INTERVAL);

  waitTimer.onNext(handleWaitTimer)
  .interval(WAIT_INTERVAL);

  sensors.begin();

  leftMotor.begin().setCorrection(leftXCorrection, leftYCorrection);
  rightMotor.begin().setCorrection(rightXCorrection, rightYCorrection);

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
  do {
    left = float(random(511) - 255) / 255;
    right = float(random(511) - 255) / 255;
  } while (left == 0 && right == 0);
  sensors.reset();
  leftMotor.speed(left);
  rightMotor.speed(right);
  sensors.setDirection(left, right);
  measureTimer.start();
}

void handleMeasureTimer(void *, unsigned long) {
  leftMotor.speed(0);
  rightMotor.speed(0);
  long leftPulses = sensors.leftPulses();
  long rightPulses = sensors.rightPulses();
  Serial.print(F("sa "));
  Serial.print(left);
  Serial.print(F(" "));
  Serial.print(right);
  Serial.print(F(" "));
  Serial.print(leftPulses);
  Serial.print(F(" "));
  Serial.print(rightPulses);
  Serial.println();
  waitTimer.start();
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

void start() {
  if (!running) {
    running = true;
    waitTimer.start();
  }
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

/*

*/
void sendAsset() {
  Serial.print(F("as 0 0 0 0 0 0 "));
  Serial.println();
}

/*

*/
void printVect(float * vect) {
  Serial.print(*vect++);
  Serial.print(F(" "));
  Serial.print(*vect++);
  Serial.print(F(" "));
  Serial.print(*vect++);
}

/*

*/
void sendClock(const char *cmd, unsigned long timing) {
  Serial.print(cmd);
  Serial.print(F(" "));
  Serial.print(timing);
  Serial.print(F(" "));
  unsigned long ms = millis();
  Serial.print(ms);
  Serial.println();
}

/*

*/
void sendStatus() {
  Serial.print(F("st "));
  Serial.println();
}
