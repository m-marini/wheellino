
#include <Wire.h>

//#define DEBUG
#include "debug.h"
#include "Timer.h"
#include "SR04.h"
#include "AsyncServo.h"

#define WITH_IMU

#ifdef WITH_IMU
#include "mpu6050mm.h"
#endif

#include "MotionCtrl.h"
#include "Utils.h"

/*
   Pins
*/
#define LEFT_PIN        2
#define RIGHT_FORW_PIN  3
#define RED_LED_PIN     4
#define RIGHT_BACK_PIN  5
#define LEFT_BACK_PIN   6
#define ECHO_PIN        7
#define TRIGGER_PIN     8
#define SERVO_PIN       9
#define RIGHT_PIN       10
#define LEFT_FORW_PIN   11
#define GREEN_LED_PIN   12
#define BLUE_LED_PIN    LED_BUILTIN

#define FRONT_CONTACTS_PIN  A1
#define REAR_CONTACTS_PIN   A2
#define VOLTAGE_PIN         A3

/*
   Serial config
*/
#define SERIAL_BPS  115200

/*
   Distances
*/
#define STOP_DISTANCE_TIME (20ul * 5887 / 100)
#define WARN_DISTANCE_TIME (60ul * 5887 / 100)
#define MAX_DISTANCE_TIME  (400ul * 5887 / 100)


/*
    Obsatcle pulses
*/
#define MIN_OBSTACLE_PULSES 3
#define MAX_OBSTACLE_PULSES 12

/*
   Scanner constants
*/
#define NO_SCAN_DIRECTIONS  10
#define SERVO_OFFSET        4
#define FRONT_DIRECTION     90

/*
   Command parser definitions
*/
#define LINE_SIZE 100
#define MAX_POWER_VALUE 255
#define MAX_SPEED 100

/*
   Intervals
*/
#define LED_INTERVAL            50ul
#define OBSTACLE_INTERVAL       50ul
#define STATS_INTERVAL          10000ul
#define QUERIES_INTERVAL        1000ul
#define MIN_QUERIES_INTERVAL    300ul
#define SCANNER_RESET_INTERVAL  1000ul

/*
   Dividers
*/
#define FAST_LED_PULSE_DIVIDER  13
#define LED_PULSE_DIVIDER       23

/*
   Voltage scale
*/
#define CONTACT_THRESHOLD 163

/*
   LED colors
*/

#define BLACK   0
#define RED     1
#define GREEN   2
#define YELLOW  3
#define BLUE    4
#define MAGENTA 5
#define CYAN    6
#define WHITE   7

/*
   Command parser
*/
char line[LINE_SIZE];

/*
   Timers
*/

Timer ledTimer;
Timer statsTimer;

/*
   Proximity sensor servo
*/
AsyncServo servo;

/*
   Proximity sensor
   Proximity distance scanner
*/
SR04 sr04(TRIGGER_PIN, ECHO_PIN);
byte nextScan;
unsigned long distanceTime;
unsigned long resetTime;

/*
   Movement motors
   Motor speeds [left, right] by direction
*/
MotionCtrl motionController(LEFT_FORW_PIN, LEFT_BACK_PIN, RIGHT_FORW_PIN, RIGHT_BACK_PIN, LEFT_PIN, RIGHT_PIN);

/*
   Statistics
*/
long counter;
unsigned long started;
unsigned long statsTime;
unsigned long tps;

/*
   Gyrosc
*/
#ifdef WITH_IMU
MPU6050 mpu;
#endif

/*
   Contact signals
   0 -> 0
   1 -> 340
   2 -> 509
   3 -> 610
*/
int frontSignals;
int rearSignals;
int error;

/*
   Set up
*/
void setup() {
  // init hardware
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(SERIAL_BPS);
  Serial.println();

  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(LEFT_FORW_PIN, OUTPUT);
  pinMode(LEFT_BACK_PIN, OUTPUT);
  pinMode(RIGHT_FORW_PIN, OUTPUT);
  pinMode(RIGHT_BACK_PIN, OUTPUT);

  writeLedColor(BLACK);
  digitalWrite(LEFT_FORW_PIN, LOW);
  digitalWrite(LEFT_BACK_PIN, LOW);
  digitalWrite(RIGHT_FORW_PIN, LOW);
  digitalWrite(RIGHT_BACK_PIN, LOW);

  /*
    Init controller
  */
  writeLedColor(RED);
  delay(100);
  Serial.println();

#ifdef WITH_IMU
  /*
     Init IMU
  */
  writeLedColor(BLACK);
  delay(100);
  writeLedColor(RED);
  mpu.begin();
  mpu.calibrate();
#endif

  ledTimer.onNext(handleLedTimer);
  ledTimer.interval(LED_INTERVAL);
  ledTimer.continuous(true);
  ledTimer.start();

  /*
    Init scanner and servo
  */
  writeLedColor(BLACK);
  delay(100);
  writeLedColor(YELLOW);
  delay(100);
  sr04.begin();
  //  sr04.noSamples(NO_SAMPLES);
  sr04.onSample(&handleSample);

  servo.attach(SERVO_PIN);
  servo.offset(SERVO_OFFSET);
  servo.onReached([](void *, byte angle) {
    // Handles position reached event from scan servo
    sr04.start();
  });
  servo.angle(FRONT_DIRECTION);

  /*
     Init motor controllers
  */
  motionController.begin();

  // Init statistics time
  started = millis();
  statsTimer.onNext(handleStatsTimer);
  statsTimer.interval(STATS_INTERVAL);
  statsTimer.continuous(true);
  statsTimer.start();

  // Final setup
  writeLedColor(BLACK);
  delay(100);
  writeLedColor(YELLOW);
  delay(100);
  writeLedColor(BLACK);
  delay(100);
  writeLedColor(GREEN);
  delay(100);
  writeLedColor(BLACK);
  delay(100);

  Serial.println(F("ha"));
  serialFlush();
}

/*
   Main loop
*/
void loop() {
  unsigned long now = millis();

  counter++;

#ifdef WITH_IMU
  mpu.polling(now);
  if (error == 0) {
    error = mpu.rc();
    if (error) {
      if (mpu.readIntStatus() == 0) {
        Serial.print(F("// mpu status: "));
        Serial.print(mpu.intStatus(), HEX);
        Serial.println();
      }
    }
  }
  float yaw = mpu.yaw();
  DEBUG_PRINT(F("// polling: yaw="));
  DEBUG_PRINTLN(yaw);
  motionController.angle(roundf(yaw * 180 / PI));
#endif
  pollContactSensors();
  motionController.polling(now);
  servo.polling(now);
  sr04.polling(now);

  ledTimer.polling(now);
  statsTimer.polling(now);
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

void pollContactSensors() {
  frontSignals = analogRead(FRONT_CONTACTS_PIN);
  rearSignals = analogRead(REAR_CONTACTS_PIN);
}

bool isFrontContact() {
  return frontSignals >= CONTACT_THRESHOLD;
}

bool isRearContact() {
  return rearSignals >= CONTACT_THRESHOLD;
}

void writeLedColor(byte color) {
  digitalWrite(RED_LED_PIN, color & 1 ? HIGH : LOW);
  digitalWrite(GREEN_LED_PIN, color & 2 ? HIGH : LOW);
  digitalWrite(BLUE_LED_PIN, color & 4 ? HIGH : LOW);
}

/*
  Handles timeout event from statistics timer
*/
void handleStatsTimer(void *context, unsigned long) {
  statsTime = millis();
  unsigned long dt = (statsTime - started);
  tps = counter * 1000 / dt;
  started = statsTime;
  counter = 0;
  sendCps();
}

/*
   Handles the led timer
*/
void handleLedTimer(void *, unsigned long n) {
  byte color = decodeFlashing(n) ? decodeColor() : BLACK;
  writeLedColor(color);
}

byte decodeColor() {
  if (error) {
    return RED;
  }
  boolean forward = canMoveForward();
  boolean backward = canMoveBackward();
  if (!forward && !backward)
    return RED;
  if (forward && !backward)
    return GREEN;
  if (!forward && backward)
    return YELLOW;
  if (distanceTime > 0 && distanceTime <= ((WARN_DISTANCE_TIME + STOP_DISTANCE_TIME) / 2)) {
    return MAGENTA;
  }
  if (distanceTime > 0 && distanceTime <= WARN_DISTANCE_TIME) {
    return CYAN;
  }
  return motionController.isHalt() ? BLUE : WHITE;
}

bool decodeFlashing(unsigned long n) {
  boolean alert = !canMoveForward() || !canMoveBackward() || error;
  unsigned long divider = alert ? FAST_LED_PULSE_DIVIDER : LED_PULSE_DIVIDER;
  unsigned long frame = n % divider;
  return frame == 0 || (error && frame == 3);
}

void sendMissingArgument(int i, const char *cmd) {
  Serial.print(F("!! Missing arg["));
  Serial.print(i + 1);
  Serial.print(F("]: "));
  Serial.print(cmd);
  Serial.println();
}

void sendEchoCommand(const char *cmd) {
  Serial.print(F("// "));
  Serial.print(cmd);
  Serial.println();
}

/*
    Handles cm command
*/
void handleCmCommand(const char* cmd) {
  String args = cmd + 3;
  int p[8];
  int s0 = 0;
  int s1 = 0;
  for (int i = 0; i < 7; i++) {
    s1 = args.indexOf(' ', s0);
    if (s1 <= 0) {
      sendMissingArgument(i, cmd);
      return;
    }
    p[i] = min(max(args.substring(s0 , s1).toInt(), -MAX_POWER_VALUE), MAX_POWER_VALUE);
    s0 = s1 + 1;
  }
  p[7] = min(max(args.substring(s0).toInt(), -MAX_POWER_VALUE), MAX_POWER_VALUE);
  motionController.correction(p);
  sendEchoCommand(cmd);
}

/*
    Handles cs command
*/
void handleCsCommand(const char* cmd) {
  String args = cmd + 3;
  int decayTime = min(max(args.toInt(), 1), 10000);
  float decay = (float)1 / decayTime;
  motionController.decay(decay);
  sendEchoCommand(cmd);
}

/*
    Handles cm command
*/
void handleCcCommand(const char* cmd) {
  String args = cmd + 3;
  int p[6];
  int s0 = 0;
  int s1 = 0;
  for (int i = 0; i < 3; i++) {
    s1 = args.indexOf(' ', s0);
    if (s1 <= 0) {
      sendMissingArgument(i, cmd);
      return;
    }
    p[i] = args.substring(s0 , s1).toInt();
    s0 = s1 + 1;
  }
  for (int i = 3; i < 5; i++) {
    s1 = args.indexOf(' ', s0);
    if (s1 <= 0) {
      sendMissingArgument(i, cmd);
      return;
    }
    p[i] = min(max(args.substring(s0 , s1).toInt(), 0), 180);
    s0 = s1 + 1;
  }
  p[5] = min(max(args.substring(s0).toInt(), 0), 180);
  motionController.controllerConfig(p);
  sendEchoCommand(cmd);
}

/*
  Handles mv command
*/
void handleMvCommand(const char* cmd) {
  String args = cmd + 3;
  int s1 = args.indexOf(' ');
  if (s1 <= 0) {
    sendMissingArgument(0, cmd);
    return;
  }
  int direction = normalDeg(args.substring(0 , s1).toInt());
  int speed = min(max(args.substring(s1 + 1).toInt(), -MAX_SPEED), MAX_SPEED);
  motionController.move(direction, speed);
  if (motionController.isForward() && !canMoveForward()
      || motionController.isBackward() && !canMoveBackward()) {
    motionController.halt();
  }
}

/*
  Handles sc command
*/
void handleScCommand(const char* cmd) {
  String args = cmd + 3;
  int angle = min(max(args.toInt(), -90), 90);

  nextScan = 90 - angle;
  resetTime = millis() + SCANNER_RESET_INTERVAL;
  DEBUG_PRINT(F("// handleScCommand: next scan="));
  DEBUG_PRINT(nextScan);
}

/*
   Handles sample event from distance sensor
*/
void handleSample(void *, unsigned long time) {
  DEBUG_PRINT(F("// handleSample: dir="));
  DEBUG_PRINT(servo.angle());
  DEBUG_PRINT(F(", time="));
  DEBUG_PRINTLN(timest);
  distanceTime = time;
  if (motionController.isForward() && !canMoveForward()
      || motionController.isBackward() && !canMoveBackward()) {
    motionController.halt();
  }
  sendStatus(distanceTime);

  unsigned long now = millis();
  if (now >= resetTime) {
    nextScan = FRONT_DIRECTION;
  }
  DEBUG_PRINT(F("// handleSample: nextDir="));
  DEBUG_PRINT(nextScan);
  DEBUG_PRINTLN();
  servo.angle(nextScan);
}

/*
  Process a command from serial port
*/
void processCommand(unsigned long time) {
  DEBUG_PRINT(F("// processCommand: "));
  DEBUG_PRINTLN(line);
  strtrim(line, line);
  if (strncmp(line, "ck ", 3) == 0) {
    sendClock(line, time);
  } else if (strcmp(line, "rs") == 0) {
    resetWhelly();
  } else if (strncmp(line, "sc ", 3) == 0) {
    handleScCommand(line);
  } else if (strncmp(line, "mv ", 3) == 0) {
    handleMvCommand(line);
  } else if (strncmp(line, "cm ", 3) == 0) {
    handleCmCommand(line);
  } else if (strncmp(line, "cc ", 3) == 0) {
    handleCcCommand(line);
  } else if (strncmp(line, "cs ", 3) == 0) {
    handleCsCommand(line);
  } else if (strcmp(line, "ha") == 0) {
    motionController.halt();
  } else if (strncmp(line, "//", 2) == 0
             || strncmp(line, "!!", 2) == 0
             || strlen(line) == 0) {
    // Ignore comments, errors, empty line
  } else {
    Serial.print(F("!! Wrong command: "));
    Serial.print(line);
    Serial.println();
  }
}

/*
   Reset Wheelly
*/
void resetWhelly() {
  motionController.halt();
  motionController.reset();
  error = 0;
  /*
    Serial.println(F("// Resetting"));
    mpu.reset();
    while (mpu.readPowerManagement() == 0 && mpu.isDeviceResetting()) {
    delay(100);
    }
    Serial.println(F("// MPU ready"));
    mpu.begin();
    Serial.println(F("// Calibrating"));
    mpu.calibrate();
    Serial.println(F("// Resetted"));
  */
}

void sendCps() {
  Serial.print(F("cs "));
  Serial.print(statsTime);
  Serial.print(F(" "));
  Serial.print(tps);
  Serial.println();
}

/*

*/
void sendStatus(unsigned long distanceTime) {
  Serial.print(F("st "));
  Serial.print(millis());
  Serial.print(F(" "));
  Serial.print(motionController.xPulses(), 1);
  Serial.print(F(" "));
  Serial.print(motionController.yPulses(), 1);
  Serial.print(F(" "));
  Serial.print(roundf(mpu.yaw() * 180 / PI));
  Serial.print(F(" "));
  Serial.print(90 - servo.angle());
  Serial.print(F(" "));
  Serial.print(distanceTime);
  Serial.print(F(" "));
  Serial.print(motionController.leftPps(), 1);
  Serial.print(F(" "));
  Serial.print(motionController.rightPps(), 1);
  Serial.print(F(" "));
  Serial.print(frontSignals);
  Serial.print(F(" "));
  Serial.print(rearSignals);
  Serial.print(F(" "));
  int voltageValue = analogRead(VOLTAGE_PIN);
  Serial.print(voltageValue);
  Serial.print(F(" "));
  Serial.print(canMoveForward());
  Serial.print(F(" "));
  Serial.print(canMoveBackward());
  Serial.print(F(" "));
  Serial.print(error);
  Serial.print(F(" "));
  Serial.print(motionController.isHalt());
  Serial.print(F(" "));
  Serial.print(motionController.direction());
  Serial.print(F(" "));
  Serial.print(motionController.speed());
  Serial.print(F(" "));
  Serial.print(90 - nextScan);
  Serial.println();
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
   Returns true if forward direction
*/
bool isForward(float left, float right) {
  return left > 0 || right > 0;
}

/*
   Returns true if can move forward
*/
bool canMoveForward() {
  return forwardBlockDistanceTime() > STOP_DISTANCE_TIME && !isFrontContact();
}

bool canMoveBackward() {
  return !isRearContact();
}

int forwardBlockDistanceTime() {
  return (distanceTime > 0 && distanceTime <= MAX_DISTANCE_TIME) ? distanceTime : MAX_DISTANCE_TIME;
}

/*

*/
void serialFlush() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}
