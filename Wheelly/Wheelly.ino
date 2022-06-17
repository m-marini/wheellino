
#include <Wire.h>

//#define DEBUG
#include "debug.h"
#include "Timer.h"
#include "SR04.h"
#include "AsyncServo.h"
#include "I2Cdev.h"

#define WITH_IMU

#ifdef WITH_IMU
#include "IMU.h"
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
#define NO_SCAN_DIRECTIONS  10
#define SERVO_OFFSET        4
#define FRONT_DIRECTION     90

/*
   Command parser definitions
*/
#define LINE_SIZE 100

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
#define VOLTAGE_SCALE 15.22e-3

/*
   Voltage scale
*/
#define MAX_CONTACT_LEVEL 511
#define FRONT_SIGNAL_MASK 0xc
#define REAR_SIGNAL_MASK 0x3

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
int distance;
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
IMU imu(mpu);
bool imuFailure;
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
byte contactSignals;
const int contactLevels[] PROGMEM = {170, 424, 560};
#define NO_LEVELS (sizeof(contactLevels) / sizeof(contactLevels[0]))

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
    Init led
  */
  writeLedColor(BLACK);
  for (int i = 1; i < 3; i++) {
    delay(100);
    writeLedColor(RED);
    delay(50);
    writeLedColor(BLACK);
  }
  Serial.println();

#ifdef WITH_IMU
  /*
     Init IMU
  */
  imuFailure = true;
  imu.begin();
  imu.onData(handleImuData);
  imu.onWatchDog(handleWatchDog);
  imu.calibrate();
  imu.enableDMP();
  imu.reset();
#endif


  ledTimer.onNext(handleLedTimer);
  ledTimer.interval(LED_INTERVAL);
  ledTimer.continuous(true);
  ledTimer.start();

  /*
    Init servo and scanner
  */
  sr04.begin();
  //  sr04.noSamples(NO_SAMPLES);
  sr04.onSample(&handleSample);
  servo.attach(SERVO_PIN);
  servo.offset(SERVO_OFFSET);
  servo.onReached([](void *, byte angle) {
    // Handles position reached event from scan servo
    /*
      DEBUG_PRINT(F("// handleReached: dir="));
      DEBUG_PRINTLN(angle);
    */
    sr04.start();
  });
  servo.angle(FRONT_DIRECTION);

  /*
     Init motor controllers
  */
  motionController.begin();

  // Init staqstistics time
  started = millis();
  statsTimer.onNext(handleStatsTimer);
  statsTimer.interval(STATS_INTERVAL);
  statsTimer.continuous(true);
  statsTimer.start();

  // Final setup
#ifdef WITH_IMU
  imu.reset();
#endif

  for (int i = 1; i < 3; i++) {
    delay(100);
    writeLedColor(YELLOW);
    delay(50);
    writeLedColor(BLACK);
  }

  Serial.println(F("ha"));
  serialFlush();
}

/*
   Main loop
*/
void loop() {
  unsigned long now = millis();

  counter++;

  motionController.polling(now);
#ifdef WITH_IMU
  imu.polling(now);
#endif
  pollContactSensors();
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
  contactSignals = decodeContactSignals(frontSignals) * 4
                   + decodeContactSignals(rearSignals);
}

int decodeContactSignals(int value) {
  int contactSignals = 0;
  while (contactSignals < NO_LEVELS) {
    int threshold = pgm_read_word_near(&contactLevels[contactSignals]);
    DEBUG_PRINT(F("// threshold: "));
    DEBUG_PRINT(threshold);
    DEBUG_PRINT(F(", value: "));
    DEBUG_PRINT(value);
    DEBUG_PRINTLN();
    DEBUG_PRINT(threshold);
    if (value <= threshold) {
      break;
    }
    contactSignals++;
  }
  return contactSignals;
}

bool isFrontContact() {
  return (contactSignals & FRONT_SIGNAL_MASK) != 0;
}

bool isRearContact() {
  return (contactSignals & REAR_SIGNAL_MASK) != 0;
}

#ifdef WITH_IMU

/*

*/
void handleImuData(void*) {
  imuFailure = false;
  float yaw = imu.ypr()[0];
  DEBUG_PRINT(F("// handleImuData: yaw="));
  DEBUG_PRINTLN(yaw);
  motionController.angle(yaw);
}

void writeLedColor(byte color) {
  digitalWrite(RED_LED_PIN, color & 1 ? HIGH : LOW);
  digitalWrite(GREEN_LED_PIN, color & 2 ? HIGH : LOW);
  digitalWrite(BLUE_LED_PIN, color & 4 ? HIGH : LOW);
}
/*

*/
void handleWatchDog(void*) {
  imuFailure = true;
  DEBUG_PRINT(F("!! Watch dog status: "));
  DEBUG_PRINTLN(imu.status());
  imu.reset();
  imu.kickAt(micros() + 1000000ul);
}
#endif

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
  boolean forward = canMoveForward();
  boolean backward = canMoveBackward();
  if (!forward && !backward)
    return RED;
  if (forward && !backward)
    return GREEN;
  if (!forward && backward)
    return YELLOW;
  if (distance > 0 && distance <= ((WARN_DISTANCE + STOP_DISTANCE) / 2)) {
    return MAGENTA;
  }
  if (distance > 0 && distance <= WARN_DISTANCE) {
    return CYAN;
  }
  return motionController.isHalt() ? BLUE : WHITE;
}

bool decodeFlashing(unsigned long n) {
  boolean alert = !canMoveForward() || !canMoveBackward() || imuFailure;
  unsigned long divider = alert ? FAST_LED_PULSE_DIVIDER : LED_PULSE_DIVIDER;
  unsigned long frame = n % divider;
  return frame == 0 || (imuFailure && frame == 3);
}

/*
    Handles mv command
*/
void handleMvCommand(const char* parms) {
  String args = parms;
  int s1 = args.indexOf(' ');
  if (s1 <= 0) {
    Serial.println(F("!! Wrong arg[1]"));
    return;
  }
  float direction = normalRad(args.substring(0 , s1).toInt() * PI / 180);
  float speed = min(max(args.substring(s1 + 1).toFloat(), -1.0), 1.0);
  motionController.move(direction, speed);
  if (motionController.isForward() && !canMoveForward()
      || motionController.isBackward() && !canMoveBackward()) {
    motionController.halt();
  }
}

/*
    Handles sc command
*/
void handleScCommand(const char* parms) {
  String args = parms;
  int angle = min(max(args.toInt(), -90), 90);

  nextScan = 90 - angle;
  resetTime = millis() + SCANNER_RESET_INTERVAL;
  DEBUG_PRINT(F("// handleScCommand: next scan="));
  DEBUG_PRINT(nextScan);
}

/*
   Handles sample event from distance sensor
*/
void handleSample(void *, int dist) {
  DEBUG_PRINT(F("// handleSample: dir="));
  DEBUG_PRINT(servo.angle());
  DEBUG_PRINT(F(", distance="));
  DEBUG_PRINTLN(dist);
  distance = dist;
  if (motionController.isForward() && !canMoveForward()
      || motionController.isBackward() && !canMoveBackward()) {
    motionController.halt();
  }
  sendStatus(dist);

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
    handleScCommand(line + 3);
  } else if (strncmp(line, "mv ", 3) == 0) {
    handleMvCommand(line + 3);
  } else if (strcmp(line, "al") == 0) {
    motionController.halt();
  } else if (strncmp(line, "//", 2) == 0
             || strncmp(line, "!!", 2) == 0
             || strlen(line) == 0) {
    // Ignore comments, errors, empty line
  } else {
    Serial.println(F("!! Wrong command"));
  }
}

/*
   Reset Wheelly
*/
void resetWhelly() {
  motionController.halt();
  motionController.reset();
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
void sendStatus(int distance) {
  Serial.print(F("st "));
  Serial.print(millis());
  Serial.print(F(" "));
  Serial.print(motionController.x(), 3);
  Serial.print(F(" "));
  Serial.print(motionController.y(), 3);
  Serial.print(F(" "));
  Serial.print(motionController.angle() * 180 / PI, 0);
  Serial.print(F(" "));
  Serial.print(90 - servo.angle());
  Serial.print(F(" "));
  Serial.print(distance * 0.01, 2);
  Serial.print(F(" "));
  Serial.print(motionController.leftSpeed(), 3);
  Serial.print(F(" "));
  Serial.print(motionController.rightSpeed(), 3);
  Serial.print(F(" "));
  Serial.print(contactSignals);
  Serial.print(F(" "));
  int voltageValue = analogRead(VOLTAGE_PIN);
  Serial.print(VOLTAGE_SCALE * voltageValue);
  Serial.print(F(" "));
  Serial.print(canMoveForward());
  Serial.print(F(" "));
  Serial.print(canMoveBackward());
  Serial.print(F(" "));
  Serial.print(imuFailure);
  Serial.print(F(" "));
  Serial.print(motionController.isHalt());
  Serial.print(F(" "));
  Serial.print(motionController.direction() * 180 / PI, 0);
  Serial.print(F(" "));
  Serial.print(motionController.speed(), 2);
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
  return forwardBlockDistance() > STOP_DISTANCE && !isFrontContact();
}

bool canMoveBackward() {
  return !isRearContact();
}

int forwardBlockDistance() {
  return (distance > 0 && distance <= MAX_DISTANCE) ? distance : MAX_DISTANCE;
}

/*

*/
void serialFlush() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}
