/*
  The motion sensor test performs a test checking the motion of robot:
  1. When drives only the left motor forward to max speed
     Then the motion sensors should measure the left motion and right stop
       and the mpu should measure the rotation to right
  2. When drives only the left motor backward only to max speed
     Then the motion sensors should measure the left motion and right stop
       and the mpu should measure the rotation to left
  3. When drives only the right motor forward to max speed
     Then the motion sensors should measure the right motion and left stop
       and the mpu should measure the rotation to left
  4. When drives only the right motor backward to max speed
     Then the motion sensors should measure the left motion and right stop
       and the mpu should measure the rotation to right
  The measured speeds should differ by no more than 30% of the expected values
*/
#include <Wire.h>

//#define DEBUG
#include "debug.h"
#include "Timer.h"

#define WITH_IMU

#ifdef WITH_IMU
#include "mpu6050mm.h"
#endif

#include "MotionTest.h"
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

#define TEST_TIMEOUT 5000ul

#define NUM_PULSES  40
#define EXP_MAX_PPS 44
#define PPS_EPS     28
#define STOP_PPS_EPS     2

#define EXP_MAX_ANG_SPEED 194
#define ANG_SPEED_EPS 123

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

MotionTest test(LEFT_FORW_PIN, LEFT_BACK_PIN, RIGHT_FORW_PIN, RIGHT_BACK_PIN, LEFT_PIN, RIGHT_PIN);

class TestData {
  public:
    TestData(const long timeout,
             const float leftSpeed, const float rightSpeed,
             const int numPulses,
             const float leftPps, const float leftPpsEps,
             const float rightPps, const float rightPpsEps,
             const float mpuAng, const float mpuAngEps,
             const float sensAng, const float sensAngEps
            );
    const long timeout;
    const float leftSpeed;
    const float rightSpeed;
    const int numPulses;

    const float leftPps;
    const float rightPps;
    const float leftPpsEps;
    const float rightPpsEps;
    const float mpuAng;
    const float mpuAngEps;
    const float sensAng;
    const float sensAngEps;
};

TestData::TestData(const long timeout,
                   const float leftSpeed, const float rightSpeed,
                   const int numPulses,
                   const float leftPps, const float leftPpsEps,
                   const float rightPps, const float rightPpsEps,
                   const float mpuAng, const float mpuAngEps,
                   const float sensAng, const float sensAngEps
                  ):
  timeout(timeout),
  leftSpeed(leftSpeed), rightSpeed(rightSpeed),
  numPulses(numPulses),
  leftPps(leftPps), leftPpsEps(leftPpsEps),
  rightPps(rightPps), rightPpsEps(rightPpsEps),
  mpuAng(mpuAng), mpuAngEps(mpuAngEps),
  sensAng(sensAng), sensAngEps(sensAngEps)
{
}

const TestData testData[] = {
  // Test 0
  TestData(TEST_TIMEOUT, 1, 0, NUM_PULSES,
  EXP_MAX_PPS, PPS_EPS, 0, STOP_PPS_EPS,
  0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS,
  0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS),

  // Test 1
  TestData(TEST_TIMEOUT, -1, 0, NUM_PULSES,
  -EXP_MAX_PPS, PPS_EPS, 0, STOP_PPS_EPS,
  -0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS,
  -0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS),

  // Test 2
  TestData(TEST_TIMEOUT, 0, 1,  NUM_PULSES,
  0, STOP_PPS_EPS, EXP_MAX_PPS, PPS_EPS,
  -0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS,
  -0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS),

  // Test 3
  TestData(TEST_TIMEOUT, 0, -1, NUM_PULSES,
  0, STOP_PPS_EPS, -EXP_MAX_PPS, PPS_EPS,
  0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS,
  0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS),

  // Test 4
  TestData(TEST_TIMEOUT, 1, -1, 2 * NUM_PULSES,
  EXP_MAX_PPS, PPS_EPS, -EXP_MAX_PPS, PPS_EPS,
  EXP_MAX_ANG_SPEED, ANG_SPEED_EPS,
  EXP_MAX_ANG_SPEED, ANG_SPEED_EPS),

  // Test 5
  TestData(TEST_TIMEOUT, -1, 1, 2 * NUM_PULSES,
  -EXP_MAX_PPS, PPS_EPS, EXP_MAX_PPS, PPS_EPS,
  -EXP_MAX_ANG_SPEED, ANG_SPEED_EPS,
  -EXP_MAX_ANG_SPEED, ANG_SPEED_EPS),

  // Test 6
  TestData(TEST_TIMEOUT, 0.5, -0.5, NUM_PULSES,
  0.5 * EXP_MAX_PPS , PPS_EPS, -0.5 * EXP_MAX_PPS , PPS_EPS,
  0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS,
  0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS),

  // Test 7
  TestData(TEST_TIMEOUT, -0.5, 0.5, NUM_PULSES,
  -0.5 * EXP_MAX_PPS , PPS_EPS, 0.5 * EXP_MAX_PPS , PPS_EPS,
  -0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS,
  -0.5 * EXP_MAX_ANG_SPEED, 0.5 * ANG_SPEED_EPS),
};

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

  ledTimer.onNext(handleLedTimer);
  ledTimer.interval(LED_INTERVAL);
  ledTimer.continuous(true);
  ledTimer.start();

  /*
     Init motor controllers
  */
  test.begin();
  test.setOnCompleted(handleTestCompleted);

  writeLedColor(BLACK);
  delay(100);
  writeLedColor(GREEN);
  delay(100);
  writeLedColor(BLACK);
  delay(100);

  Serial.println(F("Testing active"));
  serialFlush();
}

/*
   Main loop
*/
void loop() {
  unsigned long now = millis();
  pollContactSensors();
  test.polling(now);
  ledTimer.polling(now);
  pollSerialPort();
}

int testIndex = 0;

void startTest() {
  testIndex = 0;
  nextTest();
}

void nextTest() {
  Serial.println();
  Serial.print(F("Test #"));
  Serial.print(testIndex);
  Serial.println();
  const TestData& data = testData[testIndex];
  test.start(data.timeout, data.leftSpeed, data.rightSpeed, data.numPulses);
}

void printResult(const String& text, const float value, const float expected, const float epsilon, const int prec = 0) {
  float minValue = expected - epsilon;
  float maxValue = expected + epsilon;
  if (!(value >= minValue && value <= maxValue)) {
    Serial.print(F("* "));
  }
  Serial.print(text);
  Serial.print(value, prec);
  Serial.print(F(" ref. ("));
  Serial.print(minValue, prec);
  Serial.print(F(", "));
  Serial.print(maxValue, prec);
  Serial.print(F(")"));
  Serial.println();
}

void handleTestCompleted(void *, MotionTest& test) {

  long elapsed = test.testTime();
  float leftPulses = test.leftPulsesCounter() * 1000.0 / elapsed;
  float rightPulses = test.rightPulsesCounter() * 1000.0 / elapsed;
  float mpuAngleDeg = test.mpuAngle() * 180 / PI;
  float sensAngleDeg = test.sensAngle() * 180 / PI;

  float mpuAngularSpeed = mpuAngleDeg * 1000 / elapsed;
  float sensAngularSpeed = sensAngleDeg * 1000 / elapsed;
  int mpuError = test.mpuError();
  const TestData& data = testData[testIndex];

  Serial.print(F("Speed: "));
  Serial.print(data.leftSpeed);
  Serial.print(F(", "));
  Serial.print(data.rightSpeed);
  Serial.print(F(" duration: "));
  Serial.print(test.testTime() * 1e-3);
  Serial.println();


  if (mpuError) {
    Serial.print(F("* "));
  }
  Serial.print(F("MPU error: "));
  Serial.print(mpuError);
  Serial.println();

  printResult(F("L Pulses: "), leftPulses, data.leftPps, data.leftPpsEps);
  printResult(F("R Pulses: "), rightPulses, data.rightPps, data.rightPpsEps);
  printResult(F("MPU angular speed (DEG/s): "), mpuAngularSpeed, data.mpuAng, data.mpuAngEps);
  printResult(F("Sensors angular speed (DEG/s): "), sensAngularSpeed, data.sensAng, data.sensAngEps);

  testIndex++;
  int n = sizeof(testData) / sizeof(testData[0]);
  if (testIndex < n) {
    nextTest();
  }
}

/*
   Poll the serial port
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
#if 0
    DEBUG_PRINT(F("// threshold: "));
    DEBUG_PRINT(threshold);
    DEBUG_PRINT(F(", value: "));
    DEBUG_PRINT(value);
    DEBUG_PRINTLN();
#endif
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


/*

*/
void writeLedColor(byte color) {
  digitalWrite(RED_LED_PIN, color & 1 ? HIGH : LOW);
  digitalWrite(GREEN_LED_PIN, color & 2 ? HIGH : LOW);
  digitalWrite(BLUE_LED_PIN, color & 4 ? HIGH : LOW);
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
  return BLUE;
}

bool decodeFlashing(unsigned long n) {
  boolean alert = false;
  unsigned long divider = alert ? FAST_LED_PULSE_DIVIDER : LED_PULSE_DIVIDER;
  unsigned long frame = n % divider;
  return frame == 0;
}

/*
  Process a command from serial port
*/
void processCommand(unsigned long time) {
  DEBUG_PRINT(F("// processCommand: "));
  DEBUG_PRINTLN(line);
  strtrim(line, line);
  if (strcmp(line, "start") == 0) {
    startTest();
  } else if (strncmp(line, "//", 2) == 0
             || strncmp(line, "!!", 2) == 0
             || strlen(line) == 0) {
    // Ignore comments, errors, empty line
  } else {
    Serial.println(F("!! Wrong command"));
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
   Returns true if forward direction
*/
bool isForward(float left, float right) {
  return left > 0 || right > 0;
}

/*
   Returns true if can move forward
*/
bool canMoveForward() {
  return !isFrontContact();
}

bool canMoveBackward() {
  return !isRearContact();
}

/*

*/
void serialFlush() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}
