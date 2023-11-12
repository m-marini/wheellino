#include "SR04.h"

//#define DEBUG
#include "debug.h"

static const unsigned long INACTIVITY = 50ul;
static const unsigned long INACTIVITY_MICROS = INACTIVITY * 1000;

/*
   Creates the SR04Class controller
*/
SR04Class::SR04Class(const uint8_t triggerPin, const uint8_t echoPin)
  : _triggerPin{triggerPin}, _echoPin{echoPin}, _samplesNum(3)
{}

/*
   Initializes the SR04Class controller
*/
void SR04Class::begin() {
  pinMode(_echoPin, INPUT);
  pinMode(_triggerPin, OUTPUT);
  _lastMeasure = millis();
}

/*
   Starts the measures of distance
*/
void SR04Class::start() {
  _armed = true;
  _validSamplesNum = _falseSamplesNum = 0;
  _duration = 0;
}

/*
   Stops the measures
*/
void SR04Class::stop() {
  _armed = false;
}

/*
   Measures the distance
*/
void SR04Class::measure(const unsigned long t0) {
  _lastMeasure = t0;
  digitalWrite(_triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_triggerPin, LOW);
  delayMicroseconds(2);
  // Stops interrupt
  noInterrupts();
  unsigned long duration = pulseIn(_echoPin, HIGH, INACTIVITY_MICROS);
  // Starts interrupt
  interrupts();
  if (duration > 0) {
    _duration += duration;
    _validSamplesNum++;
  } else {
    _falseSamplesNum++;
  }
  DEBUG_PRINT("// SR04Class::measure duration: ");
  DEBUG_PRINT(duration);
  DEBUG_PRINTLN();

  unsigned long echoTime = 0;
  if (_validSamplesNum + _falseSamplesNum >= _samplesNum) {
    _armed = false;
    if (_validSamplesNum > _falseSamplesNum) {
      echoTime = _duration / _validSamplesNum;
    }
    DEBUG_PRINT("// SR04Class::measure echo: ");
    DEBUG_PRINT(echoTime);
    DEBUG_PRINTLN();
    if (_onSample != NULL) {
      _onSample(_context, echoTime);
    }
  }
}

/*
   Polls for controller
*/
void SR04Class::polling(const unsigned long t0) {
  if (_armed && t0 >= _lastMeasure + INACTIVITY) {
    measure(t0);
  }
}
