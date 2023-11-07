#include "SR04.h"

//#define DEBUG
#include "debug.h"

static const unsigned long INACTIVITY = 50ul;
static const unsigned long INACTIVITY_MICROS = INACTIVITY * 1000;

/*
   Creates the SR04Class controller
*/
SR04Class::SR04Class(const uint8_t triggerPin, const uint8_t echoPin)
  : _triggerPin{triggerPin}, _echoPin{echoPin}  {
}

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
  unsigned long _duration = pulseIn(_echoPin, HIGH, INACTIVITY_MICROS);
  _armed = false;
  DEBUG_PRINT("// SR04Class::_measure to,duration: ");
  DEBUG_PRINT(to);
  DEBUG_PRINT(", ");
  DEBUG_PRINT(_duration);
  DEBUG_PRINTLN();
  if (_onSample != NULL) {
    _onSample(_context, _duration);
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
