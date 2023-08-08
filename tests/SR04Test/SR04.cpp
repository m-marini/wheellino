#include "SR04.h"

//#define DEBUG
#include "debug.h"

static const unsigned long INACTIVITY = 50ul;
static const unsigned long INACTIVITY_MICROS = INACTIVITY * 1000;
static const int NO_SAMPLES = 1;

/*
   Creates the SR04Class controller
*/
SR04Class::SR04Class(const uint8_t triggerPin, const uint8_t echoPin)
  : _triggerPin{triggerPin}, _echoPin{echoPin}  {
  _timer.onNext(_handleTimeout, this);
}

/*
   Initializes the SR04Class controller
*/
void SR04Class::begin() {
  pinMode(_echoPin, INPUT);
  pinMode(_triggerPin, OUTPUT);
}

/*
   Starts the measures of distance
*/
void SR04Class::start() {
  _timer.stop();
  _timer.interval(INACTIVITY);
  _noMeasures = 0;
  _noValidSamples = 0;
  _totalDuration = 0;
  _timer.start();
  _measure();
}

/*
   Stops the measures
*/
void SR04Class::stop() {
  _timer.stop();
}

/*
   Measures the distance
*/
void SR04Class::_measure() {
  digitalWrite(_triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_triggerPin, LOW);
  delayMicroseconds(2);
  unsigned long duration = pulseIn(_echoPin, HIGH, INACTIVITY_MICROS);
  DEBUG_PRINT("// SR04Class::_measure to,duration: ");
  DEBUG_PRINT(to);
  DEBUG_PRINT(", ");
  DEBUG_PRINT(duration);
  DEBUG_PRINTLN();
  _noMeasures++;
  if (duration > 0 || duration < INACTIVITY_MICROS) {
    _totalDuration += duration;
    _noValidSamples++;
  }
}

/*
   Polls for controller
*/
void SR04Class::polling(const unsigned long clockTime) {
  _timer.polling(clockTime);
}

/*
   Sends the measure of distance and restarts the timer
*/
void SR04Class::_send() {
  if (_noMeasures >= NO_SAMPLES) {
    unsigned long distance = 0;
    if (_noValidSamples > 0) {
      distance = _totalDuration / _noValidSamples;
    }
    if (_onSample != NULL) {
      _onSample(_context, distance);
    }
  } else {
    _timer.start();
    _measure();
  }
}

/*
   Timer callback
*/
void SR04Class::_handleTimeout(void *context, const unsigned long) {
  ((SR04Class*)context)->_send();
}
