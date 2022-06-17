#include "SR04.h"

//#define DEBUG
#include "debug.h"

#define INACTIVITY  50ul
#define INACTIVITY_MICROS  (INACTIVITY * 1000)
#define NO_SAMPLES  1

SR04::SR04(byte triggerPin, byte echoPin)
  : _triggerPin{triggerPin}, _echoPin{echoPin}  {
  _timer.onNext(_handleTimeout, this);
}

void SR04::begin() {
  pinMode(_echoPin, INPUT);
  pinMode(_triggerPin, OUTPUT);
}

void SR04::onSample(void (*callback)(void* context, int distance), void *context) {
  _onSample = callback;
  _context = context;
}

void SR04::start() {
  _timer.stop();
  _timer.interval(INACTIVITY);
  _noMeasures = 0;
  _noValidSamples = 0;
  _totalDuration = 0;
  _timer.start();
  _measure();
}

void SR04::stop() {
  _timer.stop();
}

void SR04::_measure() {
  digitalWrite(_triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_triggerPin, LOW);
  delayMicroseconds(2);
  long duration = pulseIn(_echoPin, HIGH, INACTIVITY_MICROS);
  DEBUG_PRINT(F("// SR04::_measure to,duration: "));
  DEBUG_PRINT(to);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(duration);
  DEBUG_PRINTLN();
  _noMeasures++;
  if (duration > 0 || duration < INACTIVITY_MICROS) {
    _totalDuration += duration;
    _noValidSamples++;
  }
}

void SR04::polling(unsigned long clockTime) {
  _timer.polling(clockTime);
}

void SR04::_send() {
  if (_noMeasures >= NO_SAMPLES) {
    long distance = 0;
    if (_noValidSamples > 0) {
      distance = (_totalDuration * 100) / _noValidSamples / 5882;
    }
    if (_onSample != NULL) {
      _onSample(_context, distance);
    }
  } else {
    _timer.start();
    _measure();
  }
}

static void SR04::_handleTimeout(void *context, unsigned long){
  ((SR04*)context)->_send();
}
