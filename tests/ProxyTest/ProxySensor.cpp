#include "ProxySensor.h"

//#define DEBUG
#include "debug.h"

static const unsigned long MILLIS_PER_DEG = 180ul / 60;
static const unsigned long RESET_INTERVAL = 3000ul;
static const unsigned long INACTIVITY = 50ul;
static const unsigned long INACTIVITY_MICROS = INACTIVITY * 1000;
static const int NO_SAMPLES = 3;

/**
   Creates the proxy sensor
*/
ProxySensor::ProxySensor(const uint8_t servoPin, const uint8_t triggerPin, const uint8_t echoPin)
  : _servoPin(servoPin),
    _triggerPin(triggerPin),
    _echoPin(echoPin),
    _direction(90),
    _interval(INACTIVITY) {
}

/**
   Begins the sensor
*/
void ProxySensor::begin(void) {
  _servo.attach(_servoPin);
  pinMode(_echoPin, INPUT);
  pinMode(_triggerPin, OUTPUT);
}

/**
   Sets the measure interval
*/
void ProxySensor::interval(const unsigned long interval) {
  _interval = max(interval, INACTIVITY);
}

/*
   Polls
*/
void ProxySensor::polling(const unsigned long t0) {
  /* Checks for reset timeout*/
  if (_resetTime != 0 && t0 >= _resetTime) {
    /* reset timed out */
    direction(0, t0);
    _resetTime = 0;
  } else if (t0 >= _waitTime) { /* Checks for wait timeout */
    /* Samples the echo sensor */
    digitalWrite(_triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_triggerPin, LOW);
    delayMicroseconds(2);
    unsigned long duration = pulseIn(_echoPin, HIGH, INACTIVITY_MICROS);
    DEBUG_PRINT("// ProxySensor::polling duration: ");
    DEBUG_PRINT(duration);
    DEBUG_PRINTLN();

    /* Update the measures */
    _noMeasures++;
    /* Checks for valid measure */
    if (duration > 0 || duration < INACTIVITY_MICROS) {
      _totalDuration += duration;
      _noValidSamples++;
    }
    /* Checks for last measure */
    if (_noMeasures >= NO_SAMPLES) {
      /* averages the measure and latch it */
      _echoTime = t0;
      _echoDirection = _direction;
      _echoDelay = _noValidSamples > 0 ? _totalDuration / _noValidSamples : 0;
      /* Resets the measures */
      _noValidSamples = _noMeasures = 0;
      _totalDuration = 0;
      /* set the wait timeout for next measure */
      _waitTime = t0 + _interval;
      /* Signals for measure */
      if (_onDataReady) {
        _onDataReady(_context, *this);
      }
    } else {
      /* Next measure */
      _waitTime = t0 + INACTIVITY;
    }
  }
}

/*
   Sets the angle
   @param angle the angle in (DEG)
*/
void ProxySensor::direction(const int value, const unsigned long t0) {
  /* Computes the servo rotation angle */
  int angle = value;
  int da = abs(angle - _direction);
  _direction = angle;
  if (da != 0) {
    /* Moves servo */
    int wr = 90 - _direction - _offset;
    DEBUG_PRINT("// ProxySensor::direction wr: ");
    DEBUG_PRINT(wr);
    DEBUG_PRINTLN();
    _servo.write(wr);
  }
  /* Computes the movement interval */
  const unsigned long moveTime = da * MILLIS_PER_DEG;
  /* Sets the new measure timeout */
  _waitTime = max(moveTime + t0, _waitTime);
  /* Sets the reset timeout */
  _resetTime = t0 + RESET_INTERVAL;
  /* Reset measures */
  _noMeasures = _noValidSamples = 0;
  _totalDuration = 0;
}
