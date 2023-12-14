#include "ProxySensor.h"

//#define DEBUG
#include "debug.h"

static const unsigned long RESET_INTERVAL = 3000ul;
static const unsigned long INACTIVITY = 50ul;
static const unsigned long INACTIVITY_MICROS = INACTIVITY * 1000;
static const int NO_SAMPLES = 3;
static const float DEG_PER_MILLIS = 0.315; // Max 0.333

/**
   Computes the spline coefficents
*/
static void spline(float&a, float&b, unsigned long& time, const int from, const int to) {
  float v = to >= from ? DEG_PER_MILLIS : -DEG_PER_MILLIS;
  int dy = to - from;
  time = (unsigned long) round(1.5 * dy / v);
  a = -16 * v * v * v / 27 / dy / dy;
  b = -4 * v * v / 3 / dy;
  DEBUG_PRINT("// ProxySensor::spline from=");
  DEBUG_PRINT(from);
  DEBUG_PRINT(", to=");
  DEBUG_PRINT(to);
  DEBUG_PRINTLN();

  DEBUG_PRINT("//   dy=");
  DEBUG_PRINT(dy);
  DEBUG_PRINT(", v=");
  DEBUG_PRINT(v);
  DEBUG_PRINTLN();

  DEBUG_PRINT("//   time=");
  DEBUG_PRINT(time);
  DEBUG_PRINT(", a=");
  DEBUG_PRINTF(a, 6);
  DEBUG_PRINT(", b=");
  DEBUG_PRINTF(b, 6);
  DEBUG_PRINTLN();
}

/**
   Creates the proxy sensor
*/
ProxySensor::ProxySensor(const uint8_t servoPin, const uint8_t triggerPin, const uint8_t echoPin)
  : _servoPin(servoPin),
    _triggerPin(triggerPin),
    _echoPin(echoPin),
    _interval(INACTIVITY) {
}

/**
   Begins the sensor
*/
void ProxySensor::begin(void) {
  DEBUG_PRINTLN("ProxySensor::begin");
  _servo.attach(_servoPin);
  _servo.write(90 - _direction - _offset);
  pinMode(_echoPin, INPUT);
  pinMode(_triggerPin, OUTPUT);
  /* Computes the time to position and scan */
  unsigned long t0 = millis();
  _pingTime = t0 + (unsigned long) round(90 / DEG_PER_MILLIS);
  _positionTime = _lastPoll = t0;
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
  if (t0 > _lastPoll) {
    /* Checks for reset timeout*/
    if (_resetTime != 0 && t0 >= _resetTime) {
      /* reset timed out */
      direction(0, t0);
      _resetTime = 0;
    } else if (_moving) { /* Checks for moving */
      moveServo(t0 < _positionTime ? _positionTime - t0 : 0);
    } else if (t0 >= _pingTime) { /* Check for scan time */
      ping(t0);
    }
    _lastPoll = t0;
  }
}

/**
   Scans for echo
*/
void ProxySensor::ping(const unsigned long t0) {
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
    _pingTime = t0 + _interval;
    /* Signals for measure */
    if (_onDataReady) {
      _onDataReady(_context, *this);
    }
  } else {
    /* Next measure */
    _pingTime = t0 + INACTIVITY;
  }
}

/**
   Moves the servo to position the sensor
*/
void ProxySensor::moveServo(const unsigned long dt) {
  /* Computes and move the direction of sensor */
  _direction = direction(dt);
  int wr = min(max(0, 90 - _direction - _offset), 180);
  DEBUG_PRINT("// ProxySensor::moveServo dt: ");
  DEBUG_PRINT(dt);
  DEBUG_PRINT(", dir=");
  DEBUG_PRINT(_direction);
  DEBUG_PRINT(", wr: ");
  DEBUG_PRINT(wr);
  DEBUG_PRINTLN();
  _servo.write(wr);

  /* Set moving sensor if not yet in position */
  _moving = dt > 0;
}

/**
   Returns the interpolated direction (DEG) at dt time to position
   @param dt the time to position (ms)
*/
const int ProxySensor::direction(const unsigned long dt) {
  float dir = -_a * dt * dt * dt + _b * dt * dt + _toDirection;
  DEBUG_PRINT("// ProxySensor::direction dt=");
  DEBUG_PRINT(dt);
  DEBUG_PRINT(", dir=");
  DEBUG_PRINTF(dir, 3);
  DEBUG_PRINTLN();
  return (int) round(dir);
}

/*
   Sets the sensor direction
   @param angle the direction in (DEG)
*/
void ProxySensor::direction(const int value, const unsigned long t0) {
  DEBUG_PRINT("// ProxySensor::direction dir=");
  DEBUG_PRINT(value);
  DEBUG_PRINTLN();
  if (_toDirection != value) { /* Checks for direction change */
    _toDirection = value;
    /* Computes the spline parameters */
    unsigned long timeToPosition;
    spline(_a, _b, timeToPosition, _direction, _toDirection);
    /* Computes the position instant */
    _positionTime = t0 + timeToPosition;
    /* Computes the scan instant */
    _pingTime = max(_pingTime, _positionTime);
    /* Reset measures */
    _noMeasures = _noValidSamples = 0;
    _totalDuration = 0;
    _moving = true;
  }
  /* Computes the next reset instant */
  _resetTime = t0 + RESET_INTERVAL;
}
