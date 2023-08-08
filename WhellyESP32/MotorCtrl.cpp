#include "Arduino.h"

#include "MotorCtrl.h"

//#define DEBUG

#ifdef DEBUG
//#define DEBUG_MOTOR_CTRL
//#define DEBUG_SPEEDOMETER
//#define DEBUG_POLLING
#endif

#include "debug.h"

#include "num.h"
#include "pins.h"

#define MIN_INTERVAL  100ul
#define MAX_POWER     255
#define SCALE 10000l

#define DEFAULT_P0  27
#define DEFAULT_P1  73
#define DEFAULT_MU  64
#define DEFAULT_AX  1270
#define DEFAULT_TAU 300ul

/*
   Creates the motor controller
*/
MotorCtrl::MotorCtrl(const uint8_t forwPin, const uint8_t backPin, MotorSensor& sensor) :
  _forwPin(forwPin),
  _backPin(backPin),
  _sensor(sensor),
  _automatic(true),
  _ax(DEFAULT_AX),
  _p0Forw(DEFAULT_P0),
  _p1Forw(DEFAULT_P1),
  _muForw(DEFAULT_MU),
  _p0Back(-DEFAULT_P0),
  _p1Back(-DEFAULT_P1),
  _muBack(DEFAULT_MU) {
}

/*
   Initializes motor controller
*/
void MotorCtrl::begin() {
  pinMode(_forwPin, OUTPUT);
  pinMode(_backPin, OUTPUT);
  _sensor.begin();
}

/*
   Sets the configuration parameters
   [
         muForw, muBack, ax
         p0Forw, p1Forw,
         p0Back, p1Back,
   ]
*/
void MotorCtrl::config(const int* parms) {
  _muForw = parms[0];
  _muBack = parms[1];
  _ax = parms[2];
  _p0Forw = parms[3];
  _p1Forw = parms[4];
  _p0Back = parms[5];
  _p1Back = parms[6];
}

/*
   Polls motor controller
*/
void MotorCtrl::polling(const unsigned long timestamp) {
  _sensor.polling(timestamp);
  const long dt = (long)(timestamp - _prevTimestamp);
  if (_automatic && dt > MIN_INTERVAL) {
    _prevTimestamp = timestamp;

    // Computes the power
    const int realSpeed = round(_sensor.pps());

    DEBUG_PRINT("//   _speed: ");
    DEBUG_PRINT(_speed);
    DEBUG_PRINT(", dt: ");
    DEBUG_PRINT(dt);
    DEBUG_PRINT(", realSpeed: ");
    DEBUG_PRINT(realSpeed);
    DEBUG_PRINTLN();
    int pwr = 0;
    if (_speed == 0) {
      // Halt the motor
      pwr = 0;
    } else if (_speed > 0) {
      // Move forward
      const int dP = clip((long)_muForw * (_speed - realSpeed), (long) - _ax, (long)_ax) * dt / SCALE;
      const int requiredPwr = _power + dP;
      const int pth = realSpeed == 0 ? _p1Forw : _p0Forw;
      pwr = clip(requiredPwr, pth, MAX_POWER);

      DEBUG_PRINT(F("//   _power: "));
      DEBUG_PRINT(_power);
      DEBUG_PRINT(F(", _mu: "));
      DEBUG_PRINT(_muForw);
      DEBUG_PRINT(F(", req: "));
      DEBUG_PRINT(requiredPwr);
      DEBUG_PRINT(F(", pth: "));
      DEBUG_PRINT(pth);
      DEBUG_PRINTLN();

    } else  {
      // Move backward
      const int dP = clip((long)_muBack * (_speed - realSpeed), (long) - _ax, (long)_ax) * dt / SCALE;
      const int requiredPwr = _power + dP;
      const int pth = realSpeed == 0 ? _p1Back : _p0Back;
      pwr = clip(requiredPwr, -MAX_POWER, pth);

      DEBUG_PRINT(F("//   _power: "));
      DEBUG_PRINT(_power);
      DEBUG_PRINT(F(", _mu: "));
      DEBUG_PRINT(_muBack);
      DEBUG_PRINT(F(", req: "));
      DEBUG_PRINT(requiredPwr);
      DEBUG_PRINT(F(", pth: "));
      DEBUG_PRINT(pth);
      DEBUG_PRINTLN();

    }
    DEBUG_PRINT("//   post _power: ");
    DEBUG_PRINT(pwr);
    DEBUG_PRINTLN();
    power(pwr);
  }
}

/*
   Applies the power to the motor
   @param pwr the power -255 ... 255
*/
void MotorCtrl::power(const int pwr) {
  // Applies the power
  _power = pwr;
  if (_power == 0) {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, 0);
  } else if (_power > 0) {
    analogWrite(_forwPin, _power);
    analogWrite(_backPin, 0);
  } else {
    analogWrite(_forwPin, 0);
    analogWrite(_backPin, -_power);
  }
  _sensor.direction(_power);
}

/*
   Interrupt service routine for motor sensor
*/
static void ARDUINO_ISR_ATTR speedSensorChanged(void* arg) {
  MotorSensor* sensor = static_cast<MotorSensor*>(arg);
  sensor->update();
}

/*
  Creates the motor sensor
*/
MotorSensor::MotorSensor(const uint8_t sensorPin) :
  _sensorPin(sensorPin),
  _direction(0) {
}

/*
  Initializes motor sensor
*/
void MotorSensor::begin() {
  pinMode(_sensorPin, INPUT);
  _speedometer.reset(millis());
  attachInterruptArg(_sensorPin, &speedSensorChanged, this, CHANGE);
}

/*
   Resets the motor sensor
*/
void MotorSensor::reset(const unsigned long timestamp) {
  _direction = 0;
  _pulses = 0;
  _speedometer.reset(timestamp);
}

/*
  Sets the direction of motor
  @param direction > 0 if forward, < 0 if backward
*/
void MotorSensor::direction(const int direction) {
  DEBUG_PRINT("// MotorSensor::setDirection ");
  DEBUG_PRINT(direction);
  DEBUG_PRINTLN();

  _direction = direction;
  if (direction == 0) {
    _speedometer.reset(millis());
  }
}

/*
  Polls the motoro sensor
*/
void MotorSensor::polling(const unsigned long clockTime) {
  // Computes the dPulse
  const long pulses = _pulses;
  const long dPulses = pulses - _lastPulses;
  _lastPulses = pulses;
  update(dPulses, clockTime);

#ifdef DEBUG_POLLING
  DEBUG_PRINT("// MotorSensor::polling ");
  DEBUG_PRINT(_filter.value());
  DEBUG_PRINTLN();
#endif
}

/*
   Updates the pulse counter (called by interupt service routine)
*/
void MotorSensor::update(void) {
  if (_direction > 0) {
    _pulses++;
  } else if (_direction < 0) {
    _pulses--;
  }
}

/*
  Updates the motor sensor
  Computes the speed and invokes the callback function
*/
void MotorSensor::update(const int dPulses, const unsigned long clockTime) {
#ifdef DEBUG_SPEEDOMETER
  DEBUG_PRINT("// MotorSensor::update dPulse:");
  DEBUG_PRINT(dPulses);
  DEBUG_PRINT(", _pulses ");
  DEBUG_PRINT(_pulses);
  DEBUG_PRINTLN();
#endif

  _speedometer.update(clockTime, dPulses);
  if (_onSample != NULL && dPulses != 0) {
    _onSample(_context, dPulses, clockTime, *this);
  }
}

/*
   Creates the speedometer
*/
Speedometer::Speedometer(): _tau(DEFAULT_TAU) {
}

/*
   Updates the speedometer
   @param clockTime the instant
   @param dpulse the number of pulses
*/
void Speedometer::update(const unsigned long clockTime, const int dpulse) {
  const unsigned long dt = clockTime - _prevTime;
  if (dt > _tau) {
    _pps = 1000.0 * dpulse / dt;
  } else {
    _pps = (1000.0 * dpulse + _pps * (_tau - dt)) / _tau;
  }
#ifdef DEBUG_SPEEDOMETER
  DEBUG_PRINT("// Speedometer::update dpulse:");
  DEBUG_PRINT(dpulse);
  DEBUG_PRINT(", dt: ");
  DEBUG_PRINT(dt);
  DEBUG_PRINT(", tau: ");
  DEBUG_PRINT(_tau);
  DEBUG_PRINT(", pps: ");
  DEBUG_PRINTF(_pps, 3);
  DEBUG_PRINTLN();
#endif
  _prevTime = clockTime;
}

void Speedometer::reset(const unsigned long timestamp) {
  _pps = 0;
  _prevTime = timestamp;
}
