/*
 * Copyright (c) 2023  Marco Marini, marco.marini@mmarini.org
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 *    END OF TERMS AND CONDITIONS
 *
 */

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


#define DEFAULT_P0 59
#define DEFAULT_P1 73
#define DEFAULT_PX 255
//#define DEFAULT_MU    0
#define DEFAULT_MU 20000
//#define DEFAULT_MU    10000
#define DEFAULT_AX 200
#define DEFAULT_ALPHA 25
#define DEFAULT_TAU 300ul

static unsigned long MIN_INTERVAL = 100ul;
static int MAX_POWER = 255;
static int MAX_SPEED = 120;
static long ASR_SCALE = 1000;
static long ALPHA_SCALE = 100;
static long FEEDBACK_SCALE = 1000000;

/*
   Creates the motor controller
*/
MotorCtrl::MotorCtrl(const uint8_t forwPin, const uint8_t backPin, MotorSensor& sensor)
  : _forwPin(forwPin),
    _backPin(backPin),
    _sensor(sensor),
    _automatic(true),
    _ax(DEFAULT_AX),
    _alpha(DEFAULT_ALPHA),
    _p0Forw(DEFAULT_P0),
    _p1Forw(DEFAULT_P1),
    _pxForw(DEFAULT_PX),
    _muForw(DEFAULT_MU),
    _p0Back(-DEFAULT_P0),
    _p1Back(-DEFAULT_P1),
    _pxBack(-DEFAULT_PX),
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

/**
   Sets the tcs parameters
   [
     p0Forw, p1Forw, pxForw
     p0Back, p1Back, pxBack
     ax, alpha
   ]
   p0Forw, p0Back: power for dynamic friction (min power for moving motor)
   p1Forw, p1Back: power for static friction (min power for stopped motor)
   pxForw, pxBack: max theoretical power to run max speed
   ax: asr acceleration value
   alpha: alpha mix value
*/
void MotorCtrl::tcsConfig(const int* parms) {
  _p0Forw = parms[0];
  _p1Forw = parms[1];
  _pxForw = parms[2];
  _p0Back = parms[3];
  _p1Back = parms[4];
  _pxBack = parms[5];
  _ax = parms[6];
  _alpha = parms[7];
}

/**
   Sets the feedback parameters
   [
     muForw, muBack
   ]
  muForw, muBack: delta power by delta speed by dt (power correction for speed difference)
*/
void MotorCtrl::muConfig(const long* parms) {
  _muForw = parms[0];
  _muBack = parms[1];
}

/**
   ASR function
*/
const long MotorCtrl::asr(const long dPower, const long dt) const {
  const long px = _ax * dt / ASR_SCALE;
  return clip(dPower, -px, px);
}

/*
   Sets the motor speed
*/
void MotorCtrl::speed(const int value) {
  if (value == 0
      || _speed < 0 && value > 0
      || _speed > 0 && value < 0) {
    power(0);
  }
  _speed = value;
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

    DEBUG_PRINT("// MotorCtrl::polling 0x");
    DEBUG_PRINTF((unsigned long)this, HEX);
    DEBUG_PRINTLN();
    DEBUG_PRINT("//   dt: ");
    DEBUG_PRINT(dt);
    DEBUG_PRINT(", _speed: ");
    DEBUG_PRINT(_speed);
    DEBUG_PRINT(", realSpeed: ");
    DEBUG_PRINT(realSpeed);
    DEBUG_PRINT(", _power: ");
    DEBUG_PRINT(_power);
    DEBUG_PRINTLN();
    int pwr = 0;
    if (_speed > 0) {
      // Move forward
      const int pth = realSpeed == 0 ? _p1Forw : _p0Forw;
      const long fx = pth + (long)(_pxForw - pth) * _speed / MAX_SPEED;
      const int dpt = _alpha * (fx - _power) / ALPHA_SCALE;
      DEBUG_PRINT("//   fx: ");
      DEBUG_PRINT(fx);
      DEBUG_PRINT(", dpt: ");
      DEBUG_PRINT(dpt);
      DEBUG_PRINTLN();

      const long dpf = _muForw * (_speed - realSpeed) * dt / FEEDBACK_SCALE;
      DEBUG_PRINT("//   dpf: ");
      DEBUG_PRINT(dpf);
      DEBUG_PRINTLN();

      const int dp = asr(dpt + dpf, dt);
      pwr = clip(_power + dp, pth, MAX_POWER);

      DEBUG_PRINT("//   dp: ");
      DEBUG_PRINT(dp);
      DEBUG_PRINT(", pth: ");
      DEBUG_PRINT(pth);
      DEBUG_PRINT(", pwr: ");
      DEBUG_PRINT(pwr);
      DEBUG_PRINTLN();

    } else if (_speed < 0) {
      // Move backward
      const int pth = realSpeed == 0 ? _p1Back : _p0Back;
      const long fx = pth - (long)(_pxBack - pth) * _speed / MAX_SPEED;
      const int dpt = _alpha * (fx - _power) / ALPHA_SCALE;
      DEBUG_PRINT("//   fx: ");
      DEBUG_PRINT(fx);
      DEBUG_PRINT(", dpt: ");
      DEBUG_PRINT(dpt);
      DEBUG_PRINTLN();

      const long dpf = _muBack * (_speed - realSpeed) * dt / FEEDBACK_SCALE;
      DEBUG_PRINT("//   dpf: ");
      DEBUG_PRINT(dpf);
      DEBUG_PRINTLN();

      const int dp = asr(dpt + dpf, dt);
      pwr = clip(_power + dp, -MAX_POWER, pth);

      DEBUG_PRINT("//   dp: ");
      DEBUG_PRINT(dp);
      DEBUG_PRINT(", pth: ");
      DEBUG_PRINT(pth);
      DEBUG_PRINT(", pwr: ");
      DEBUG_PRINT(pwr);
      DEBUG_PRINTLN();
    }
    DEBUG_PRINT("//   pwr: ");
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
MotorSensor::MotorSensor(const uint8_t sensorPin)
  : _sensorPin(sensorPin),
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
Speedometer::Speedometer()
  : _tau(DEFAULT_TAU) {
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
