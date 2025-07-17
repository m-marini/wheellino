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

#ifndef MotorCtrl_h
#define MotorCtrl_h

/*
  Speedometer measures the speed
*/
class Speedometer {
public:
  /*
       Creates the speedometer
    */
  Speedometer(void);

  /*
       Updates the speedometer data
       @param time the instant
       @param step the number of step
    */
  void update(const unsigned long time, const int step);

  /*
       Resets the speedometer
    */
  void reset(const unsigned long timestamp);

  /*
       Sets the tau parameter
    */
  void tau(const unsigned long tau) {
    _tau = tau;
  }

  /*
       Returns the speed (pps)
    */
  const float pps(void) const {
    return _pps;
  }

  /**
       Returns the tau parameter
    */
  const unsigned long tau(void) const {
    return _tau;
  }

private:
  float _pps;
  unsigned long _tau;
  unsigned long _prevTime;
};

/*
   Motor sensor measures the movement of motor
*/
class MotorSensor {
public:
  /*
       Creates the sensor
    */
  MotorSensor(const uint8_t sensorPin);

  /*
       Initializes the sensor
    */
  void begin(void);

  /*
       Polls the sensor
    */
  void polling(const unsigned long clockTime = millis());

  /*
       Updates the pulse counter
    */
  void update(void);

  /*
       Sets the direction
       @param speed the speed direction (> 0 forward, < 0 backward)
    */
  void direction(const int speed);

  /*
       Resets the sensor
    */
  void reset(const unsigned long timestamp);

  /*
       Sets the tau parameter of sensor
    */
  void tau(const unsigned long tau) {
    _speedometer.tau(tau);
  }

  /*
       Sets the callback on sample
    */
  void onSample(void (*callback)(void* context, const int dPulse, const unsigned long clockTime, MotorSensor& sensor), void* context = NULL) {
    _onSample = callback;
    _context = context;
  }

  /*
       Returns the pulses
    */
  const long pulses() const {
    return _pulses;
  }

  /*
       Returns the speed (pps)
    */
  const float pps(void) const {
    return _speedometer.pps();
  }

  /**
       Returns the tau parameter
    */
  const unsigned long tau(void) const {
    return _speedometer.tau();
  }

private:
  uint8_t _sensorPin;
  volatile long _pulses;
  long _lastPulses;
  int _direction;
  void* _context;
  void (*_onSample)(void*, const int, const unsigned long clockTime, MotorSensor&);
  Speedometer _speedometer;

  void update(const int dPulse, const unsigned long clockTime);
};

/*
  Motor ontroller
*/
class MotorCtrl {

public:
  /*
       Creates the motor controller
    */
  MotorCtrl(const uint8_t forwPin, const uint8_t backPin, MotorSensor& sensor);

  /*
       Initializes the motor controller
    */
  void begin(void);

  /*
       Sets the motor speed
    */
  void speed(const int value);

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
  void tcsConfig(const int* parms);

  /**
       Sets the feedback parameters
       [
         muForw, muBack
       ]
      muForw, muBack: delta power by delta speed by dt (power correction for speed difference)
    */
  void muConfig(const long* parms);

  /*
       Returns the speed
    */
  const int speed() const {
    return _speed;
  }

  void power(const int pwr);

  /*
       Polls the motor controller
    */
  void polling(const unsigned long timestamp = millis());

  /*
       Sets automatic motor drive
       @param automatic true if automatic motor drive
    */
  void automatic(const boolean automatic) {
    _automatic = automatic;
  }

  /*s
       Returns the sensor
    */
  MotorSensor& sensor(void) const {
    return _sensor;
  }

  /**
       Returns the power applied to motor
    */
  const int power(void) const {
    return _power;
  }

  /**
       Returns the p0Forw parameter
    */
  const int p0Forw(void) const {
    return _p0Forw;
  }

  /**
       Returns the p1Forw parameter
    */
  const int p1Forw(void) const {
    return _p1Forw;
  }

  /**
       Returns the pxForw parameter
    */
  const int pxForw(void) const {
    return _pxForw;
  }

  /**
       Returns the muForw parameter
    */
  const long muForw(void) const {
    return _muForw;
  }

  /**
       Returns the p0Back parameter
    */
  const int p0Back(void) const {
    return _p0Back;
  }

  /**
       Returns the p1Back parameter
    */
  const int p1Back(void) const {
    return _p1Back;
  }

  /**
       Returns the pxBack parameter
    */
  const int pxBack(void) const {
    return _pxBack;
  }

  /**
       Returns the muBack parameter
    */
  const long muBack(void) const {
    return _muBack;
  }

  /**
       Returns the ax parameter
    */
  const int ax(void) const {
    return _ax;
  }


  /**
       Returns the ax parameter
    */
  const int alpha(void) const {
    return _alpha;
  }

private:
  const uint8_t _forwPin;
  const uint8_t _backPin;
  boolean _automatic;
  MotorSensor& _sensor;
  int _p0Forw;
  int _p1Forw;
  int _pxForw;
  long _muForw;
  int _p0Back;
  int _p1Back;
  int _pxBack;
  long _muBack;
  int _alpha;
  int _ax;
  unsigned long _prevTimestamp;
  int _speed;
  int _power;

  const long asr(const long dp, const long dt) const;
};

#endif
