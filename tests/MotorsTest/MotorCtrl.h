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
private:
  float _pps;
  unsigned long _tau;
  unsigned long _prevTime;

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
};

/*
   Motor sensor measures the movement of motor
*/
class MotorSensor {
private:
  uint8_t _sensorPin;
  volatile long _pulses;
  long _lastPulses;
  int _direction;
  void* _context;
  void (*_onSample)(void*, const int, const unsigned long clockTime, MotorSensor&);
  Speedometer _speedometer;

  void update(const int dPulse, const unsigned long clockTime);

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
};

/**
  Traction control system parameters
*/
typedef struct {
  /**
    Forward increment threshold voltage
  */
  int fi0;
  /**
    Forward increment voltage
  */
  int fix;
  /**
    Forward decrement threshold voltage
  */
  int fd0;
  /**
    Forward decrement voltage
  */
  int fdx;
  /**
    Backward increment threshold voltage
  */
  int bi0;
  /**
    Backward increment voltage
  */
  int bix;
  /**
    Backward decrement threshold voltage
  */
  int bd0;
  /**
    Backward decrement voltage
  */
  int bdx;
  /**
    Forward feedback factor
  */
  long muForw;
  /**
    Backward feedback factor
  */
  long muBack;
  /**
    Voltage integration facotr
  */
  int alpha;
  /**
    ASR factor
  */
  int ax;
} tcsParams_t;

/*
  Motor ontroller
*/
class MotorCtrl {
private:
  const uint8_t _forwPin;
  const uint8_t _backPin;
  boolean _automatic;
  MotorSensor& _sensor;
  tcsParams_t _tcs;
  unsigned long _prevTimestamp;
  int _speed;
  int _voltage;
  int _supply;
  int _pwm;

  /**
  * Returns the ASR  (Anti Slip Regulation) voltage
  */
  const long asr(const long dv, const long dt) const;

  /**
    Set the motor voltage
  */
  void voltage(const int v);

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
    Sets the supply voltage
  */
  void supply(const int supply) {
    _supply = supply;
  }

  /**
       Sets the tcs parameters
    */
  void tcs(const tcsParams_t& tcs) {
    _tcs = tcs;
  }

  /**
       Returns the tcs parameters
    */
  const tcsParams_t& tcs(void) const {
    return _tcs;
  }

  /*
       Returns the speed
    */
  const int speed(void) const {
    return _speed;
  }

  /**
    Return the pwm
  */
  const int pwm(void) const {
    return _pwm;
  }

  /**
    Returns the voltage
  */
  const int voltage(void) const {
    return _voltage;
  }

  /**
   Sets the pwm factor
  */
  void pwm(const int pwm);

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
};

#endif
