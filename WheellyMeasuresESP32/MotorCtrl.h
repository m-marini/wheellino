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

class MotorSensor;

typedef void (*onSampleCallback_t)(void* context, const int pulses, const unsigned long t, MotorSensor* sensor);

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
  onSampleCallback_t _onSample;
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
  void onSample(const onSampleCallback_t callback, void* context = NULL) {
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
  long asr;
  long maxPulseInterval;
  int lambdaFactor;
  long delayedInterval;
} tcsParams_t;

/*
  Motor controller states
*/
enum motorControllerStatus_t {
  MOTOR_HALT = 0,
  MOTOR_STARTING = 1,
  MOTOR_RUNNING = 2,
};

/*
  Motor ontroller
*/
class MotorCtrl {
private:
  const uint8_t _forwPin;
  const uint8_t _backPin;
  MotorSensor _sensor;
  tcsParams_t _tcs;
  motorControllerStatus_t _status;
  bool _automatic;
  bool _asr;
  int _speed;
  int _pwm;
  int _targetPwm;
  long _fPwmFactor;
  long _bPwmFactor;

  int _pulseInterval;              // required pulse interval
  unsigned long _startTime;        // start phase instant
  unsigned long _lastPwmTime;      // last pwm set instant
  unsigned long _lastPulsesTime;   // last notified pulses instant
  unsigned long _lastDelayedTime;  // last pulse delayed check instant

  void* _context;
  onSampleCallback_t _onSample;

  /*
    Handling polling during starting status
  */
  void starting(const unsigned long t);

  /*
    Handling polling during running status
  */
  void running(const unsigned long t);

  /*
  Applies the pwm to the motor
  */
  void applyPwm(const int pwm, const unsigned long t);

  /*
  Applies the pwm instant to the motor
  */
  void applyInstantPwm(const int pwm, const unsigned long t) {
    _targetPwm = pwm;
    applyPwm(pwm, t);
  }

  /*
  Returns the target pwm
  */
  const int computePwm(void) const;

  /*
  Update pwm factor
  @pulseWidth the pulse width (ms)
  */
  void updatePwmFactor(const long pulseWidth);

  /*
    Handles pulses from sensor
  */
  void onPulses(const int pulses, const unsigned long t);

  // Friend functions
  friend void handlePulses(void* context, const int pulses, const unsigned long t, MotorSensor* sensor);

public:
  /*
       Creates the motor controller
    */
  MotorCtrl(const uint8_t forwPin, const uint8_t backPin, const uint8_t sensorPin);

  /*
    Returns the pulses
  */
  const long pulses() const {
    return _sensor.pulses();
  }

  /*
    Returns the speed (pps)
  */
  const float pps(void) const {
    return _sensor.pps();
  }

  /*
    Returns the sensor tau parameter (pps)
  */
  const unsigned long tau(void) const {
    return _sensor.pps();
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
    Return the pwm
  */
  const int targetPwm(void) const {
    return _targetPwm;
  }

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
    */
  void tcs(const tcsParams_t& tcs) {
    _tcs = tcs;
  }
  /**
   Sets the pwm factor
  */
  void pwm(const int pwm, const unsigned long t = millis());

  /*
       Polls the motor controller
    */
  void polling(const unsigned long timestamp);

  /*
       Sets automatic motor drive
       @param automatic true if automatic motor drive
    */
  void automatic(const boolean automatic) {
    _automatic = automatic;
  }

  /*
    Sets the direction
    @param direction the direction (> 0 forward, < 0 backward)
    */
  void sensorDirection(const int direction) {
    _sensor.direction(direction);
  }

  /*
       Sets the callback on sample
    */
  void onSample(const onSampleCallback_t callback, void* context = NULL) {
    _onSample = callback;
    _context = context;
  }

  /*
    Sets sensor tau paramiter
  */
  void tau(const unsigned long tau) {
    _sensor.tau(tau);
  }

  /*
    Reset the sensor
  */
  void reset(const unsigned long t) {
    _sensor.reset(t);
  }

  /*
    Set the anti-slip regulation
*/
  void asr(const bool active) {
    _asr = active;
  }
};

#endif
