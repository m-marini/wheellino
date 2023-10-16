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
    const float pps() const {
      return _speedometer.pps();
    }

  private:
    uint8_t       _sensorPin;
    volatile long _pulses;
    long          _lastPulses;
    int           _direction;
    void          *_context;
    void          (*_onSample)(void*, const int, const unsigned long clockTime, MotorSensor&);
    Speedometer   _speedometer;

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
    void speed(const int value) {
      _speed = value;
    }

    /*
       Sets the configuration parameters
       [
         muForw, muBack, ax
         p0Forw, p1Forw,
         p0Back, p1Back,
       ]
       p0Forw, p0Back: power for dynamic friction (min power for moving motor)
       p1Forw, p1Back: power for static friction (min power for stopped motor)
       muForw, muBack: delta power by delta speed by dt (power correction for speed difference)
       ax: maximum delta power by dt
    */
    void config(const int* parms);

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

    /*
       Returns the sensor
    */
    MotorSensor& sensor(void) {
      return _sensor;
    }

  private:
    const uint8_t _forwPin;
    const uint8_t _backPin;
    boolean       _automatic;
    MotorSensor&  _sensor;
    unsigned long _prevTimestamp;
    int _ax;
    int _speed;
    int _power;
    int _p0Forw;
    int _p1Forw;
    int _muForw;
    int _p0Back;
    int _p1Back;
    int _muBack;
};

#endif
