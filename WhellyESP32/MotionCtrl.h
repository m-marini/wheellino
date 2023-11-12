#ifndef Motion_h
#define Motion_h

#include "MotorCtrl.h"
#include "Timer.h"

#define PULSES_PER_ROOT     40
#define WHEEL_DIAMETER      0.067f

#define DISTANCE_PER_PULSE  (WHEEL_DIAMETER * PI / PULSES_PER_ROOT)

/*
  Motion sensor detects the movement from left and right motor sensors
*/
class MotionSensor {
  public:
    /*
       Creates the sensor
    */
    MotionSensor(const uint8_t leftPin, const uint8_t rightPin);

    /*
       Initializes the sensor
    */
    void begin(void);

    /*
       Polls the sensor
    */
    void polling(const unsigned long clockTime);

    /*
       Set the motor directions
       @param leftForward left speed
       @param rightForward left speed
    */
    void direction(const int leftForward, const int rightForward);

    /*
       Set the callback on change
    */
    void setOnChange(void (*callback)(void* context, const unsigned long clockTime, MotionSensor& sensor), void* context = NULL);

    /*
       Sets the tau parameter
    */
    void tau(const unsigned long tau);

    /*
       Sets the direction angle
    */
    void angle(const int angle) {
      _angle = angle;
    }
    /*
       Resets the sensor
    */
    void reset(const unsigned long timestamp);

    /*
       Sets left pulses
       @param dPulse the number of pulse
    */
    void setLeftPulses(const int dPulse) {
      _dl = dPulse;
    }

    /*
       Sets left pulses
       @param dPulse the number of pulse
    */
    void setRightPulses(const int dPulse) {
      _dr = dPulse;
    }

    /*
       Returns the direction (DEG)
    */
    const int angle(void) const {
      return _angle;
    }

    /*
       Returns the x position coordinate (pulses)
    */
    const float xPulses(void) const {
      return _xPulses;
    }

    /*
       Returns the y position coordinate (pulses)
    */
    const float yPulses(void) const {
      return _yPulses;
    }

    /*
       Returns the right pulses counter
    */
    const long rightPulses(void) const {
      return _rightSensor.pulses();
    }


    /*
       Returns the left pulses counter
    */
    const long leftPulses(void) const {
      return _leftSensor.pulses();
    }

    /*
       Returns the left speed (pps)
    */
    const float leftPps(void) const {
      return _leftSensor.pps();
    }

    /*
       Returns the right speed (pps)
    */
    const float rightPps(void) const {
      return _rightSensor.pps();
    }

    /*
       Updates the angle
    */
    void updateAngle(const boolean updateAngle) {
      _updateAngle = updateAngle;
    }

    /*
       Returns the left motor sensor
    */
    MotorSensor& leftSensor() {
      return _leftSensor;
    }

    /*
       Returns the right motor sensor
    */
    MotorSensor& rightSensor() {
      return _rightSensor;
    }

    const unsigned long tau(void) const;

  private:
    MotorSensor _leftSensor;
    MotorSensor _rightSensor;
    boolean _updateAngle;
    int _angle;
    float _xPulses;
    float _yPulses;
    int _dl;
    int _dr;
    void (*_onChange)(void*, const unsigned long, MotionSensor&);
    void* _context;

    void update(const unsigned long clockTime);
};

/*
   Handles the two motors power (left and right) to drive for selected speed and direction
*/
class MotionCtrlClass {
  public:
    /*
       Create the motion controller
    */
    MotionCtrlClass(const uint8_t leftForwPin, const uint8_t leftBackPin,
                    const uint8_t rightForwPin, uint8_t rightBackPin,
                    const uint8_t leftSensorPin, const uint8_t rightSensorPin);

    /*
       Initializes the motion controller
    */
    void begin(void);

    /*
       Polls the controller
    */
    void polling(const unsigned long clockTime = millis());

    /*
       Resets the controller
    */
    void reset(const unsigned long timestamp);

    /*
       Handles the motion
    */
    void handleMotion(unsigned long clockTime);

    /*
       Moves to given direction at given speed
       @param direction the direction (DEG)
       @param speed the speed
    */
    void move(const int direction, const int speed);

    /*
       Sets the configuration parameters
       [moveRotThreshold, minRotRange, maxRotRange, maxRotPps ]
    */
    void configController(const int *p);

    /*
       Sets the tau parameter
    */
    void tau(const unsigned long tau) {
      _sensors.tau(tau);
    }

    /*
       Halts the motion
    */
    void halt(void);

    /*
       Returns the x position pulses
    */
    const float xPulses(void) const {
      return _sensors.xPulses();
    }

    /*
       Returns the y position pulses
    */
    const float yPulses(void) const {
      return _sensors.yPulses();
    }

    /*
       Returns the direction angle (DEG)
    */
    const int angle(void) const {
      return _sensors.angle();
    }

    /*
       Returns the left speed (pps)
    */
    const float leftPps(void) const {
      return _sensors.leftPps();
    }

    /*
       Returns the left speed (pps)
    */
    const float rightPps(void) const {
      return _sensors.rightPps();
    }

    /*
       Returns true if moving forward
    */
    const boolean isForward(void) const;

    /*
       Returns true if moving backward
    */
    const boolean isBackward(void) const;

    /*
       Returns true if halt
    */
    const boolean isHalt(void) const {
      return _halt;
    }

    /*
       Sets current direction angle (DEG)
       (used with MPU)
    */
    void angle(const int angle) {
      _sensors.angle(angle);
    }

    /*
       Returns the exptected speed (pps)
    */
    const int speed(void) const {
      return _speed;
    }

    /*
       Returns the expected direction (DEG)
    */
    const int direction(void) const {
      return _direction;
    }

    /*
       Returns the left motor controller
    */
    MotorCtrl& leftMotor(void) {
      return _leftMotor;
    }

    /*
       Returns the right motor controlller
    */
    MotorCtrl& rightMotor(void) {
      return _rightMotor;
    }

    /**
       Returns the minRotRange parameter
    */
    const int minRotRange(void) const {
      return _minRotRange;
    }

    /**
       Returns the maxRotRange parameter
    */
    const int maxRotRange(void) const {
      return _maxRotRange;
    }

    /**
       Returns the maxRotPps parameter
    */
    const int maxRotPps(void) const {
      return _maxRotPps;
    }

    /**
       Returns the motion sensors
    */
    MotionSensor& sensors() {
      return _sensors;
    }

  private:
    MotorCtrl _leftMotor;
    MotorCtrl _rightMotor;
    MotionSensor _sensors;
    Timer _stopTimer;
    Timer _checkTimer;
    int _minRotRange;
    int _maxRotRange;
    int _maxRotPps;

    int _direction;
    int _speed;
    boolean _halt;

    unsigned long _prevTime;

    void motorSpeed(const int left, const int right);
};

#endif
