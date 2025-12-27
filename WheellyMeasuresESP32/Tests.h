#ifndef Tests_h
#define Tests_h

#include "Arduino.h"
#include "MotorCtrl.h"
#include "Contacts.h"
#include "Timer.h"

typedef void (*TestCallback_t)(void* context);

/**
   Test of the motor
   Creates a power ramp up until movement
   then runs for constant power
   then ramp down to stop
*/
class MotorTest {
private:
  int _maxPower;
  unsigned _accelerationPower;
  unsigned _decelerationPower;
  unsigned long _accelerationInterval;
  unsigned long _decelerationInterval;

  MotorCtrl& _motorCtrl;
  bool _isTesting;
  bool _isWaitingForEnd;
  bool _isSteppingUp;
  int _power;
  unsigned long _nextStepInstant;

  TestCallback_t _onPowerChange;
  void* _context;

  /**
  Change power
  */
  void changePower(void);

public:
  /**
       Creates the motor test
    */
  MotorTest(MotorCtrl& motor);

  /**
  Returns true if test is running
  */
  const bool testing(void) const {
    return _isTesting;
  }

  /**
  Returns the current power test
  */
  const int power(void) const {
    return _power;
  }

  /**
  Sets the power change callback
  */
  void onPowerChange(TestCallback_t callback, void* context = 0) {
    _onPowerChange = callback;
    _context = context;
  }

  /**
       Starts the test
    */
  void start(const unsigned long t0,
             const int maxPower,
             const unsigned long accelerationInterval,
             const unsigned accelerationPower,
             const unsigned long decelerationInterval,
             const unsigned decelerationPower);

  /**
       Stops the test
    */
  void stop(void);

  /*
  Poolling the test
  */
  void pooling(const unsigned long t0);
};

#endif
