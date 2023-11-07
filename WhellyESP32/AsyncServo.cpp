#include "pins.h"
#include "AsyncServo.h"

//#define DEBUG
#include "debug.h"
#define MILLIS_PER_DEG  (180ul / 60)
#define MIN_INTERVAL    1ul

/*
   Creates the asynchonous servo controller
*/
AsyncServoClass::AsyncServoClass() {
  _timer.onNext(_handleTimeout, this);
}

/*
   Moves servo to the given angle
   @param value angle (DEG)
*/
void AsyncServoClass::angle(const int value) {
  _timer.stop();
  int da = abs(value - _angle);
  _angle = value;
  if (da != 0) {
    int wr = (int) value + _offset;
    DEBUG_PRINT("// AsyncServoClass::angle ");
    DEBUG_PRINT(wr);
    DEBUG_PRINTLN();
    _servo.write(wr);
  }
  _reached = false;
  _timer.interval(max(da * MILLIS_PER_DEG, MIN_INTERVAL));
  _timer.start();
}

/*
   Handles timeout for position reached
*/
void AsyncServoClass::_handleTimeout(void * context, unsigned long) {
  AsyncServoClass* servo = (AsyncServoClass*) context;
  servo->_reached = true;
  if (servo->_onReached != NULL) {
    servo->_onReached(servo->_context, servo->_angle);
  }
}

/*
   Polls the controller
   @param clockTime the current time instant
*/
void AsyncServoClass::polling(unsigned long clockTime) {
  _timer.polling(clockTime);
}
