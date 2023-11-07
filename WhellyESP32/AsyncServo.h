#ifndef AsyncServo_h
#define AsyncServo_h

#include "Arduino.h"
#include "Timer.h"
#include <Servo.h>

/*
   Asynchronous servo controller
*/
class AsyncServoClass {
  public:
    /*
       Creates the asynchronous servo controller
    */
    AsyncServoClass();

    /*
       Attaches the pin
    */
    void attach(const uint8_t pin) {
      _servo.attach(pin);
    }

    /*
       Sets callback on reached
    */
    void onReached(void (*callback)(void *context, const int angle), void* context = NULL) {
      _onReached = callback;
      _context = context;
    }
    /*
       Sets the angle
       @param angle the angle in (DEG)
    */
    void angle(const int value);

    /*
       Polls
    */
    void polling(const unsigned long clockTime = millis());

    /*
       Sets the offset
    */
    void offset(const int value) {
      _offset = value;
    }

    /*
       Returns the angle (DEG)
    */
    const int angle() const {
      return _angle;
    }

  private:
    Servo _servo;
    Timer _timer;
    void (*_onReached)(void *context, const int angle);
    int _angle;
    void *_context;
    int _offset;
    boolean _reached;

    static void _handleTimeout(void *, const unsigned long);
};

#endif
