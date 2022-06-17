#ifndef AsyncServo_h
#define AsyncServo_h

#include "Arduino.h"
#include "Timer.h"
#include <Servo.h>

/*
   Multiplexer
*/
class AsyncServo {
  public:
    AsyncServo();
    void attach(byte pin);
    void onReached(void (*callback)(void *context, byte angle), void* context = NULL);
    void angle(byte value);
    void polling(unsigned long clockTime = millis());
    void offset(const int value);


    byte angle() const {
      return _angle;
    }

  private:
    Servo _servo;
    Timer _timer;
    void (*_onReached)(void *context, byte angle);
    byte _angle;
    void *_context;
    int _offset;

    static void _handleTimeout(void *, unsigned long);
};

#endif
