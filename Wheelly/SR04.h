#ifndef SR04_h
#define SR04_h

#include "Arduino.h"
#include "Timer.h"

/*
   ASynchronous SR04
*/
class SR04 {
  public:
    SR04(byte triggerPin, byte echoPin);

    // Sets a single interval
    void begin();

    // Starts the sampling
    void start();

    // Stops the sampling
    void stop();

    // Sets the callback
    void onSample(void (*callback)(void* context, int distance), void* context = NULL);

    // Polls the timer
    void polling(unsigned long clockTime = millis());

  private:
    byte _triggerPin;
    byte _echoPin;
    void (*_onSample)(void*, int);

    byte _noMeasures;
    byte _noValidSamples;
    unsigned long _totalDuration;
    void* _context;
    Timer _timer;

    void _measure();
    void _send();
    static void _handleTimeout(void *context, unsigned long);
};

#endif
