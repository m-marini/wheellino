#ifndef Timer_h
#define Timer_h

#include "Arduino.h"

#define MAX_INTERVALS 4

/*
   ASynchronous timer
*/
class Timer {
  public:

    Timer() {};

    // Sets a single interval
    void interval(unsigned long interval) {
      _interval = interval;
    }

    // Sets true if continuos events
    void continuous(boolean cont)  {
      _continuous = cont;
    }


    // Starts the timer
    void start();

    // Starts the timer
    void start(unsigned long timeout);

    // Stops the timer
    void stop();

    // Restarts the timer
    void restart();

    // Returns true if timer is not expired (is timing)
    bool isRunning() const {
      return _running;
    }

    // Returns the interval
    unsigned long interval() const {
      return _interval;
    }

    // Sets the callback
    void onNext(void (*callback)(void* context, unsigned long counter), void* context = NULL);

    // Polls the timer
    void polling(unsigned long clockTime = millis());

    unsigned long next() const {
      return _next;
    }

  private:
    bool _continuous;
    unsigned long _interval;
    void (*_onNext)(void*, unsigned long);
    void *_context;

    unsigned long _next;
    unsigned long _counter;
    boolean _running;
};

#endif
