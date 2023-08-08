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
    void interval(const unsigned long interval) {
      _interval = interval;
    }

    // Sets true if continuos events
    void continuous(const boolean cont)  {
      _continuous = cont;
    }

    // Starts the timer
    void start(void);

    // Starts the timer
    void start(const unsigned long timeout);

    // Stops the timer
    void stop(void);

    // Restarts the timer
    void restart(void);

    // Returns true if timer is not expired (is timing)
    const bool isRunning(void) const {
      return _running;
    }

    // Returns the interval
    const unsigned long interval(void) const {
      return _interval;
    }

    // Sets the callback
    void onNext(void (*callback)(void* context, const unsigned long counter), void* context = NULL);

    // Polls the timer
    void polling(const unsigned long clockTime = millis());

    const unsigned long next(void) const {
      return _next;
    }

  private:
    bool _continuous;
    unsigned long _interval;
    void (*_onNext)(void*, const unsigned long);
    void *_context;

    unsigned long _next;
    unsigned long _counter;
    boolean _running;
};

#endif
