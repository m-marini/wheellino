#ifndef Timer_h
#define Timer_h

#include "Arduino.h"

typedef void (*TimerCallback_t)(void* context, const unsigned long counter);

/*
   Asynchronous timer
*/
class Timer {

private:
  bool _continuous;
  unsigned long _interval;
  TimerCallback_t _onNext;
  void* _context;

  unsigned long _next;
  unsigned long _counter;
  boolean _running;

public:
  /**
  * Creates the timer
  */
  Timer(){};

  /**
  * Sets the interval
  *
  * @param interval the interval (ms)
  */
  void interval(const unsigned long interval) {
    _interval = interval;
  }

  /**
  * Sets true if continuos events
  */
  void continuous(const boolean cont) {
    _continuous = cont;
  }

  /**
  * Starts the timer
  */
  void start(void);

  /**
  * Starts the timer
  *
  * @param timeout the next event instant
  */
  void start(const unsigned long timeout);

  /**
  * Stops the timer
  */
  void stop(void);

  /**
  * Restarts the timer
  */
  void restart(void);

  /**
  * Sets the callback
  */
  void onNext(TimerCallback_t callback, void* context = NULL) {
    _onNext = callback;
    _context = context;
  }

  /**
  * Polls the timer
  */
  void polling(const unsigned long clockTime = millis());

  /**
  * Returns the next event instant
  */
  const unsigned long next(void) const {
    return _next;
  }

  /**
  * Returns the interval
  */
  const unsigned long interval(void) const {
    return _interval;
  }

  /**
  * Returns true if timer is not expired (is timing)
  */
  const bool isRunning(void) const {
    return _running;
  }
};

#endif
