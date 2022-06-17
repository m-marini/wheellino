#ifndef Timer_h
#define Timer_h

#include "Arduino.h"

#define MAX_INTERVALS 4

/*
 * ASynchronous timer
 */
class Timer {
  public:

    Timer();

    // Sets a single interval
    Timer& interval(unsigned long interval);

    // Sets true if continuos events
    Timer& continuous(boolean cont);

    // Starts the timer
    Timer& start(void *context = NULL);

    // Starts the timer
    Timer& start(void *context, unsigned long timeout);

    // Starts the timer
    Timer& start(unsigned long timeout) {return start(NULL, timeout);};

    // Stops the timer
    Timer& stop();
  
    // Restarts the timer
    Timer& restart();
  
    // Returns true if timer is not expired (is timing)
    bool isRunning() const {return _running;}

    // Returns the interval
    unsigned long interval() const {return _interval;}

    // Sets the callback 
    Timer& onNext(void (*callback)(void* context, unsigned long counter));

    // Polls the timer
    Timer& polling(unsigned long clockTime = millis());

    unsigned long next() const {return _next;}

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
