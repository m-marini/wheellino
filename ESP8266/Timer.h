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

    // Sets the intervals sequence
    Timer& intervals(int noIntervals, unsigned long *intervals);

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

    // Sets the callback 
    Timer& onNext(void (*callback)(void* context, int interval, long cycles));

    // Polls the timer
    Timer& polling();

    unsigned long next() const {return _next;}

  private:
    bool _continuous;
    int _noIntervals;
    unsigned long _intervals[MAX_INTERVALS];
    void (*_onNext)(void* context, int interval, long cycles);
    
    unsigned long _next;
    int _interval;
    long _cycles;
    boolean _running;
    void *_context;
};

#endif
