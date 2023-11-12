#ifndef SR04_h
#define SR04_h

#include "Arduino.h"
#include "Timer.h"

/*
   ASynchronous SR04
*/
class SR04Class {
  public:
    SR04Class(const uint8_t triggerPin, const uint8_t echoPin);

    // Sets a single interval
    void begin(void);

    // Starts the sampling
    void start(void);

    // Stops the sampling
    void stop(void);

    // Sets the callback
    void onSample(void (*callback)(void* context, const unsigned long echoTime), void* context = NULL) {
      _context = context;
      _onSample = callback;
    }

    // Polls the timer
    void polling(const unsigned long clockTime = millis());

  private:
    uint8_t _triggerPin;
    uint8_t _echoPin;
    int _validSamplesNum;
    int _falseSamplesNum;
    int _samplesNum;
    void (*_onSample)(void*, const unsigned long);

    boolean _armed;
    unsigned long _lastMeasure;
    unsigned long _duration;
    void* _context;

    void measure(const unsigned long t0);
};

#endif
