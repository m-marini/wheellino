#ifndef ProxySensor_h
#define ProxySensor_h

#include "Arduino.h"
#include <Servo.h>

/*
   Proxy sensor
*/
class ProxySensor {
  public:
    ProxySensor(const uint8_t servoPin, const uint8_t triggerPin, const uint8_t echoPin);

    /**
       Begins the sensor
    */
    void begin(void);

    /*
       Sets callback on reached
    */
    void onDataReady(void (*callback)(void *context, ProxySensor& sensor), void* context = NULL) {
      _onDataReady = callback;
      _context = context;
    }

    /*
       Sets the angle
       @param angle the angle in (DEG)
    */
    void direction(const int value, const unsigned long t0);

    /*
       Polls
    */
    void polling(const unsigned long t0);

    /*
       Sets the offset
    */
    void offset(const int value) {
      _offset = value;
    }

    /**
       Sets the measure interval
    */
    void interval(const unsigned long interval);

    /**
       Returns the instant of echo signal (ms)
    */
    const unsigned long echoTime(void) const {
      return _echoTime;
    }

    /**
       Returns the direction of echo signal (DEG)
    */
    const int echoDirection(void) const {
      return _echoDirection;
    }

    /**
       Returns the delay of echo (ns)
    */
    const unsigned long echoDelay(void) const {
      return _echoDelay;
    }

  private:
    const uint8_t _servoPin;
    const uint8_t _triggerPin;
    const uint8_t _echoPin;
    Servo _servo;
    int _direction;
    int _offset;
    unsigned long _interval;
    unsigned long _waitTime;
    unsigned long _resetTime;
    unsigned long _echoTime;
    unsigned long _echoDelay;
    int _echoDirection;
    int _noMeasures;
    int _noValidSamples;
    unsigned long _totalDuration;
    void (*_onDataReady)(void *, ProxySensor&);
    void *_context;
};

#endif
