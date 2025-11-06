#include "Timer.h"

/**
  * Starts the timer
  *
  * @param timeout the next event instant
  */
void Timer::start(unsigned long timeout) {
  _counter = 0;
  _next = timeout;
  _running = true;
}

/**
  * Starts the timer
  */
void Timer::start(void) {
  start(millis() + _interval);
}

/**
  * Stops the timer
  */
void Timer::stop(void) {
  _running = false;
}

/**
  * Restarts the timer
  */
void Timer::restart(void) {
  if (_running) {
    _next = millis() + _interval;
  }
}

/**
  * Polls the timer
  */
void Timer::polling(unsigned long clockTime) {
  if (_running) {
    if (clockTime >= _next) {
      unsigned long counter = _counter;
      _counter++;
      if (!_continuous) {
        stop();
      } else {
        _next += _interval;
      }
      if (_onNext != NULL) {
        _onNext(_context, counter);
      }
    }
  }
}
