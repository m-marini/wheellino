#include "Timer.h"

/*
 * 
 */
Timer::Timer() {
}

/*
 * 
 */
Timer& Timer::continuous(bool cont) {
  _continuous = cont;
  return *this;
}

/*
 * 
 */
Timer& Timer::interval(unsigned long interval) {
  return intervals(1, &interval);
}

/*
 * 
 */
Timer& Timer::intervals(int noIntervals, unsigned long *intervals) {
  if (noIntervals >= 1) {
    if (noIntervals > MAX_INTERVALS) {
      noIntervals = MAX_INTERVALS;
    }
    for (int i = 0; i < noIntervals; i++) {
      _intervals[i] = intervals[i];
    }
    _noIntervals = noIntervals;
  }
  return *this;
}

/*
 * 
 */
Timer& Timer::start(void *context) {
  _interval = 0;
  _cycles = 0;
  unsigned long now = millis();
  _next = now + _intervals[0];
  _context = context;
  _running = true;
  return *this;
}

/*
 * 
 */
Timer& Timer::start(void *context, unsigned long timeout) {
  _interval = 0;
  _cycles = 0;
  _next = timeout;
  _context = context;
  _running = true;
  return *this;
}

Timer& Timer::stop() {
  _running = false;
  return *this;
}

Timer& Timer::restart() {
  if (_running) {
    unsigned long now = millis();
    _next = now + _intervals[_interval];
  }
  return *this;
}

Timer& Timer::polling() {
  if (_running) {
    unsigned long clockTime = millis();
    if (clockTime >= _next) {
      int interval = _interval;
      long cycles = _cycles;
      _interval++;
      if (_interval >= _noIntervals) {
        _interval = 0;
        _cycles++;
      }
      if (!_continuous) {
        stop();
      } else {
        _next += _intervals[_interval];
      }
      if (_onNext != NULL) {
        _onNext(_context, interval, cycles);
      }
    }
  }
  return *this;
}

Timer& Timer::onNext(void (*callback)(void*, int, long)) {
  _onNext = callback;
  return *this;
}
