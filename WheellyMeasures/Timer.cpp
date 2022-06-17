#include "Timer.h"

/*

*/
Timer::Timer() {
}

/*

*/
Timer& Timer::continuous(bool cont) {
  _continuous = cont;
  return *this;
}

/*

*/
Timer& Timer::interval(unsigned long interval) {
  _interval = interval;
  return *this;
}

/*

*/
Timer& Timer::start(void *context) {
  return start(context, millis() + _interval);
}

/*

*/
Timer& Timer::start(void *context, unsigned long timeout) {
  _counter = 0;
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
    _next = millis() + _interval;
  }
  return *this;
}

Timer& Timer::polling(unsigned long clockTime) {
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
  return *this;
}

Timer& Timer::onNext(void (*callback)(void*, unsigned long)) {
  _onNext = callback;
  return *this;
}
