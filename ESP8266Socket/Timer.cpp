#include "Timer.h"

/*

*/
void Timer::start(unsigned long timeout) {
  _counter = 0;
  _next = timeout;
  _running = true;
}

/*

*/
void Timer::start() {
  start(millis() + _interval);
}

/*

*/
void Timer::stop() {
  _running = false;
}

/*

*/
void Timer::restart() {
  if (_running) {
    _next = millis() + _interval;
  }
}

/*

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

/*

*/
void Timer::onNext(void (*callback)(void*, unsigned long), void* context) {
  _onNext = callback;
  _context = context;
}
