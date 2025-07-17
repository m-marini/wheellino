/*
 * Copyright (c) 2023  Marco Marini, marco.marini@mmarini.org
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 *    END OF TERMS AND CONDITIONS
 *
 */

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
void Timer::start(void) {
  start(millis() + _interval);
}

/*

*/
void Timer::stop(void) {
  _running = false;
}

/*

*/
void Timer::restart(void) {
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
