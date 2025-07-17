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

#ifndef Timer_h
#define Timer_h

#include "Arduino.h"

/*
   ASynchronous timer
*/
class Timer {
public:
  Timer(){};

  // Sets interval
  void interval(const unsigned long interval) {
    _interval = interval;
  }

  // Sets true if continuos events
  void continuous(const boolean cont) {
    _continuous = cont;
  }

  // Starts the timer
  void start(void);

  // Starts the timer
  void start(const unsigned long timeout);

  // Stops the timer
  void stop(void);

  // Restarts the timer
  void restart(void);

  // Returns true if timer is not expired (is timing)
  const bool isRunning(void) const {
    return _running;
  }

  // Returns the interval
  const unsigned long interval(void) const {
    return _interval;
  }

  // Sets the callback
  void onNext(void (*callback)(void* context, const unsigned long counter), void* context = NULL);

  // Polls the timer
  void polling(const unsigned long clockTime = millis());

  const unsigned long next(void) const {
    return _next;
  }

private:
  bool _continuous;
  unsigned long _interval;
  void (*_onNext)(void*, const unsigned long);
  void* _context;

  unsigned long _next;
  unsigned long _counter;
  boolean _running;
};

#endif
