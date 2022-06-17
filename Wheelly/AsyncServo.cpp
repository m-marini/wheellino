#include "AsyncServo.h"

//#define DEBUG
#include "debug.h"
#define MILLIS_PER_DEG  (180ul / 60)
#define MIN_INTERVAL    1ul

AsyncServo::AsyncServo() {
  _timer.onNext(_handleTimeout, this);
}

void AsyncServo::attach(byte pin) {
  _servo.attach(pin);
}

void AsyncServo::onReached(void (*callback)(void*, byte), void* context) {
  _onReached = callback;
  _context = context;
}

void AsyncServo::offset(const int value) {
  _offset = value;
}

void AsyncServo::angle(byte value) {
  _timer.stop();
  int da = abs((int) value - _angle);
  _angle = value;
  if (da != 0) {
    int wr = (int) value + _offset;
    DEBUG_PRINT(F("// AsyncServo::angle "));
    DEBUG_PRINT(wr);
    DEBUG_PRINTLN();
    _servo.write(wr);
  }
  _timer.interval(max(da * MILLIS_PER_DEG, MIN_INTERVAL));
  _timer.start();
}

static void AsyncServo::_handleTimeout(void * context, unsigned long) {
  AsyncServo* servo = (AsyncServo*) context;
  if (servo->_onReached != NULL) {
    servo->_onReached(servo->_context, servo->_angle);
  }
}

void AsyncServo::polling(unsigned long clockTime) {
  _timer.polling(clockTime);
}
