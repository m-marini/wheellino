#include "Tests.h"

//#define DEBUG
#include "debug.h"

#define MAX_POWER 255

void FrictionTest::stepTimerCallback(void* test, const unsigned long) {
  ((FrictionTest*)test)->handleStepTimer();
}

void FrictionTest::stopTimerCallback(void* test, const unsigned long) {
  ((FrictionTest*)test)->handleStopTimer();
}

/**
   Creates the friction test
*/
FrictionTest::FrictionTest(MotorSensor& leftSensor, MotorSensor& rightSensor,
                           MotorCtrl& leftMotor, MotorCtrl& rightMotor,
                           ContactSensors& contacts,
                           RecordList& leftRecords, RecordList& rightRecords):
  _leftSensor(leftSensor),
  _rightSensor(rightSensor),
  _leftMotor(leftMotor),
  _rightMotor(rightMotor),
  _contacts(contacts),
  _leftRecords(leftRecords),
  _rightRecords(rightRecords)
{
  _stepTimer.continuous(true);
  _stepTimer.onNext(stepTimerCallback, this);
  _stopTimer.onNext(stopTimerCallback, this);
}

/**
    Starts the test
*/
void FrictionTest::start(const unsigned long t0,
                         const unsigned long stepInterval,
                         const unsigned long stopDuration,
                         const int leftStep, const int rightStep) {
  _startTime = t0;
  _leftStep = leftStep;
  _rightStep = rightStep;
  _leftRecords.clear();
  _rightRecords.clear();
  _leftRecords.add(t0, 0, 0);
  _rightRecords.add(t0, 0, 0);
  _leftMotor.power(0);
  _rightMotor.power(0);
  _running = true;
  if (_leftStep == 0 && _rightStep == 0) {
    completeTest(t0);
  } else {
    _stopTimer.interval(stopDuration);
    _stepTimer.interval(stepInterval);
    _stepTimer.start();
  }
}

/**
   Polls the test
*/
void FrictionTest::polling(const unsigned long t0) {
  _stopTimer.polling(t0);
  _stepTimer.polling(t0);
}

void FrictionTest::runStop(const unsigned long t0) {
  DEBUG_PRINT("FrictionTest::runStop");
  DEBUG_PRINTLN();
  if (_running && !_stopTimer.isRunning()) {
    _stepTimer.stop();
    _stopTimer.start();
  }
}

void FrictionTest::completeTest(const unsigned long t0) {
  DEBUG_PRINT("FrictionTest::completeTest");
  DEBUG_PRINTLN();
  _stopTimer.stop();
  _leftPwr = _rightPwr = 0;
  _leftMotor.power(0);
  _rightMotor.power(0);
  _leftRecords.add(t0, 0, 0);
  _rightRecords.add(t0, 0, 0);
  if (!_running) {
    return;
  }
  _running = false;
  if (_onCompletion) {
    _onCompletion(_completionContext);
  }
}

void FrictionTest::handleStopTimer(void) {
  completeTest(millis());
}

void FrictionTest::handleStepTimer(void) {
  _leftPwr += _leftStep;
  _rightPwr += _rightStep;
  if (abs(_leftPwr) > MAX_POWER || abs(_rightPwr) > MAX_POWER) {
    stop("Power limit reached");
    return;
  }
  _leftMotor.power(_leftPwr);
  _rightMotor.power(_rightPwr);
}

/**
   Processes contacts
*/
void FrictionTest::processContacts(const boolean front, const boolean rear) {
  if (_running && (front || rear)) {
    // Stop by contact
    char msg[256];
    sprintf(msg, "!! Stopped for %s, %s contacts",
            front ? "front" : "",
            rear ? "rear" : "");
    stop(msg);
  }
}

/**
   Processes left pulses
*/
void FrictionTest::processLeftPulses(const unsigned long t0, const int dPulses) {
  if (_running && !_leftRecords.add(t0, _leftPwr, dPulses)) {
    stop("!! Buffer overflow");
  }
  _leftStep = 0;
  if (_rightStep == 0) {
    runStop(t0);
  }
}

/**
   Processes right pulses
*/
void FrictionTest::processRightPulses(const unsigned long t0, const int dPulses) {
  if (_running && !_rightRecords.add(t0, _rightPwr, dPulses)) {
    stop("!! Buffer overflow");
  }
  _rightStep = 0;
  if (_leftStep == 0) {
    runStop(t0);
  }
}

/**
   Stops the test
*/
void FrictionTest::stop(const char * reason) {
  _stepTimer.stop();
  _running = false;
  _leftMotor.power(0);
  _rightMotor.power(0);

  if (_onStop) {
    _onStop(_stopContext, reason);
  }
}

/**
   Creates the power test
*/
PowerTest::PowerTest(MotorSensor& leftSensor, MotorSensor& rightSensor,
                     MotorCtrl& leftMotor, MotorCtrl& rightMotor,
                     ContactSensors& contacts,
                     RecordList& leftRecords, RecordList& rightRecords):
  _leftSensor(leftSensor),
  _rightSensor(rightSensor),
  _leftMotor(leftMotor),
  _rightMotor(rightMotor),
  _contacts(contacts),
  _leftRecords(leftRecords),
  _rightRecords(rightRecords)
{}

/**
   Polls the test
*/
void PowerTest::polling(const unsigned long t0) {
  if (!_running || t0 < _startTime) {
    return;
  }
  const unsigned long dt = t0 - _startTime;

  DEBUG_PRINT("// pollTest t0: ");
  DEBUG_PRINT(t0);
  DEBUG_PRINT(", startTime: ");
  DEBUG_PRINT(_startTime);
  DEBUG_PRINT(", dt: ");
  DEBUG_PRINT(dt);
  DEBUG_PRINTLN();

  if (dt > _duration) {
    _leftPwr = _rightPwr = 0;
    _leftMotor.power(0);
    _rightMotor.power(0);
    _leftRecords.add(t0, 0, 0);
    _rightRecords.add(t0, 0, 0);
    if (!_running) {
      return;
    }
    _running = false;
    if (_onCompletion) {
      _onCompletion(_completionContext);
    }
    return;
  }
  int left;
  int right;
  if (dt <= _duration / 2) {
    left = map(dt, 0, _duration / 2, 0, _maxLeftPwr);
    right = map(dt, 0, _duration / 2, 0, _maxRightPwr);
  } else {
    left = map(dt, _duration / 2, _duration, _maxLeftPwr, 0);
    right = map(dt, _duration / 2, _duration, _maxRightPwr, 0);
  }
  if (_leftPwr != left) {
    _leftPwr = left;
    _leftMotor.power(left);
  }
  if (_rightPwr != right) {
    _rightPwr = right;
    _rightMotor.power(right);
  }
}

/**
   Processes left pulses
*/
void PowerTest::processLeftPulses(const unsigned long t0, const int dPulses) {
  if (_running && !_leftRecords.add(t0, _leftPwr, dPulses)) {
    stop("!! Buffer overflow");
  }
}

/**
   Processes right pulses
*/
void PowerTest::processRightPulses(const unsigned long t0, const int dPulses) {
  if (_running && !_rightRecords.add(t0, _rightPwr, dPulses)) {
    stop("!! Buffer overflow");
  }
}

/**
   Stops the test
*/
void PowerTest::stop(const char * reason) {
  _running = false;
  _leftMotor.power(0);
  _rightMotor.power(0);

  if (_onStop) {
    _onStop(_stopContext, reason);
  }
}

void PowerTest::start(const unsigned long t0, const unsigned long duration,
                      const int leftPwr, const int rightPwr) {

  _duration = duration;
  _maxLeftPwr = leftPwr;
  _maxRightPwr = rightPwr;
  _leftRecords.clear();
  _rightRecords.clear();
  _startTime = t0;
  _leftRecords.add(t0, 0, 0);
  _rightRecords.add(t0, 0, 0);

  _leftPwr = _rightPwr = 0;
  _running = true;

  _leftMotor.power(0);
  _rightMotor.power(0);
}

/**
   Processes contacts
*/
void PowerTest::processContacts(const boolean front, const boolean rear) {
  if (_running && (front || rear)) {
    // Stop by contact
    char msg[256];
    sprintf(msg, "!! Stopped for %s, %s contacts",
            front ? "front" : "",
            rear ? "rear" : "");
    stop(msg);
  }
}

/**
   Append a records
*/
const boolean RecordList::add(const unsigned long t0, const int power, const int dPulses) {
  if (isFull()) {
    return false;
  } else {
    _records[_size].pulses = dPulses;
    _records[_size].power = power;
    _records[_size].time = t0;
    _size++;
    return true;
  }
}
