#include "Tests.h"

//#define DEBUG
#include "debug.h"

#define MAX_POWER 255

/**
   Creates the motor test
*/
MotorTest::MotorTest(MotorCtrl& motor, MotorSensor& sensor, RecordList& records) :
  _motor(motor),
  _sensor(sensor),
  _records(records)
{
}

/**
   Starts the test
*/
void MotorTest::start(const unsigned long t0, const int step, const unsigned long thresholdPulses) {
  DEBUG_PRINT("MotorTest::start");
  DEBUG_PRINTLN();
  _step = step;
  _thresholdPulses = thresholdPulses;
  _startPulses = _sensor.pulses();
  _records.clear();
  _records.add(t0, 0, 0);
  _power = 0;
  _motor.power(0);
  _status = TEST_RAMP_UP;
  if (step == 0) {
    complete(t0);
  }
}

/**
   Stops the test
*/
void MotorTest::stop(void) {
  DEBUG_PRINT("MotorTest::stop");
  DEBUG_PRINTLN();
  _motor.power(0);
  _status = TEST_STOPPED;
}

/**
   Completes the test
*/
void MotorTest::complete(const unsigned long t0) {
  DEBUG_PRINT("MotorTest::complete");
  DEBUG_PRINTLN();
  _records.add(t0, 0, 0);
  _motor.power(0);
  _status = TEST_COMPLETED;
}

/**
   Runs for a step
   Returns true if step ok else false if test has stopped
*/
void MotorTest::runStep(const unsigned long t0) {
  DEBUG_PRINT("MotorTest::runStep");
  DEBUG_PRINTLN();
  switch (_status) {
    case TEST_RAMP_UP:
      _power += _step;
      if (_power > MAX_POWER) {
        stop();
      }
      DEBUG_PRINT("  power=");
      DEBUG_PRINT(_power);
      DEBUG_PRINTLN();
      _motor.power(_power);
      break;
    case TEST_RAMP_DOWN:
      _power -= _step;
      DEBUG_PRINT("  power=");
      DEBUG_PRINT(_power);
      DEBUG_PRINTLN();
      _motor.power(_power);
      if (_power == 0) {
        complete(t0);
      }
      break;
  }
}

/**
   Adds the pulses
*/
void MotorTest::addPulses(const unsigned long t0, const int dPulses) {
  // Add record
  if (isRunning()) {
    if (!_records.add(t0, _power, dPulses)) {
      stop();
    } else if (_status == TEST_RAMP_UP
               && abs(_sensor.pulses() - _startPulses) >= _thresholdPulses) {
      // Ramp up completed
      _status = TEST_RAMP_DOWN;
    }
  }
}

void FrictionTest::stepTimerCallback(void* test, const unsigned long) {
  ((FrictionTest*)test)->handleStepTimer();
}

/**
   Creates the friction test
*/
FrictionTest::FrictionTest(MotorSensor & leftSensor, MotorSensor & rightSensor,
                           MotorCtrl & leftMotor, MotorCtrl & rightMotor,
                           ContactSensors & contacts,
                           RecordList & leftRecords, RecordList & rightRecords):
  _leftMotorTest(leftMotor, leftSensor, leftRecords),
  _rightMotorTest(rightMotor, rightSensor, rightRecords),
  _contacts(contacts)
{
  _stepTimer.continuous(true);
  _stepTimer.onNext(stepTimerCallback, this);
}

/**
    Starts the test
*/
void FrictionTest::start(const unsigned long t0,
                         const unsigned long stepInterval,
                         const unsigned long sustain,
                         const int leftStep, const int rightStep) {
  _startTime = t0;
  _leftMotorTest.start(t0, leftStep, sustain);
  _rightMotorTest.start(t0, rightStep, sustain);
  if (isCompleted()) {
    completeTest(t0);
  } else {
    _stepTimer.interval(stepInterval);
    _stepTimer.start();
  }
}

/**
   Polls the test
*/
void FrictionTest::polling(const unsigned long t0) {
  _stepTimer.polling(t0);
}

void FrictionTest::completeTest(const unsigned long t0) {
  DEBUG_PRINT("FrictionTest::completeTest");
  DEBUG_PRINTLN();
  if (_onCompletion) {
    _onCompletion(_completionContext);
  }
}

void FrictionTest::handleStepTimer(void) {
  if (isRunning()) {
    const unsigned long t0 = millis();
    _leftMotorTest.runStep(t0);
    _rightMotorTest.runStep(t0);
    if (_leftMotorTest.isStopped() || _rightMotorTest.isStopped()) {
      stop("Power limit reached");
      return;
    }
    if (isCompleted()) {
      completeTest(t0);
      return;
    }
  }
}

/**
   Processes contacts
*/
void FrictionTest::processContacts(void) {
  if (isRunning()) {
    // Stop by contact
    stop("!! Stopped for contact");
  }
}

/**
   Processes left pulses
*/
void FrictionTest::processLeftPulses(const unsigned long t0, const int dPulses) {
  if (isRunning()) {
    _leftMotorTest.addPulses(t0, dPulses);
    if (_leftMotorTest.isStopped()) {
      stop("!! Buffer overflow");
    }
  }
}

/**
   Processes right pulses
*/
void FrictionTest::processRightPulses(const unsigned long t0, const int dPulses) {
  if (isRunning()) {
    _rightMotorTest.addPulses(t0, dPulses);
    if (_rightMotorTest.isStopped()) {
      stop("!! Buffer overflow");
    }
  }
}

/**
   Stops the test
*/
void FrictionTest::stop(const char * reason) {
  _stepTimer.stop();
  _leftMotorTest.stop();
  _rightMotorTest.stop();

  if (_onStop) {
    _onStop(_stopContext, reason);
  }
}

/**
   Creates the power test
*/
PowerTest::PowerTest(MotorSensor & leftSensor, MotorSensor & rightSensor,
                     MotorCtrl & leftMotor, MotorCtrl & rightMotor,
                     ContactSensors & contacts,
                     RecordList & leftRecords, RecordList & rightRecords):
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
