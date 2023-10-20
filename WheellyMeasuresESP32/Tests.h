#ifndef Tests_h
#define Tests_h

#include "Arduino.h"
#include "MotorCtrl.h"
#include "Contacts.h"
#include "Timer.h"

#define BUFFER_SIZE 2048

struct Record {
  unsigned long time;
  int power;
  int pulses;
};

class RecordList {
  public:
    RecordList(void) {}

    /**
       Adds a record.
       Returns true if successfull add or false if overflow error
    */
    const boolean add(const unsigned long time, const int power, const int dPulses);

    /**
       Clears the list
    */
    void clear(void) {
      _size = 0;
    }

    /**
       Returns the records
    */
    const Record* records(void) const {
      return _records;
    }

    /**
       Returns the size
    */
    const size_t size(void) const {
      return _size;
    }

    /**
       Returns true if the list is empty
    */
    const boolean isEmpty(void) const {
      return _size == 0;
    }

    /**
       Returns true if the list is full
    */
    const boolean isFull(void) const {
      return _size >= BUFFER_SIZE;
    }
  private:
    Record _records[BUFFER_SIZE];
    size_t _size;
};

/**
   Handles the friction test
*/
class FrictionTest {
  public:
    FrictionTest(MotorSensor &leftSensor, MotorSensor &rightSensor,
                 MotorCtrl &leftMotor, MotorCtrl &_rightMotor,
                 ContactSensors &contacts,
                 RecordList& leftRecords, RecordList& rightRecords);

    /**
       Starts the test
    */
    void start(const unsigned long t0, const unsigned long stepInterval, const unsigned long stopDuration, const int leftStep, const int rightStep);

    /**
       Stops the test
    */
    void stop(const char * reason);

    /**
       Polls the test
    */
    void polling(const unsigned long t0);

    /**
       Sets the completion call back
    */
    void onCompletion(void (*callback)(void *context), void* context = NULL) {
      _onCompletion = callback;
      _completionContext = context;
    }

    /**
       Sets the stop call back
    */
    void onStop(void (*callback)(void *context, const char *reason), void* context = NULL) {
      _onStop = callback;
      _stopContext = context;
    }

    /**
       Processes left pulses
    */
    void processLeftPulses(const unsigned long t0, const int dPulses);

    /**
       Processes right pulses
    */
    void processRightPulses(const unsigned long t0, const int dPulses);

    /**
       Processes contacts
    */
    void processContacts(const boolean front, const boolean rear);

    /**
       Returns true if test is running
    */
    const boolean isRunning(void) const {
      return _running;
    }

  private:
    MotorSensor &_leftSensor;
    MotorSensor &_rightSensor;
    MotorCtrl &_leftMotor;
    MotorCtrl &_rightMotor;
    ContactSensors &_contacts;
    RecordList& _leftRecords;
    RecordList& _rightRecords;
    Timer _stepTimer;
    Timer _stopTimer;
    boolean _running;
    int _leftPwr;
    int _rightPwr;
    int _leftStep;
    int _rightStep;
    unsigned long _startTime;
    unsigned long _stepDuration;
    void* _completionContext;
    void* _stopContext;
    void (*_onCompletion)(void *);
    void (*_onStop)(void *, const char*);

    void handleStopTimer(void);
    void handleStepTimer(void);
    void completeTest(const unsigned long t0);
    void runStop(const unsigned long t0);
    static void stepTimerCallback(void*, const unsigned long);
    static void stopTimerCallback(void*, const unsigned long);
};


/**
   Handles the power test
*/
class PowerTest {
  public:
    /**
       Creates the power test
    */
    PowerTest(MotorSensor &leftSensor, MotorSensor &rightSensor,
              MotorCtrl &leftMotor, MotorCtrl &rightMotor,
              ContactSensors &contacts,
              RecordList& leftRecords, RecordList& rightRecords);


    /**
       Starts the test
    */
    void start(const unsigned long t0, const unsigned long duration,
               const int leftPwr, const int rightPwr);

    /**
       Stops the test
    */
    void stop(const char * reason);

    /**
       Polls the test
    */
    void polling(const unsigned long t0);

    /**
       Sets the completion call back
    */
    void onCompletion(void (*callback)(void *context), void* context = NULL) {
      _onCompletion = callback;
      _completionContext = context;
    }

    /**
       Sets the stop call back
    */
    void onStop(void (*callback)(void *context, const char *reason), void* context = NULL) {
      _onStop = callback;
      _stopContext = context;
    }

    /**
       Processes left pulses
    */
    void processLeftPulses(const unsigned long t0, const int dPulses);

    /**
       Processes right pulses
    */
    void processRightPulses(const unsigned long t0, const int dPulses);

    /**
       Processes contacts
    */
    void processContacts(const boolean front, const boolean rear);

    /**
       Returns true if test is running
    */
    const boolean isRunning(void) const {
      return _running;
    }

  private:
    MotorSensor &_leftSensor;
    MotorSensor &_rightSensor;
    MotorCtrl &_leftMotor;
    MotorCtrl &_rightMotor;
    ContactSensors &_contacts;
    RecordList& _leftRecords;
    RecordList& _rightRecords;
    boolean _running;
    int _leftPwr;
    int _rightPwr;
    int _maxLeftPwr;
    int _maxRightPwr;
    unsigned long _startTime;
    unsigned long _duration;
    void* _completionContext;
    void* _stopContext;
    void (*_onCompletion)(void *);
    void (*_onStop)(void *, const char*);
};

#endif
