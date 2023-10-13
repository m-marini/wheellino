#ifndef Contacts_h
#define Contacts_h

#include "Arduino.h"

/*
   Reads the status of contact sensors
*/
class ContactSensors {
  public:
    /*
       Creates the contact sensors
    */
    ContactSensors(const uint8_t frontSensorPin, const uint8_t rearSensorPin);

    /*
       Initializes the contact sensors
    */
    void begin(void);

    /*
       Polls for contact sensors
    */
    void polling(const unsigned long t0 = millis());

    /*
       Returns true if front contact clear
    */
    const boolean frontClear(void) const {
      return _frontClear;
    }

    /*
       Returns true if rear contact clear
    */
    const boolean rearClear(void) const {
      return _rearClear;
    }

    /**
       Sets callback on reached
    */
    void onChanged(void (*callback)(void *context, ContactSensors& sensors), void* context = NULL) {
      _onChanged = callback;
      _context = context;
    }

  private:
    const uint8_t _frontSensorPin;
    const uint8_t _rearSensorPin;
    boolean _frontClear;
    boolean _rearClear;
    void *_context;
    void (*_onChanged)(void *, ContactSensors&);
};

#endif
