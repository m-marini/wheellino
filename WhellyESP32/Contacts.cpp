#include "Contacts.h"

/*
   Creates the contact sensors
*/
ContactSensors::ContactSensors(const uint8_t frontSensorPin, const uint8_t rearSensorPin):
  _frontSensorPin(frontSensorPin), _rearSensorPin(rearSensorPin) {
}

/*
   Initializes the contact sensors
*/
void ContactSensors::begin(void) {
  pinMode(_frontSensorPin, INPUT_PULLUP);
  pinMode(_rearSensorPin, INPUT_PULLUP);
}

/*
   Polls for contact sensors
*/
void ContactSensors::polling(const unsigned long t0) {
  boolean frontClear = digitalRead(_frontSensorPin);
  boolean rearClear = digitalRead(_rearSensorPin);
  boolean changed = _frontClear != frontClear || _rearClear != rearClear;
  _frontClear = frontClear;
  _rearClear = rearClear;
  if (_onChanged && changed) {
    _onChanged(_context, *this);
  }
}
