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

#include "Contacts.h"



/*
   Creates the contact sensors
*/
ContactSensors::ContactSensors(const uint8_t frontSensorPin, const uint8_t rearSensorPin)
  : _frontSensorPin(frontSensorPin), _rearSensorPin(rearSensorPin) {
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
