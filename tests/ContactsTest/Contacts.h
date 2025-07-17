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

private:
  const uint8_t _frontSensorPin;
  const uint8_t _rearSensorPin;
  boolean _frontClear;
  boolean _rearClear;
};

#endif
