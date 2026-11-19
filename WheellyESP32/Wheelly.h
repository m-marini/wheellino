/*
 * Copyright (c) 2023-2006 Marco Marini, marco.marini@mmarini.org
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

#ifndef Wheelly_h
#define Wheelly_h

#include <Arduino.h>

#define WHEELLY_VERSION "1.0.0"
#define WHEELLY_MESSAGES_VERSION "v2"

enum HeadStatus {
  FIX_DIRECTION = 0,
  FRONT_TRACKING = 1,
  REAR_TRACKING = 2,
};

/*
   Wheelly controller.
   Handles all the behavior of wheelly robot:
   - commands from wifi modules
   - scanning of proxy sensors
   - contact sensors
   - mpu sensors
   - motor sensors
   - drives the motors
   - operational timing processes
*/
class Wheelly {
public:
  /**
 * @brief Constructs a new `Wheelly` robot controller instance and initialises device identifiers.
 * 
 * Sets the default operational parameters for telemetry transmission intervals, 
 * head servo angular limits, and the anti-gimbal radius. Reads the hardware eFuse 
 * MAC address from the ESP32, reverses its 6-byte order into standard big-endian 
 * representation, formats it as a hexadecimal device identifier (`_id`), and 
 * builds the MQTT sensor publication prefix and command subscription wildcard topics.
 */
  Wheelly();

  /**
 * @brief Retrieves the unique identifier of the robot controller.
 * 
 * Returns a null-terminated string representing the identifier assigned to this 
 * `Wheelly` instance, typically used for messaging topics, logging, and telemetry.
 * 
 * @return Pointer to a null-terminated constant character string containing the robot ID.
 */
  const char* id(void) const;

  /**
 * @brief Retrieves the current operational status of the robot's head.
 * 
 * Returns the active `HeadStatus` enumeration value indicating the current mode 
 * or state of the head subsystem (such as idle, front tracking, or rear tracking).
 * 
 * @return Current `HeadStatus` state of the head.
 */
  const HeadStatus headStatus(void) const;

  /*
  * Returns the x head target (pulses)
  */
  const float xHeadTarget(void) const;

  /*
  * Returns the y head target (pulses)
  */
  const float yHeadTarget(void) const;

  /**
  * Returns the head anti gimbal radius (pulses)
  */
  const int antiGimbalRadius(void) const;

  /**
    * Returns the command subscription topics
    */
  const char* subCommandTopics(void) const;
  /**
       Initializes wheelly controller
       Return true if successfully initialized
    */
  boolean begin(void);

  /**
       Pools wheelly controller
    */
  void polling(const unsigned long t0 = millis());

  /**
 * @brief Updates the online connectivity state of the robot controller.
 * 
 * Sets the internal `_onLine` flag to reflect whether the messaging connection 
 * is active. When transitioning to an online state (`true`), it immediately 
 * transmits a `"hi"` presence message to announce availability.
 * 
 * @param onLine Connection status flag: `true` if the controller is connected and 
 *               online; `false` otherwise.
 */
  void onLine(const boolean onLine);

  /**
       Set connected state
       @param state true if connected
    */
  void connected(const boolean state);

  /**
       Sets activity
    */
  void activity(const unsigned long t0 = millis());

  /**
       Displays the message
       @param message the message
    */
  void display(const char* message);
};

extern Wheelly wheelly;

#endif
