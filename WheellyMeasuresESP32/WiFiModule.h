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

#ifndef WifiModuleClass_h
#define WifiModuleClass_h

#include <WiFi.h>

#include "Arduino.h"
#include "ConfStore.h"

/*
  Handles the Wifi module by setting station or acces point module
*/
class WiFiModuleClass {
private:
  wl_status_t _status;
  unsigned long _connectingTime;
  ConfigRecord _config;

  void (*_onChange)(void*, WiFiModuleClass&);
  void* _context;

  void startAccessPoint(void);
  void startStation(void);
public:
  WiFiModuleClass(void);

  // Initializes serial wifi
  void begin(const ConfigRecord& config);

  /*
  * Starts the wifi module
  */
  void start(void);

  // Sets the callback
  void onChange(void (*callback)(void* context, WiFiModuleClass& module), void* context = NULL) {
    _context = context;
    _onChange = callback;
  }

  // Polls the serial wifi
  void polling(const unsigned long clockTime = millis());

  // Returns true if acts as access point
  const boolean accessPoint(void) const;

  // Returns true if acts as station
  const boolean station(void) const;

  // Returns the wifi status
  const wl_status_t status(void) const {
    return _status;
  }

  // Returns true if wifi is connected
  const boolean connected(void) const {
    return accessPoint() || status() == WL_CONNECTED;
  }

  // Returns true if wifi is connecting
  const boolean connecting(void) const {
    return station() && status() != WL_CONNECTED;
  }

  // Returns the ip address
  const IPAddress ipAddress(void);

  // Returns the ssid
  const String& ssid(void) const;
};

//extern WiFiModuleClass WiFiModule;

#endif
