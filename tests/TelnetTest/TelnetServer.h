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

#ifndef TelnetServerClass_h
#define TelnetServerClass_h

#include <Arduino.h>
#include <WiFi.h>

/*
   Serial WiFi
*/
class TelnetServerClass {
public:

  /*
       Create the telnet server
    */
  TelnetServerClass(const uint8_t port = 22)
    : _server(port) {
  }

  // Initializes telnet server
  void begin(void);

  // Sets the callback
  void onLineReady(void (*callback)(void* context, const char* line), void* context = NULL) {
    _lineContext = context;
    _onLineReady = callback;
  }

  // Sets the callback
  void onClient(void (*callback)(void*, TelnetServerClass&), void* context = NULL) {
    _clientContext = context;
    _onClient = callback;
  }

  // Sets the callback
  void onActivity(void (*callback)(void*, TelnetServerClass&), void* context = NULL) {
    _activityContext = context;
    _onActivity = callback;
  }

  // Polls
  void polling(const unsigned long clockTime = millis());

  // Send data
  void print(const char* data);
  void println(const char* data);


  // Returns true if has client
  const boolean hasClient(void) const {
    return _hasClient;
  }
private:
  WiFiServer _server;
  WiFiClient _client;
  boolean _hasClient;
  size_t _noReadChars;
  void (*_onLineReady)(void*, const char*);
  void* _lineContext;
  void (*_onClient)(void*, TelnetServerClass&);
  void* _clientContext;
  void (*_onActivity)(void*, TelnetServerClass&);
  void* _activityContext;
  char _readBuffer[256];

  void handleClient(void);
  void setActivity(void);
};

#endif
