#include "TelnetServer.h"

//#define DEBUG
#include "debug.h"

/*
   Initializes the telnet server
*/
void TelnetServerClass::begin(void) {
  _server.begin();
  DEBUG_PRINTLN("Telnet Server begin");
}

/*
   Handles the client
*/
void TelnetServerClass::handleClient(void) {
  boolean activity = false;
  // Read the data from wifi client
  while (_client.available() > 0) {
    activity = true;
    const char c = _client.read();
    if (c == '\n') {
      // End line received
      if (_onLineReady) {
        // Flush data
        _readBuffer[_noReadChars] = 0;
        _onLineReady(_lineContext, _readBuffer);
      }
      _noReadChars = 0;
    } else if (_noReadChars < sizeof(_readBuffer) - 1) {
      // Buffer size available
      _readBuffer[_noReadChars] = c;
      _noReadChars++;
    }
    // else ignore data due to buffer full 
  }
  if (activity) {
    setActivity();
  }
}

/*
   Polls the server for client handling
   @param clockTime the current clock time
*/
void TelnetServerClass::polling(unsigned long clockTime) {
  if (!_hasClient) {
    _client = _server.available();
    if (_client) {
      // Client has just connected
      DEBUG_PRINTLN("// Client available");
      _hasClient = true;
      if (_onClient) {
        _onClient(_clientContext, *this);
      }
    }
  }
  if (_hasClient) {
    if (_client && _client.connected()) {
      // Client is connected
      handleClient();
    } else {
      // Client has disconnected
      DEBUG_PRINTLN("// Client disconnected.");
      _client.stop();
      _hasClient = false;
      if (_onClient) {
        _onClient(_clientContext, *this);
      }
    }
  }
}

/*
   Write data to wifi client
   @param data the data
*/
void TelnetServerClass::print(const char* data) {
  if (_client && _hasClient) {
    _client.print(data);
    setActivity();
  }
}

/*
   Write data to wifi client
   @param data the data
*/
void TelnetServerClass::println(const char* data) {
  if (_client && _hasClient) {
    _client.println(data);
    _client.flush();
    setActivity();
  }
}

/*
   Sets the activity
*/
void TelnetServerClass::setActivity(void) {
  DEBUG_PRINTLN("Telnet Server activity");
  if (_onActivity) {
    _onActivity(_activityContext, *this);
  }
}
