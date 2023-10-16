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
    TelnetServerClass(const uint8_t port = 22) : _server(port) {
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
    void *_lineContext;
    void (*_onClient)(void*, TelnetServerClass&);
    void *_clientContext;
    void (*_onActivity)(void*, TelnetServerClass&);
    void *_activityContext;
    char _readBuffer[256];

    void handleClient(void);
    void setActivity(void);
};

#endif
