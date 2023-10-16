#ifndef WifiModuleClass_h
#define WifiModuleClass_h

#include <WiFi.h>

#include "Arduino.h"

#define CURRENT_VERSION 0

/*
   Wifi data configuration
*/
struct WiFiConfig {
  int version;
  bool active;
  char ssid[128];
  char password[64];
};

/*
  Handles the Wifi module by setting station or acces point module
*/
class WiFiModuleClass {
  public:
    WiFiModuleClass(void);

    /**
     * Initializes serial wifi
     */
    void begin(void);

    /**
     * Sets the configuration
     */
    void setConfig(const WiFiConfig& config);

    /**
     * Sets the callback
     */
    void onChange(void (*callback)(void* context, WiFiModuleClass& module), void* context = NULL) {
      _context = context;
      _onChange = callback;
    }

    /**
     * Polls the serial wifi
     */
    void polling(const unsigned long clockTime = millis());

    /**
     * Returns true if acts as access point
     */
    const boolean accessPoint(void) const;

    /**
     * Returns true if acts as station
     */
    const boolean station(void) const;

    /**
     * Returns the wifi status
     */
    const wl_status_t status(void) const {
      return _status;
    }

    /**
     * Returns true if wifi is connected
     */
    const boolean connected(void) const {
      return accessPoint() || status() == WL_CONNECTED;
    }

    /**
     * Returns true if wifi is connecting
     */
    const boolean connecting(void) const {
      return station() && status() != WL_CONNECTED;
    }

    /**
     * Returns the configuration
     */
    const WiFiConfig& config(void) const {
      return _config;
    }

    /**
     * Returns the ip address
     */
    const IPAddress ipAddress(void);

    /**
     * Returns the ssid
     */
    const char* ssid(void) const;

    /**
     * Applies the configuration
     */
    void applyConfig(void);

  private:
    wl_status_t _status;
    unsigned long _connectingTime;
    WiFiConfig _config;

    void (*_onChange)(void*, WiFiModuleClass&);
    void *_context;

    const boolean loadConfig(void);
    const boolean saveConfig(void);
    void startAccessPoint(void);
    void startStation(void);
};

#endif
