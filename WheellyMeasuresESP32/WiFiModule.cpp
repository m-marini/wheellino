/*
 * Copyright (c) 2023  Marco Marini, marco.marini@mmarini.org
 *
 * Permission is hereby granted, free of charge, to any person
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

#include <ArduinoJson.h>
#include <SPIFFS.h>

#include <esp_log.h>
static const char* TAG = "WiFiModule";

#include "WiFiModule.h"

static const unsigned long CONNECT_TIMEOUT = 10 * 1000ul;
static const String DEFAULT_SSID = "Wheelly";

/*
   Creates the WiFiModuleClass
*/
WiFiModuleClass::WiFiModuleClass(void)
  : _status(WL_DISCONNECTED) {
}

/*
   Initializes the wifi module.
   Loads and applies the configuration
*/
void WiFiModuleClass::begin(const ConfigRecord& config) {
  ESP_LOGI(TAG, "Begin");
  _config = config;
}

/*
   Starts in access point mode
*/
void WiFiModuleClass::startAccessPoint(void) {
  ESP_LOGD(TAG, "Starting Access point SSID: %d", DEFAULT_SSID.c_str());
  WiFi.mode(WIFI_AP);
  WiFi.softAP(DEFAULT_SSID);

  ESP_LOGI(TAG, "WiFi connected %s@%s", ssid().c_str(), ipAddress().toString().c_str());

  if (_onChange) {
    _onChange(_context, *this);
  }
}

/**
   Returns the ip address
*/
const IPAddress WiFiModuleClass::ipAddress(void) {
  return accessPoint()
           ? WiFi.softAPIP()
           : WiFi.localIP();
}

/**
   Returns the ip address
*/
const String& WiFiModuleClass::ssid(void) const {
  return accessPoint()
           ? DEFAULT_SSID
           : _config.wifiSsid;
}

/*
   Polls the serial wifi
   @param clockTime the current clock time
*/
void WiFiModuleClass::polling(const unsigned long clockTime) {
  if (station()) {
    wl_status_t status = WiFi.status();
    if (_status != status) {
      _status = status;
      switch (status) {
        case WL_CONNECTED:
          ESP_LOGI(TAG, "WiFi connected %s@%s", ssid().c_str(), ipAddress().toString().c_str());
          break;
        case WL_DISCONNECTED:
        case WL_IDLE_STATUS:
          break;
        default:
          ESP_LOGE(TAG, "Failed with status: %d", status);
      }
      if (_onChange) {
        _onChange(_context, *this);
      }
    }
    if (_status != WL_CONNECTED && clockTime >= _connectingTime + CONNECT_TIMEOUT) {
      ESP_LOGE(TAG, "Wifi connection timeout");
      startAccessPoint();
    }
  }
}

/*
   Returns true if acts as a station
*/
const boolean WiFiModuleClass::station(void) const {
  return WiFi.getMode() & WIFI_STA;
}

/*
   Returns true if acts as an access point
*/
const boolean WiFiModuleClass::accessPoint(void) const {
  return WiFi.getMode() & WIFI_AP;
}

/*
   Applies the configuration
*/
void WiFiModuleClass::start() {
  if (_config.wifiActive) {
    ESP_LOGD(TAG, "Connecting SSID: %s", _config.wifiSsid.c_str());
    _connectingTime = millis();
    WiFi.mode(WIFI_STA);
    WiFi.begin(_config.wifiSsid, _config.wifiPassword);
    if (_onChange) {
      _onChange(_context, *this);
    }
  } else {
    startAccessPoint();
  }
}
