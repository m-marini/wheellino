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

#define DEBUG
#include "debug.h"

#include "WiFiModule.h"

static const unsigned long CONNECT_TIMEOUT = 10 * 1000ul;
static const char* DEFAULT_SSID = "Wheelly";

/*
   Creates the WiFiModuleClass
*/
WiFiModuleClass::WiFiModuleClass(void)
  : _status(WL_DISCONNECTED) {
  _config.active = false;
  _config.ssid[0] = 0;
  _config.password[0] = 0;
}

/*
   Initializes the wifi module.
   Loads and applies the configuration
*/
void WiFiModuleClass::begin(void) {
  DEBUG_PRINTLN("// Mounting FS...");

  if (SPIFFS.begin()) {
    DEBUG_PRINTLN("// Mounted FS");
    loadConfig();
  }
  applyConfig();
}

/*
   Saves configuration
   Returns true if successfuly saved
*/
const boolean WiFiModuleClass::saveConfig(void) {
  StaticJsonDocument<200> doc;
  doc["version"] = CURRENT_VERSION;
  doc["active"] = _config.active;
  doc["ssid"] = _config.ssid;
  doc["password"] = _config.password;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    DEBUG_PRINTLN("!! Failed to open /config.json file for writing");
    return false;
  }

  serializeJson(doc, configFile);
  configFile.close();
  return true;
}

/*
   Loads configuration
   Returns true if successfuly saved
*/
const boolean WiFiModuleClass::loadConfig(void) {
  if (!SPIFFS.exists("/config.json")) {
    Serial.println("!! File /config.json does not exit");
    return false;
  }
  File configFile = SPIFFS.open("/config.json", "r");
  size_t size = configFile.size();
  DEBUG_PRINT("// Config size: ");
  DEBUG_PRINT(size);
  DEBUG_PRINTLN();
  if (size > 1024) {
    Serial.println("!! Config file size is too large");
    configFile.close();
    return false;
  }

  StaticJsonDocument<200> doc;
  auto error = deserializeJson(doc, configFile);
  configFile.close();
  if (error) {
    Serial.println("!! Failed to parse config file");
    return false;
  }

  if (!doc.containsKey("version")
      || !doc.containsKey("active")
      || !doc.containsKey("ssid")
      || !doc.containsKey("password")) {
    Serial.println("!! Wrong config file");
    return false;
  }

  WiFiConfig wifiData;

  wifiData.version = doc["version"];
  wifiData.active = doc["active"];
  strncpy(wifiData.ssid, doc["ssid"].as<const char*>(), sizeof(wifiData.ssid));
  strncpy(wifiData.password, doc["password"].as<const char*>(), sizeof(wifiData.password));

  DEBUG_PRINT("// version: ");
  DEBUG_PRINTLN(wifiData.version);
  DEBUG_PRINT("// active:  ");
  DEBUG_PRINTLN(wifiData.active);
  DEBUG_PRINT("// ssid:    ");
  DEBUG_PRINTLN(wifiData.ssid);

  if (wifiData.version != CURRENT_VERSION) {
    DEBUG_PRINTLN("!! Wrong config file version");
    return false;
  }
  _config = wifiData;
  return true;
}

/*
   Starts in access point mode
*/
void WiFiModuleClass::startAccessPoint(void) {
  DEBUG_PRINT("// Starting Access point SSID: ");
  DEBUG_PRINT(DEFAULT_SSID);
  DEBUG_PRINTLN();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(DEFAULT_SSID);

  Serial.print("// WiFi connected ");
  Serial.print(ssid());
  Serial.print("@");
  Serial.println(ipAddress());

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
const char* WiFiModuleClass::ssid(void) const {
  return accessPoint()
           ? DEFAULT_SSID
           : _config.ssid;
}

/*
   Polls the serial wifi
   @param clockTime the current clock time
*/
void WiFiModuleClass::polling(const unsigned long clockTime) {
  //DEBUG_PRINTLN("Wifi polling"));
  if (station()) {
    wl_status_t status = WiFi.status();
    if (_status != status) {
      _status = status;
      switch (status) {
        case WL_CONNECTED:
          Serial.print("// WiFi connected ");
          Serial.print(ssid());
          Serial.print("@");
          Serial.println(ipAddress());
          break;
        case WL_DISCONNECTED:
        case WL_IDLE_STATUS:
          break;
        default:
          Serial.print("!! Failed with status: ");
          Serial.println(status);
      }
      if (_onChange) {
        _onChange(_context, *this);
      }
    }
    if (_status != WL_CONNECTED && clockTime >= _connectingTime + CONNECT_TIMEOUT) {
      Serial.println("!! Wifi connection timeout");
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
   Sets the configuration
   @param config the configuration
*/
void WiFiModuleClass::setConfig(const WiFiConfig& config) {
  _config = config;
  saveConfig();
}

/*
   Applies the configuration
*/
void WiFiModuleClass::applyConfig() {
  if (_config.active) {
    DEBUG_PRINT("// Connecting SSID: ");
    DEBUG_PRINT(_config.ssid);
    DEBUG_PRINTLN();
    _connectingTime = millis();
    WiFi.mode(WIFI_STA);
    WiFi.begin(_config.ssid, _config.password);
    if (_onChange) {
      _onChange(_context, *this);
    }
  } else {
    startAccessPoint();
  }
}
