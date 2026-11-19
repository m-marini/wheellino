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

#ifndef ConfStore_h
#define ConfStore_h

#include "Arduino.h"
#include <ArduinoJson.h>

#define CURRENT_VERSION "2"

/*
   Configuration
*/
struct ConfigRecord {
  char version[8];
  char wifiSsid[33];
  char wifiPassword[65];
  char mqttBrokerHost[65];
  char mqttUser[65];
  char mqttPsw[129];
  int mqttBrokerPort;
  bool wifiActive;
};

enum ConfStoreRetCode {
  SUCCESS = 0,
  CANNOT_WRITE_FILE = -1,
  FILE_NOT_FOUND = -2,
  CONFIG_FILE_TOO_LARGE = -3,
  PARSE_JSON_ERROR = -4,
  BAD_CONFIG_VERSION = -5,
};

/*
  Stores the configuration
*/
class ConfStore {
public:
  /**
  * Creates the configuration store
  */
  ConfStore(void);

  /**
  * Initializes the config store
  */
  void begin(void);

  /*
  * Stores the configuration
  * Returns 0 if success
  */
  const ConfStoreRetCode store(const ConfigRecord& config);

  /**
  * Returns the configuration
  */
  const ConfigRecord& config() const;
  
  /**
  * Converts the configuration record to json doc
  */
  static JsonDocument& toJson(JsonDocument& result, const ConfigRecord& config);

  /**
  * Converts the json doc to configuration record
  * Returns SUCCESS if success
  */
  static const ConfStoreRetCode fromJson(ConfigRecord& result, const JsonDocument& json);

  /**
  * Returns the default config record
  */
  static ConfigRecord& defaultConfig(ConfigRecord& result);
};

/**
* Configuration store
*/
extern ConfStore confStore;


#endif
