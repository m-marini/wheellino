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
struct SDConfigRecord {
  String version;
  bool wifiActive;
  String wifiSsid;
  String wifiPassword;
  String mqttBrokerHost;
  int mqttBrokerPort;
  String mqttUser;
  String mqttPsw;
};

enum SDConfStoreRetCode {
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
class SDConfStore {
private:
  SDConfigRecord _config;
  const SDConfStoreRetCode loadConfig(void);

public:
  /**
  * Creates the configuration store
  */
  SDConfStore(void);

  /**
  * Initializes the config store
  */
  void begin(void);

  /**
  * Returns the configuration
  */
  const SDConfigRecord& config() const {
    return _config;
  }

  /**
  * Converts the json doc to configuration record
  * Returns SUCCESS if success
  */
  static const SDConfStoreRetCode fromJson(SDConfigRecord& result, const JsonDocument& json);

  /**
  * Returns the default config record
  */
  static SDConfigRecord& defaultConfig(SDConfigRecord& result);
};

#endif
