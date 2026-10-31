/*
 * Copyright (c) 2026  Marco Marini, marco.marini@mmarini.org
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

#define CURRENT_VERSION "2"  ///< Current expected version of the configuration layout.

/**
 * @struct SDConfigRecord
 * @brief Holds the system configuration parameters.
 * 
 * This structure contains the network credentials, MQTT broker details, 
 * and configuration versioning used across the system.
 */
struct SDConfigRecord {
  char version[10];         ///< The configuration structure version string.
  bool wifiActive;          ///< Flag indicating if Wi-Fi should be initialised.
  char wifiSsid[32];        ///< Service Set Identifier (SSID) for the Wi-Fi network.
  char wifiPassword[64];    ///< Pre-shared key for Wi-Fi authentication.
  char mqttBrokerHost[32];  ///< Hostname or IP address of the MQTT broker.
  int mqttBrokerPort;       ///< Network port number for the MQTT connection.
  char mqttUser[32];        ///< Username for MQTT authentication.
  char mqttPsw[64];         ///< Password for MQTT authentication.
};

/**
 * @enum SDConfStoreRetCode
 * @brief Status and error return codes for configuration store operations.
 */
enum SDConfStoreRetCode {
  SUCCESS = 0,                 ///< Operation completed successfully.
  CANNOT_WRITE_FILE = -1,      ///< Unable to write configuration to the storage medium.
  FILE_NOT_FOUND = -2,         ///< Configuration file could not be located.
  CONFIG_FILE_TOO_LARGE = -3,  ///< File size exceeds the allocated parsing buffer.
  PARSE_JSON_ERROR = -4,       ///< JSON schema or syntax validation failed.
  BAD_CONFIG_VERSION = -5,     ///< The file version does not match CURRENT_VERSION.
};

/**
 * @class SDConfStore
 * @brief Manages loading, storing, and parsing configuration records from an SD card.
 * 
 * This class abstracts the handling of local persistence for configuration structures
 * by using the ArduinoJson library to cross-serialise data structures.
 */
class SDConfStore {
private:
  SDConfigRecord _config;  ///< Internal active configuration structure instance.

  /**
     * @brief Loads the configuration from the storage file into the internal record.
     * @return const SDConfStoreRetCode SUCCESS if successful, or an explicit error code on failure.
     */
  const SDConfStoreRetCode loadConfig(void);

public:
  /**
     * @brief Constructs the configuration store instance.
     */
  SDConfStore(void);

  /**
     * @brief Initialises the storage hardware and internal state.
     * 
     * Prepares peripheral buses (such as SPI/SD) to ensure configuration persistence is ready.
     */
  void begin(void);

  /**
     * @brief Retrieves a read-only reference to the active configuration record.
     * @return const SDConfigRecord& Reference to the internally cached configuration data.
     */
  const SDConfigRecord& config() const {
    return _config;
  }

  /**
     * @brief Converts a JSON document into a standard configuration record.
     * 
     * Extracts fields securely from an abstract JSON schema map into a predefined struct block.
     * 
     * @param[out] result Destination record structure to write parsed values into.
     * @param[in] json Source JSON document read from disk.
     * @return const SDConfStoreRetCode SUCCESS if parsing succeeded without critical validation mismatches.
     */
  static const SDConfStoreRetCode fromJson(SDConfigRecord& result, const JsonDocument& json);

  /**
     * @brief Populates a configuration record structure with safe fallback default values.
     * 
     * @param[out] result Destination record structure to fill.
     * @return SDConfigRecord& Mutable reference to the populated record instance.
     */
  static SDConfigRecord& defaultConfig(SDConfigRecord& result);
};

#endif
