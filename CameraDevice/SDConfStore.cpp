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

#include "SDConfStore.h"
#include <ArduinoJson.h>

// MicroSD Libraries
#include "FS.h"
#include "SD_MMC.h"

#include <esp_log.h>
static const char* TAG = "SDConfStore";

static const char* CONFIG_FILE = "/config.json";

/**
 * Creates the configuration store
 */
SDConfStore::SDConfStore(void) {
  defaultConfig(_config);
}

/**
  * Returns the default config record
  */
SDConfigRecord& SDConfStore::defaultConfig(SDConfigRecord& config) {
  config = {
    .wifiActive = false,
    .mqttBrokerPort = 1883,
  };
  strcpy(config.version, CURRENT_VERSION);
  strcpy(config.wifiSsid, "Wheelly");
  strcpy(config.mqttBrokerHost, "brokerHost");
  strcpy(config.mqttUser, "wheelly");
  config.wifiPassword[0] = '\0';
  config.mqttPsw[0] = '\0';
  return config;
}

/**
 * Converts the json doc to configuration record
 * Returns SUCCESS if success
 */
const SDConfStoreRetCode SDConfStore::fromJson(SDConfigRecord& config, const JsonDocument& doc) {

  ESP_LOGD(TAG, "Json file %s", doc);

  defaultConfig(config);
  config.wifiActive = doc["wifiActive"] | config.wifiActive;
  config.mqttBrokerPort = doc["mqttBrokerPort"] | config.mqttBrokerPort;

  snprintf(config.version, sizeof(config.version), "%s", doc["version"] | config.version);
  snprintf(config.wifiSsid, sizeof(config.wifiSsid), "%s", doc["wifiSsid"] | config.wifiSsid);
  snprintf(config.wifiPassword, sizeof(config.wifiPassword), "%s", doc["wifiPassword"] | config.wifiPassword);
  snprintf(config.mqttBrokerHost, sizeof(config.mqttBrokerHost), "%s", doc["mqttBrokerHost"] | config.mqttBrokerHost);
  snprintf(config.mqttUser, sizeof(config.mqttUser), "%s", doc["mqttUser"] | config.mqttUser);
  snprintf(config.mqttPsw, sizeof(config.mqttPsw), "%s", doc["mqttPassword"] | config.mqttPsw);

  // Validates json fields
  if (strcmp(config.version, CURRENT_VERSION)) {
    ESP_LOGE(TAG, "Bad config file version [%s]", config.version);
    return BAD_CONFIG_VERSION;
  }
  return SUCCESS;
}

/**
  * Initializes the config store
  */
void SDConfStore::begin(void) {
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    ESP_LOGE(TAG, "No MicroSD Card found");
    return;
  }
  ESP_LOGI(TAG, "Mounted MicroSD Card");
  loadConfig();
}

/*
   Loads configuration
   Returns SUCCESS if successfuly saved
*/
const SDConfStoreRetCode SDConfStore::loadConfig(void) {
  fs::FS& fs = SD_MMC;
  File configFile = fs.open(CONFIG_FILE, FILE_READ);
  if (!configFile) {
    ESP_LOGE(TAG, "Failed to open file in read mode");
    return FILE_NOT_FOUND;
  }
  size_t size = configFile.size();
  ESP_LOGD(TAG, "Config size: %ld", size);
  if (size > 1024) {
    ESP_LOGE(TAG, "Config file size is too large");
    configFile.close();
    return CONFIG_FILE_TOO_LARGE;
  }

  JsonDocument doc;
  auto error = deserializeJson(doc, configFile);
  configFile.close();
  if (error) {
    ESP_LOGE(TAG, "Failed to parse config file");
    return PARSE_JSON_ERROR;
  }

  SDConfigRecord config;
  SDConfStoreRetCode rc = fromJson(config, doc);
  if (rc != SUCCESS) {
    // Cannot parse config file content
    return rc;
  }
  _config = config;
  return SUCCESS;
}
