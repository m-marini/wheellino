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
    .version = CURRENT_VERSION,
    .wifiActive = false,
    .wifiSsid = "Wheelly",
    .wifiPassword = "",
    .mqttBrokerHost = "brokerHost",
    .mqttBrokerPort = 1883,
    .mqttUser = "wheelly",
    .mqttPsw = "",
  };
  return config;
}

/**
 * Converts the json doc to configuration record
 * Returns SUCCESS if success
 */
const SDConfStoreRetCode SDConfStore::fromJson(SDConfigRecord& config, const JsonDocument& doc) {

  ESP_LOGD(TAG, "Json file %s", doc);

  defaultConfig(config);
  config.version = doc["version"] | config.version;
  config.wifiActive = doc["wifiActive"] | config.wifiActive;
  config.wifiSsid = doc["wifiSsid"] | config.wifiSsid;
  config.wifiPassword = doc["wifiPassword"] | config.wifiPassword;
  config.mqttBrokerHost = doc["mqttBrokerHost"] | config.mqttBrokerHost;
  config.mqttBrokerPort = doc["mqttBrokerPort"] | config.mqttBrokerPort;
  config.mqttUser = doc["mqttUser"] | config.mqttUser;
  config.mqttPsw = doc["mqttPassword"] | config.mqttPsw;

  // Validates json fields
  if (config.version != CURRENT_VERSION) {
    ESP_LOGE(TAG, "Bad config file");
    return BAD_CONFIG_VERSION;
  }
  return SUCCESS;
}

/**
  * Initializes the config store
  */
void SDConfStore::begin(void) {
  ESP_LOGD(TAG, "Mounting MicroSD Card");
  if (!SD_MMC.begin()) {
    ESP_LOGE(TAG, "MicroSD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    ESP_LOGE(TAG, "No MicroSD Card found");
    return;
  }
  ESP_LOGD(TAG, "Mounted MicroSD Card");
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
