#include "ConfStore.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>

#include <esp_log.h>
static const char* TAG = "ConfStore";

static const char* CONFIG_FILE = "/config.json";

/**
 * Creates the configuration store
 */
ConfStore::ConfStore(void) {
  defaultConfig(_config);
}

/**
  * Returns the default config record
  */
ConfigRecord& ConfStore::defaultConfig(ConfigRecord& config) {
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
const ConfStoreRetCode ConfStore::fromJson(ConfigRecord& config, const JsonDocument& doc) {

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

JsonDocument& ConfStore::toJson(JsonDocument& result, const ConfigRecord& config) {
  result["version"] = config.version;
  result["wifiActive"] = config.wifiActive;
  result["wifiSsid"] = config.wifiSsid;
  result["wifiPassword"] = config.wifiPassword;
  result["mqttBrokerHost"] = config.mqttBrokerHost;
  result["mqttBrokerPort"] = config.mqttBrokerPort;
  result["mqttUser"] = config.mqttUser;
  result["mqttPassword"] = config.mqttPsw;
  return result;
}

/**
  * Initializes the config store
  */
void ConfStore::begin(void) {
  ESP_LOGD(TAG, "Mounting FS...");
  if (SPIFFS.begin()) {
    ESP_LOGD(TAG, "Mounted FS");
    loadConfig();
  }
}

/*
  * Stores the configuration
  * Returns 0 if success
  */
const ConfStoreRetCode ConfStore::store(const ConfigRecord& config) {
  _config = config;
  return saveConfig();
}

/*
   Saves configuration
   Returns true if successfuly saved
*/
const ConfStoreRetCode ConfStore::saveConfig(void) {
  StaticJsonDocument<256> doc;
  toJson(doc, _config);
  File configFile = SPIFFS.open(CONFIG_FILE, "w");
  if (!configFile) {
    ESP_LOGE(TAG, "!! Failed to open config file for writing");
    return CANNOT_WRITE_FILE;
  }
  serializeJson(doc, configFile);
  configFile.close();
  return SUCCESS;
}

/*
   Loads configuration
   Returns SUCCESS if successfuly saved
*/
const ConfStoreRetCode ConfStore::loadConfig(void) {
  if (!SPIFFS.exists(CONFIG_FILE)) {
    ESP_LOGE(TAG, "!! Config file not found");
    return FILE_NOT_FOUND;
  }
  File configFile = SPIFFS.open(CONFIG_FILE, "r");
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

  ConfigRecord config;
  ConfStoreRetCode rc = fromJson(config, doc);
  if (rc != SUCCESS) {
    // Cannot parse config file content
    return rc;
  }
  _config = config;
  return SUCCESS;
}
