#include "ConfStore.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>

#include <esp_log.h>
static const char* TAG = "ConfStore";

static const char* CONFIG_FILE = "/config.json";

static ConfigRecord _config;

/*
   Loads configuration
   Returns SUCCESS if successfuly saved
*/
static const ConfStoreRetCode loadConfig(void) {
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
  ConfStoreRetCode rc = ConfStore::fromJson(config, doc);
  if (rc != SUCCESS) {
    // Cannot parse config file content
    return rc;
  }
  _config = config;
  return SUCCESS;
}

/*
   Saves configuration
   Returns true if successfuly saved
*/
static const ConfStoreRetCode saveConfig(void) {
  StaticJsonDocument<256> doc;
  ConfStore::toJson(doc, _config);
  File configFile = SPIFFS.open(CONFIG_FILE, "w");
  if (!configFile) {
    ESP_LOGE(TAG, "!! Failed to open config file for writing");
    return CANNOT_WRITE_FILE;
  }
  serializeJson(doc, configFile);
  configFile.close();
  return SUCCESS;
}

/**
 * Creates the configuration store
 */
ConfStore::ConfStore(void) {
  defaultConfig(_config);
}

const ConfigRecord& ConfStore::config() const {
  return _config;
}

/**
  * Returns the default config record
  */
ConfigRecord& ConfStore::defaultConfig(ConfigRecord& config) {
  config = {
    .mqttBrokerPort = 1883,
    .wifiActive = false,
  };
  snprintf(config.version, sizeof(config.version), "%s", CURRENT_VERSION);
  snprintf(config.wifiSsid, sizeof(config.wifiSsid), "%s", "Wheelly");
  snprintf(config.wifiPassword, sizeof(config.wifiPassword), "%s", "");
  snprintf(config.mqttBrokerHost, sizeof(config.mqttBrokerHost), "%s", "brokerHost");
  snprintf(config.mqttPsw, sizeof(config.mqttPsw), "%s", "wheelly");
  return config;
}

/**
 * Converts the json doc to configuration record
 * Returns SUCCESS if success
 */
const ConfStoreRetCode ConfStore::fromJson(ConfigRecord& config, const JsonDocument& doc) {

  ESP_LOGD(TAG, "Json file %s", doc);

  defaultConfig(config);
  snprintf(config.version, sizeof(config.version), "%s", doc["version"] | config.version);
  snprintf(config.wifiSsid, sizeof(config.wifiSsid), "%s", doc["wifiSsid"] | config.wifiSsid);
  snprintf(config.wifiPassword, sizeof(config.wifiPassword), "%s", doc["wifiPassword"] | config.wifiPassword);
  snprintf(config.mqttBrokerHost, sizeof(config.mqttBrokerHost), "%s", doc["mqttBrokerHost"] | config.mqttBrokerHost);
  snprintf(config.mqttUser, sizeof(config.mqttUser), "%s", doc["mqttUser"] | config.mqttUser);
  snprintf(config.mqttPsw, sizeof(config.mqttPsw), "%s", doc["mqttPassword"] | config.mqttPsw);
  config.mqttBrokerPort = doc["mqttBrokerPort"] | config.mqttBrokerPort;
  config.wifiActive = doc["wifiActive"] | config.wifiActive;

  // Validates json fields
  if (strcmp(config.version, CURRENT_VERSION)) {
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
  ESP_LOGI(TAG, "Begin");
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

/**
* Configuration store
*/
ConfStore confStore;
