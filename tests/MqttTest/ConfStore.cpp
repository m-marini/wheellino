#include "ConfStore.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>

#define DEBUG
#include <debug.h>

#define CONFIG_FILE "/config.json"

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
    .mqttBrokerUri = "brokerUri",
    .mqttBrokerPort = 1883,
    .mqttUser = "Wheelly",
    .mqttPsw = "",
  };
  return config;
}

/**
 * Converts the json doc to configuration record
 * Returns SUCCESS if success
 */
const ConfStoreRetCode ConfStore::fromJson(ConfigRecord& config, const StaticJsonDocument<200>& doc) {

#ifdef DEBUG
  Serial.println("Json file");
  serializeJson(doc, Serial);
  Serial.println();
#endif

  defaultConfig(config);
  config.version = doc["version"] | config.version;
  config.wifiActive = doc["wifiActive"] | config.wifiActive;
  config.wifiSsid = doc["wifiSsid"] | config.wifiSsid;
  config.wifiPassword = doc["wifiPassword"] | config.wifiPassword;
  config.mqttBrokerUri = doc["mqttBrokerUri"] | config.mqttBrokerUri;
  config.mqttBrokerPort = doc["mqttBrokerPort"] | config.mqttBrokerPort;
  config.mqttUser = doc["mqttUser"] | config.mqttUser;
  config.mqttPsw = doc["mqttPassword"] | config.mqttPsw;

  // Validates json fields
  if (config.version != CURRENT_VERSION) {
    Serial.println("!! Bad config file");
    return BAD_CONFIG_VERSION;
  }
  return SUCCESS;
}

StaticJsonDocument<200>& ConfStore::toJson(StaticJsonDocument<200>& result, const ConfigRecord& config) {
  result["version"] = CURRENT_VERSION;
  result["wifiActive"] = config.wifiActive;
  result["wifiSsid"] = config.wifiSsid;
  result["wifiPassword"] = config.wifiPassword;
  result["mqttBrokerUri"] = config.mqttBrokerUri;
  result["mqttBrokerPort"] = config.mqttBrokerPort;
  result["mqttUser"] = config.mqttUser;
  result["mqttPassword"] = config.mqttPsw;
  return result;
}

/**
  * Initializes the config store
  */
void ConfStore::begin(void) {
  loadConfig();
}

/*
   Saves configuration
   Returns true if successfuly saved
*/
const ConfStoreRetCode ConfStore::saveConfig(void) {
  StaticJsonDocument<200> doc;
  toJson(doc, _config);
  File configFile = SPIFFS.open(CONFIG_FILE, "w");
  if (!configFile) {
    DEBUG_PRINTLN("!! Failed to open config file for writing");
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
    Serial.println("!! Config file not found");
    return FILE_NOT_FOUND;
  }
  File configFile = SPIFFS.open(CONFIG_FILE, "r");
  size_t size = configFile.size();
  DEBUG_PRINT("// Config size: ");
  DEBUG_PRINT(size);
  DEBUG_PRINTLN();
  if (size > 1024) {
    Serial.println("!! Config file size is too large");
    configFile.close();
    return CONFIG_FILE_TOO_LARGE;
  }

  StaticJsonDocument<200> doc;
  auto error = deserializeJson(doc, configFile);
  configFile.close();
  if (error) {
    Serial.println("!! Failed to parse config file");
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
