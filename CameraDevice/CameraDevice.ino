#include <WiFi.h>

#include "SDConfStore.h"
#include "Camera.h"
#include "Morse.h"
#include "MqttClient.h"

#include <esp_log.h>
static const char* TAG = "CameraDevice";

#define CONNECTION_TIMEOUT 10000
#define CONNECTION_INTERVAL 1000
#define RETRY_INTERVAL 3000
#define LOOP_INTERVAL 10
#define WATCHDOG_INTERVAL 30000

#define WHEELLY_MESSAGES_VERSION "v0"

SDConfStore confStore;

static boolean mqttConnected = false;
static char pubSensorTopicPrefix[64];
static char subCommandTopics[64];
static unsigned long watchdogTime;
static size_t jpgLen = 0;

// static functions
static const uint64_t macId(void);
static void onMessage(const char* topic, const char* message);
static void configureCamera(const char* topic, const char* arg);
static const size_t imageCallBack(void* ctx, const size_t index, const byte* data, const size_t len);
static bool endsWith(const char* text, const char* suffix);

void setup() {
  Serial.begin(115200);
  Serial.println();
  while (!Serial) {}
  delay(500);
  ESP_LOGI(TAG, "Start sequence.");

  ESP_LOGI(TAG, "Configuration store initialisation.");
  confStore.begin();

  // Initializing camrera
  ESP_LOGI(TAG, "Camera initialisation.");
  if (camera.begin()) {
    ESP_LOGE(TAG, "Camera initialisation failed.");
    delay(1000);
    ESP.restart();
  }
  ESP_LOGI(TAG, "Camera initialized.");
  // Flashing startup
  flashingStart();

  // Start Wifi
  const char* ssid = confStore.config().wifiSsid.c_str();
  ESP_LOGI(TAG, "Connecting WiFi %s ...", ssid);
  WiFi.begin(ssid, confStore.config().wifiPassword.c_str());
  WiFi.setSleep(false);

  // Wait for connection
  int ct = 0;
  unsigned long timeout = millis() + CONNECTION_TIMEOUT;
  while (WiFi.status() != WL_CONNECTED && millis() <= timeout) {
    vTaskDelay(CONNECTION_INTERVAL / portTICK_PERIOD_MS);
    flashingStarting();
    ESP_LOGI(TAG, "Connecting WiFi...");
  }
  if (WiFi.status() != WL_CONNECTED) {
    ESP_LOGE(TAG, "WiFi connection failed.");
    flashingError();
    ESP.restart();
  }
  ESP_LOGI(TAG, "WiFi connected");

  // Initialize MQTT
  ESP_LOGI(TAG, "Initialise mqtt client");
  const SDConfigRecord& mqttConfig = confStore.config();
  const uint64_t id = macId();
  snprintf(pubSensorTopicPrefix, sizeof(pubSensorTopicPrefix), "sens/wheellycam/%" PRIX64 "/" WHEELLY_MESSAGES_VERSION, id);
  snprintf(subCommandTopics, sizeof(subCommandTopics), "cmd/wheellycam/%" PRIX64 "/" WHEELLY_MESSAGES_VERSION "/+", id);
  ESP_LOGI(TAG, "Subtopics = [%s]", subCommandTopics);
  ESP_LOGI(TAG, "mqtt user = [%s]", mqttConfig.mqttPsw.c_str());
  mqttClient.begin(mqttConfig.mqttBrokerHost.c_str(), mqttConfig.mqttBrokerPort,
                   "wheellycam", mqttConfig.mqttUser.c_str(), mqttConfig.mqttPsw.c_str(),
                   subCommandTopics, 3000);
  mqttClient.onMessage(onMessage);

  // Connect MQTT
  ESP_LOGI(TAG, "Mqtt client connecting ...");
  mqttClient.connect();

  flashingStarted();
  ESP_LOGI(TAG, "Start sequence completed.");
}

void loop() {
  unsigned long t0 = millis();
  mqttClient.polling(t0);
  boolean conn = mqttClient.connected();
  if (conn != mqttConnected) {
    mqttConnected = conn;
    ESP_LOGI(TAG, "Mqtt client connected");
    if (conn) {
      char topic[64];
      strcpy(topic, pubSensorTopicPrefix);
      strcat(topic, "/hi");
      char msg[32];
      snprintf(msg, sizeof(msg), "%ul", t0);
      mqttClient.send(topic, msg);
      watchdogTime = t0 + WATCHDOG_INTERVAL;
    }
  }
  if (t0 >= watchdogTime) {
    char topic[64];
    strcpy(topic, pubSensorTopicPrefix);
    strcat(topic, "/wu");
    char msg[32];
    snprintf(msg, sizeof(msg), "%ul", t0);
    mqttClient.send(topic, msg);
    watchdogTime = t0 + WATCHDOG_INTERVAL;
  }
  vTaskDelay(LOOP_INTERVAL / portTICK_PERIOD_MS);
}


/**
* Call back jpeg stream
*
* @param ctx context
* @param index index
* @param data data buffer
* @param len data buffer len
*/
static const size_t imageCallBack(void* ctx, const size_t index, const byte* data, const size_t len) {
  if (!index) {
    jpgLen = 0;
  }
  if (!index) {
    jpgLen = 0;
  }
  jpgLen += len;
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/img", pubSensorTopicPrefix);
  mqttClient.send(topic, data, len);
  ESP_LOGI(TAG, "Jpg chunk index=%lu size=%lu tot%lu", (uint32_t)index, (uint32_t)len, (uint32_t)jpgLen);
  return len;
}

static void configureCamera(const char* topic, const char* arg) {
  char str[100];
  char replyTopic[64];
  unsigned int ledIntensity, frameSize, count;
  size_t argLen = strlen(arg);
  if (sscanf(arg, "%u,%u%n", &ledIntensity, &frameSize, &count) != 2 || count != argLen) {
    snprintf(str, sizeof(str), "Error in message %.40s", arg);
    snprintf(replyTopic, sizeof(replyTopic), "%s/err", topic);
    mqttClient.send(replyTopic, str);
    ESP_LOGE(TAG, "%s", str);
    return;
  }
  if (!(frameSize < 13)) {
    snprintf(str, sizeof(str), "Error in message %.40s", arg);
    snprintf(replyTopic, sizeof(replyTopic), "%s/err", topic);
    mqttClient.send(replyTopic, str);
    ESP_LOGE(TAG, "%s", str);
    return;
  }
  if (!(ledIntensity <= 255)) {
    snprintf(str, sizeof(str), "Error in message %.40s", arg);
    snprintf(replyTopic, sizeof(replyTopic), "%s/err", topic);
    mqttClient.send(replyTopic, str);
    ESP_LOGE(TAG, "%s", str);
    return;
  }

  ESP_LOGI(TAG, "Configuration ledIntensity=%u", ledIntensity);
  camera.ledIntensity(ledIntensity);

  const framesize_t framesize = (const framesize_t)frameSize;
  ESP_LOGI(TAG, "Configuration framesize=%u", frameSize);
  esp_err_t res = camera.frameSize(framesize);
  ESP_LOGI(TAG, "Configuration rc=%d", res);
  if (res) {
    snprintf(str, sizeof(str), "Error in message %.40s", arg);
    snprintf(replyTopic, sizeof(replyTopic), "%s/err", topic);
    mqttClient.send(replyTopic, str);
    ESP_LOGE(TAG, "%s", str);
    return;
  }

  snprintf(replyTopic, sizeof(replyTopic), "%s/res", topic);
  mqttClient.send(replyTopic, arg);
}

static void onMessage(const char* topic, const char* message) {
  ESP_LOGI(TAG, "Message from %s %s", topic, message);

  if (endsWith(topic, "/cf")) {
    configureCamera(topic, message);
  } else if (endsWith(topic, "/ca")) {
    camera.capture(imageCallBack, 0);
  }
}

static const uint64_t macId(void) {
  uint64_t mac = ESP.getEfuseMac();
  uint64_t mac1 = 0;
  mac1 |= (mac & 0xffff000000000000ul);
  mac1 |= (mac & 0x0000ff0000000000ul) >> 40;
  mac1 |= (mac & 0x000000ff00000000ul) >> 24;
  mac1 |= (mac & 0x00000000ff000000ul) >> 8;
  mac1 |= (mac & 0x0000000000ff0000ul) << 8;
  mac1 |= (mac & 0x000000000000ff00ul) << 24;
  mac1 |= (mac & 0x00000000000000fful) << 40;

  return mac1;
}

static bool endsWith(const char* string, const char* suffix) {
  const size_t strLen = strlen(string);
  const size_t suffixLen = strlen(suffix);

  return strLen >= suffixLen
         && strcmp(string + (strLen - suffixLen), suffix) == 0;
}