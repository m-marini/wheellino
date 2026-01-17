#include "esp_camera.h"
#include <WiFi.h>

#include "SDConfStore.h"
#include "MqttClient.h"
#include "Camera.h"

#include <esp_log.h>
static const char* TAG = "CameraDevice";

#define CONNECTION_TIMEOUT 10000
#define CONNECTION_INTERVAL 1000
#define WATCHDOG_INTERVAL 30000
#define LOOP_INTERVAL 10
#define DOT_MILLIS 80  // 80 ms =  15 WPM

static const char* WHEELLY_MESSAGES_VERSION = "v0";

SDConfStore confStore;

static boolean mqttConnected = false;
static String pubSensorTopicPrefix;
static String subCommandTopics;
static unsigned long watchdogTime;
static size_t jpgLen = 0;

// static functions
static const String macId(void);
static void onMessage(const String& topic, const String& message);
static void configureCamera(const String& topic, const String& arg);
static const size_t imageCallBack(void* ctx, const size_t index, const byte* data, const size_t len);
static void flashingStarting(void);
static void flashingStarted(void);
static void flashingError(void);

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
    flashingError();
    ESP.restart();
  }
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
  const String id = macId();
  pubSensorTopicPrefix = "sens/wheellycam/" + id + "/" + WHEELLY_MESSAGES_VERSION;
  subCommandTopics = "cmd/wheellycam/" + id + "/" + WHEELLY_MESSAGES_VERSION + "/+";
  ESP_LOGD(TAG, "Subtopics = %s", subCommandTopics.c_str());
  mqttClient.begin(mqttConfig.mqttBrokerHost, mqttConfig.mqttBrokerPort,
                   "wheellycam", mqttConfig.mqttUser, mqttConfig.mqttPsw,
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
      mqttClient.send(pubSensorTopicPrefix + "/hi", String(t0));
      watchdogTime = t0 + WATCHDOG_INTERVAL;
    }
  }
  if (t0 >= watchdogTime) {
    mqttClient.send(pubSensorTopicPrefix + "/wu", String(t0));
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
  jpgLen += len;
  mqttClient.send(pubSensorTopicPrefix + "/img", data, len);
  ESP_LOGI(TAG, "Jpg chunk index=%lu size=%lu tot%lu", (uint32_t)index, (uint32_t)len, (uint32_t)jpgLen);
  return len;
}

static void configureCamera(const String& topic, const String& arg) {
  char str[100];
  unsigned int ledIntensity, frameSize, count;
  if (sscanf(arg.c_str(), "%u,%u%n", &ledIntensity, &frameSize, &count) != 2 || count != arg.length()) {
    sprintf(str, "Error in message %.40s", arg.c_str());
    ESP_LOGE(TAG, "%s", str);
    mqttClient.send(topic + "/err", str);
    return;
  }
  if (!(frameSize < 13)) {
    sprintf(str, "Wrong frame size %u", frameSize);
    ESP_LOGE(TAG, "%s", str);
    mqttClient.send(topic + "/err", str);
    return;
  }
  if (!(ledIntensity <= 255)) {
    sprintf(str, "Wrong led intensity %u", ledIntensity);
    ESP_LOGE(TAG, "%s", str);
    mqttClient.send(topic + "/err", str);
    return;
  }

  ESP_LOGI(TAG, "Configuration ledIntensity=%u", ledIntensity);
  camera.ledIntensity(ledIntensity);

  const framesize_t framesize = (const framesize_t)frameSize;
  ESP_LOGI(TAG, "Configuration framesize=%u", frameSize);
  esp_err_t res = camera.frameSize(framesize);
  ESP_LOGI(TAG, "Configuration rc=%d", res);
  if (res) {
    sprintf(str, "Error configuring camera rc=%d", res);
    ESP_LOGE(TAG, "%s", str);
    mqttClient.send(topic + "/err", str);
    return;
  }
  mqttClient.send(topic + "/res", arg);
}

static void onMessage(const String& topic, const String& message) {
  ESP_LOGI(TAG, "Message from %s %s", topic.c_str(), message.c_str());
  if (topic.endsWith("/cf")) {
    configureCamera(topic, message);
  } else if (topic.endsWith("/ca")) {
    camera.capture(imageCallBack, 0);
    mqttClient.send(topic + "/res", message);
  }
}

static const String macId(void) {
  uint64_t mac = ESP.getEfuseMac();
  uint64_t mac1 = 0;
  mac1 |= (mac & 0xffff000000000000ul);
  mac1 |= (mac & 0x0000ff0000000000ul) >> 40;
  mac1 |= (mac & 0x000000ff00000000ul) >> 24;
  mac1 |= (mac & 0x00000000ff000000ul) >> 8;
  mac1 |= (mac & 0x0000000000ff0000ul) << 8;
  mac1 |= (mac & 0x000000000000ff00ul) << 24;
  mac1 |= (mac & 0x00000000000000fful) << 40;

  return String(mac1, HEX);
}

static void flashMorse(const char* data) {
  while (*data) {
    switch (*data) {
      case '.':
        camera.flashing(1, 8, DOT_MILLIS, DOT_MILLIS);
        break;
      case '-':
        camera.flashing(1, 8, 3 * DOT_MILLIS, DOT_MILLIS);
        break;
      case ' ':
        vTaskDelay(2 * DOT_MILLIS / portTICK_PERIOD_MS);
        break;
      case '_':
      case '/':
        vTaskDelay(6 * DOT_MILLIS / portTICK_PERIOD_MS);
        break;
    }
    data++;
  }
}

static void flashingError(void) {
  flashMorse("... --- .../... --- .../... --- ...");
}

static void flashingStarting(void) {
  flashMorse(".");
}

static void flashingStarted(void) {
  flashMorse(".-.-.");
}

static void flashingStart(void) {
  flashMorse("-.-.-");
}
