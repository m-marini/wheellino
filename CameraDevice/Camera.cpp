#include <Arduino.h>

#include <esp_log.h>
static const char *TAG = "Camera";

#include "esp_camera.h"
#include "esp32-hal-ledc.h"

#define CAMERA_MODEL_AI_THINKER  // Has PSRAM

#include "camera_pins.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

// Enable LED FLASH setting
#define CONFIG_LED_ILLUMINATOR_ENABLED 1

// LED FLASH setup
#if CONFIG_LED_ILLUMINATOR_ENABLED

#define LED_LEDC_GPIO 4  //configure LED pin
//#define LED_LEDC_GPIO            22  //configure LED pin
#define CONFIG_LED_MAX_INTENSITY 255

#endif

#include "Camera.h"

Camera camera;

void Camera::lightLed(const int intensity) {  // Turn LED On or Off
#if CONFIG_LED_ILLUMINATOR_ENABLED
  int duty = intensity;
  if (duty > CONFIG_LED_MAX_INTENSITY) {
    duty = CONFIG_LED_MAX_INTENSITY;
  }
  ledcWrite(LED_LEDC_GPIO, duty);
  //ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
  //ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
  ESP_LOGD(TAG, "Set LED intensity to %d", duty);
#endif
}

void Camera::flashing(const int n, const int intensity, const int on, const int off) {
  for (int i = 0; i < n; i++) {
    lightLed(intensity);
    vTaskDelay(on / portTICK_PERIOD_MS);
    lightLed(0);
    vTaskDelay(off / portTICK_PERIOD_MS);
  }
}

const esp_err_t Camera::begin(void) {

#if CONFIG_LED_ILLUMINATOR_ENABLED
  ledcAttach(LED_GPIO_NUM, 5000, 8);
#else
  ESP_LOGI(TAG, "LED flash is disabled -> CONFIG_LED_ILLUMINATOR_ENABLED = 0");
#endif

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed with error %d", err);
    return err;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif
  return err;
}

const esp_err_t Camera::frameSize(const framesize_t value) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    ESP_LOGE(TAG, "Camera sensor not found");
    return ESP_FAIL;
  }
  if (s->pixformat != PIXFORMAT_JPEG) {
    ESP_LOGE(TAG, "Camera sensor pixformat %d not supported", s->pixformat);
    return ESP_FAIL;
  }
  return s->set_framesize(s, value);
}

const esp_err_t Camera::capture(const size_t (*callback)(void *ctx, const size_t index, const byte *data, const size_t len), void *context) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;

#if CONFIG_LED_ILLUMINATOR_ENABLED
  lightLed(_ledIntensity);
  vTaskDelay(150 / portTICK_PERIOD_MS);  // The LED needs to be turned on ~150ms before the call to esp_camera_fb_get()
  fb = esp_camera_fb_get();              // or it won't be visible in the frame. A better way to do this is needed.
  lightLed(0);
#else
  fb = esp_camera_fb_get();
#endif

  if (!fb) {
    ESP_LOGE(TAG, "Camera capture failed");
    return ESP_FAIL;
  }

  if (fb->format == PIXFORMAT_JPEG) {
    ESP_LOGI(TAG, "PIXFORMAT_JPEG size=%lu", fb->len);
    callback(context, 0, fb->buf, fb->len);
  } else {
    ESP_LOGE(TAG, "Format not recognized format=%d size=%lu", (int)fb->format, (uint32_t)fb->len);
    //res = frame2jpg_cb(fb, 80, (jpg_out_cb) callback, context) ? ESP_OK : ESP_FAIL;
    res = ESP_FAIL;
  }
  esp_camera_fb_return(fb);
  return res;
}
