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

#include <esp_log.h>
static const char *TAG = "Camera";

#include <Arduino.h>
#include "esp32-hal-ledc.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

#include "esp_camera.h"

// ===========================
// Select camera model
// ===========================
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM
#include "camera_pins.h"

#include "Camera.h"

// LED FLASH setup
#define LED_LEDC_GPIO 4  //configure LED pin
#define LED_INTENSITY 8
#define DOT_MILLIS 80  // 80 ms =  15 WPM
#define FLASH_DELAY 150

const esp_err_t cameraBegin(void) {
  static camera_config_t config;
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
    ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
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
  /*
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 10);
  s->set_gainceiling(s, GAINCEILING_8X);
*/
  ledcAttach(LED_GPIO_NUM, 5000, 8);

  lightLed(0);

  return ESP_OK;
}

/**
 * @brief Adjusts the physical brightness of the onboard illumination LED.
 * 
 * This function clamps the requested input intensity value to a safe hardware boundary 
 * using the `constrain` macro before committing the calculated duty cycle value straight 
 * to the underlying ESP32 LEDC PWM peripheral driver.
 * 
 * @details The boundary limits are anchored between a absolute minimum value of 0 (fully off) 
 *          and a maximum hardware ceiling configuration defined by CONFIG_LED_MAX_INTENSITY.
 * 
 * @param[in] intensity The target brightness level requested by the application layer.
 */
void lightLed(const int intensity) {
  int duty = constrain(intensity, 0, CONFIG_LED_MAX_INTENSITY);
  ledcWrite(LED_LEDC_GPIO, duty);
  //ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
  //ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
  ESP_LOGD(TAG, "Set LED intensity to %d", duty);
}

/**
 * @brief Performs a controlled flashing sequence of the onboard illumination LED.
 * 
 * This function cycles the LED state for a specified number of repetitions. It utilizes 
 * FreeRTOS task delays rather than standard blocking loops, allowing the microcontroller 
 * scheduler to execute lower or equal priority tasks concurrently during inactive frames.
 * 
 * @details The function handles timing calculations directly by translating millisecond 
 *          input values into standard FreeRTOS scheduler system clock ticks.
 * 
 * @param[in] n         Total number of flash cycles to execute.
 * @param[in] intensity Luminous intensity level applied to the LED during active phases (0 to CONFIG_LED_MAX_INTENSITY).
 * @param[in] on        Active 'on' duration for each individual pulse frame, specified in milliseconds.
 * @param[in] off       Inactive 'off' duration gap between consecutive pulses, specified in milliseconds.
 */
void flashing(const int n, const int intensity, const int on, const int off) {
  for (int i = 0; i < n; i++) {
    lightLed(intensity);
    vTaskDelay(on / portTICK_PERIOD_MS);
    lightLed(0);
    vTaskDelay(off / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Parses and transmits a pre-encoded Morse code string via the illumination LED.
 * 
 * This function iterates through a null-terminated stream of raw Morse tokens, converting 
 * visual symbols into physical flashes and timed pauses. It delegates the hardware pulsing 
 * to the `flashing` subsystem and manages structural gaps utilizing FreeRTOS task delays.
 * 
 * @details The function maps tokens using the standard international Morse relative timing framework:
 *          - Dot ('.'): 1 unit active flash, followed by a 1-unit implicit trailing gap.
 *          - Dash ('-' or '_'): 3 units active flash, followed by a 1-unit implicit trailing gap.
 *          - Inter-character space (' '): Inserts an extra 2 units of delay. Combined with the 
 *            1-unit trailing gap from the previous symbol, this satisfies the standard 3-unit letter gap.
 *          - Inter-word slash ('/'): Inserts an extra 6 units of delay. Combined with the 
 *            1-unit trailing gap from the previous symbol, this satisfies the standard 7-unit word gap.
 *          - Unknown tokens: Safely skipped to maintain pipeline execution integrity.
 * 
 * @param[in] data Pointer to the null-terminated character string containing the raw Morse sequence.
 */
void morse(const char *data) {
  while (*data) {
    switch (*data) {
      case '.':
        flashing(1, LED_INTENSITY, DOT_MILLIS, DOT_MILLIS);
        break;
      case '-':
      case '_':
        flashing(1, LED_INTENSITY, 3 * DOT_MILLIS, DOT_MILLIS);
        break;
      case ' ':
        vTaskDelay(2 * DOT_MILLIS / portTICK_PERIOD_MS);
        break;
      case '/':
        vTaskDelay(6 * DOT_MILLIS / portTICK_PERIOD_MS);
        break;
      default:
        break;
    }
    data++;
  }
}

typedef struct {
  capture_cb callback;
  void *context;
} chunk_context_t;

static size_t chunk_cb(void *context, size_t index, const void *data, size_t len) {
  capture_data_t record = {
    .type = CHUNK_CAPTURE,
    .index = index,
    .data = data,
    .len = len,
  };
  chunk_context_t *ctx = (chunk_context_t *)context;
  return ctx->callback(ctx->context, &record);
}

const esp_err_t cameraSetSensor(image_properties_t &props) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    ESP_LOGE(TAG, "Sensor not found");
    return ESP_FAIL;
  }
  if (s->pixformat != PIXFORMAT_JPEG) {
    ESP_LOGE(TAG, "Invalid pixformat");
    return ESP_FAIL;
  }
  esp_err_t res = s->set_framesize(s, (framesize_t)props.frameSize);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Unable to set framesize rc=%d", res);
  }
  res = s->set_vflip(s, props.vflip);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Unable to set vflip rc=%d", res);
  }
  res = s->set_hmirror(s, props.hmirror);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Unable to set hmirror rc=%d", res);
  }
  res = s->set_brightness(s, props.brightness);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Unable to set brightness rc=%d", res);
  }
  res = s->set_contrast(s, props.contrast);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Unable to set contrast rc=%d", res);
  }
  res = s->set_saturation(s, props.saturation);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Unable to set saturation rc=%d", res);
  }
  return res;
}

const esp_err_t cameraCapture(capture_cb callback, void *context, image_properties_t &props) {
  // Sets sensor properties
  esp_err_t res = cameraSetSensor(props);
  if (res != ESP_OK) {
    return res;
  }

  // Flash on
  lightLed(props.flashIntensity);
  vTaskDelay(FLASH_DELAY / portTICK_PERIOD_MS);  // The LED needs to be turned on ~150ms before the call to esp_camera_fb_get()
                                                 // or it won't be visible in the frame. A better way to do this is needed.
  camera_fb_t *fb = esp_camera_fb_get();
  // Flash off
  lightLed(0);

  if (!fb) {
    ESP_LOGE(TAG, "Camera capture failed");
    return ESP_FAIL;
  }
  capture_data_t record;
  if (fb->format == PIXFORMAT_JPEG) {
    record = {
      .type = SINGLE_FRAME,
      .index = 0,
      .data = fb->buf,
      .len = fb->len
    };
    callback(context, &record);
  } else {
    chunk_context_t ctx = { .callback = callback, .context = context };
    res = frame2jpg_cb(fb, props.jpgQuality, chunk_cb, &ctx) ? ESP_OK : ESP_FAIL;
    record = {
      .type = END_CAPTURE,
      .index = 0,
      .data = NULL,
      .len = 0
    };
    callback(context, &record);
  }
  esp_camera_fb_return(fb);
  return res;
}