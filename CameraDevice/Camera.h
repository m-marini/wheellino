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

#ifndef _Camera_h_
#define _Camera_h_

#include <Arduino.h>
#include "esp_camera.h"

#define CONFIG_LED_MAX_INTENSITY 255

enum capture_type_t {
  SINGLE_FRAME,
  CHUNK_CAPTURE,
  END_CAPTURE
};

typedef struct {
  capture_type_t type;
  size_t index;
  const void *data;
  size_t len;
} capture_data_t;

typedef struct {
  int frameSize;
  int vflip;
  int hmirror;
  int brightness;
  int contrast;
  int saturation;
  int flashIntensity;
  int jpgQuality;
} image_properties_t;

typedef size_t (*capture_cb)(void *context, capture_data_t *capture_data);

/**
     * @brief Initialises the camera hardware module.
     * 
     * Configures the pins and sensor parameters based on the current definitions.
     * 
     * @return const esp_err_t ESP_OK on success, or an error code on failure.
     */
const esp_err_t cameraBegin(void);

/**
 * @brief Captures a frame from the sensor, modulates the flash, and converts it to JPEG whilst distributing data via a callback.
 * 
 * This function acquires the native frame buffer from the camera. If the source format
 * is not already JPEG (e.g., RGB or YUV), it performs "on-the-fly" JPEG compression using the
 * framework's fmt2jpg_cb() function, which significantly optimises RAM consumption.
 * The compressed data chunks are emitted sequentially.
 * 
 * @note [ESP32-CAM AI-Thinker Hardware Specifics]
 * The Flash LED shares GPIO 4const size with the D1 data line of the MicroSD card. If this function 
 * is used with flashIntensity > 0 and the callback writes to the SD card, the card must 
 * be initialised strictly in 1-bit mode using: SD_MMC.begin("/sdcard", true);
 * 
 * @param callback       A function pointer of type capture_cb to process individual emitted JPEG chunks.
 *                       Must return 'len' to proceed, or a smaller value to abort the sequence.
 * @param context        Opaque pointer (void*) passed directly as an argument to the callback 
 *                       (e.g., pointer to an SD File object or a network socket descriptor).
 * @param flashIntensity Brightness of the Flash LED (GPIO 4) from 0 (off) to 255 (maximum PWM power).
 *                       Default to 0.
 * @param jpgQuality     JPEG compression factor ranging from 1 (lowest quality, smaller file) to 100 
 *                       (highest quality, larger file). Defaults to 80.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Capture, compression, and data streaming completed successfully.
 *         - ESP_ERR_CAMERA_NOT_DETECTED: Camera sensor is unreachable or uninitialised.
 *         - ESP_ERR_CAMERA_FAILED_TO_GET_FRAME: Hardware timeout whilst retrieving the frame buffer.
 *         - ESP_FAIL: Compression or streaming was prematurely aborted by the callback (e.g., target write error).
 */
const esp_err_t cameraCapture(capture_cb callback, void *context, image_properties_t &props);
const esp_err_t cameraSetSensor(image_properties_t &props);

/**
     * @brief Turns on the flash LED at the specified intensity.
     * 
     * Controls the hardware (e.g., via PWM) to illuminate the onboard LED.
     * 
     * @param intensity Brightness level (0 to switch off, 255 for maximum brightness).
     */
void lightLed(const int intensity);

/**
     * @brief Performs a controlled flashing of the flash LED.
     * 
     * Temporarily blocks execution to flash the LED for a defined number of cycles.
     * Useful as a visual status indicator or a countdown signal.
     * 
     * @param n Number of flashes to execute.
     * @param intensity Luminous intensity of the flashes (default: 255).
     * @param on Active duration in milliseconds for each cycle (default: 20 ms).
     * @param off Inactive duration in milliseconds for each cycle (default: 80 ms).
     */
void flashing(const int n, const int intensity = 255, const int on = 20, const int off = 80);

/**
 * @brief Transmits a pre-encoded Morse code string (. and -) via the onboard LED.
 * 
 * This function directly parses a string consisting of literal Morse symbols.
 * It illuminates the LED for a single time unit for a dot ('.') and three units 
 * for a dash ('-'), adhering to international timing regulations.
 * 
 * @details The function interprets standard spacing markers within the stream:
 *          - Single space (' '): Represents an inter-character pause (3 units).
 *          - Forward slash ('/'): Represents an inter-word pause (7 units).
 *          - Unknown tokens: Safely skipped to maintain timing integrity.
 * 
 * @note This is a blocking function due to synchronous hardware delay mechanisms.
 * 
 * @param[in] code Pointer to the null-terminated sequence of raw Morse symbols.
 */
void morse(const char *code);

#endif