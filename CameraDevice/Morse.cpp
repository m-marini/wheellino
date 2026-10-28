#include <WiFi.h>

#include "Morse.h"
#include "Camera.h"

#include <esp_log.h>
static const char* TAG = "Morse";

#define LED_INTENSITY 8
#define DOT_MILLIS 80  // 80 ms =  15 WPM

// Morse messages
static const char* errorMorse = "... --- ...   ... --- ...   ... --- ...";
static const char* startingMorse = ".";
static const char* startedMorse = ".-.-.";
static const char* startMorse = "-.-.-";

void flashMorse(const char* data) {
  while (*data) {
    switch (*data) {
      case '.':
        camera.flashing(1, LED_INTENSITY, DOT_MILLIS, DOT_MILLIS);
        break;
      case '-':
      case '_':
        camera.flashing(1, LED_INTENSITY, 3 * DOT_MILLIS, DOT_MILLIS);
        break;
      case ' ':
        vTaskDelay(2 * DOT_MILLIS / portTICK_PERIOD_MS);
        break;
      default:
        vTaskDelay(6 * DOT_MILLIS / portTICK_PERIOD_MS);
        break;
    }
    data++;
  }
}

void flashingError(void) {
  flashMorse(errorMorse);
}

void flashingStarting(void) {
  flashMorse(startingMorse);
}

void flashingStarted(void) {
  flashMorse(startedMorse);
}

void flashingStart(void) {
  flashMorse(startMorse);
}
