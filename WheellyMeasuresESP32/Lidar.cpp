/*
 * Copyright (c) 2025  Marco Marini, marco.marini@mmarini.org
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

#include "Lidar.h"

#include <Wire.h>

//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <esp_log.h>
static char* TAG = "Lidar";

//#define SIMULATED
#define DEFAULT_INTERVAL 50

/**
  * Sets up single lidar sensor
  *
  * @param lox the sensor
  * @param pin the XShut pin of the sensor
  * @param addr the I2C address of the sensor
  */
static const boolean setupSingle(Adafruit_VL53L0X& lox, const uint8_t pin, const uint8_t addr) {
  // Reset pin for 10 ms
  if (pin > 0) {
    ESP_LOGD(TAG, "Reset VL53L0X pin 0x%hhx", pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delay(10);
    ESP_LOGD(TAG, "Activate VL53L0X pin 0x%hhx", pin);
    digitalWrite(pin, HIGH);
    delay(10);
  }

  // initing
  ESP_LOGD(TAG, "Init VL53L0X @ 0x%hhx", addr);
  boolean rc = lox.begin(addr, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  if (!rc) {
    ESP_LOGE(TAG, "Failed to init VL53L0X at 0x%hhx", addr);
  }
  return rc;
}

/**
  * Sets up dual lidar sensors
  *
  * @param lox0 the first sensor
  * @param pin0 the XShut pin of the first sensor
  * @param addr0 the I2C address of the first sensor
  * @param lox1 the second sensor
  * @param pin1 the XShut pin of the second sensor
  * @param addr1 the I2C address  of the second sensor
  */
static const boolean setupDual(Adafruit_VL53L0X& lox0, const uint8_t pin0, const uint8_t addr0, Adafruit_VL53L0X& lox1, const uint8_t pin1, const uint8_t addr1) {
  // Reset pin for 10 ms
  pinMode(pin0, OUTPUT);
  pinMode(pin1, OUTPUT);
  ESP_LOGD(TAG, "Reset VL53L0X pins 0x%hhx 0x%hhx", pin0, pin1);
  digitalWrite(pin0, LOW);
  digitalWrite(pin1, LOW);
  delay(10);

  // all unreset
  ESP_LOGD(TAG, "Activate VL53L0X pins 0x%hhx 0x%hhx", pin0, pin1);
  digitalWrite(pin0, HIGH);
  digitalWrite(pin1, HIGH);
  delay(10);

  // activating pin0 and reset pin1
  ESP_LOGD(TAG, "Reset VL53L0X pin 0x%hhx", pin1);
  digitalWrite(pin1, LOW);

  // initing lox0
  ESP_LOGD(TAG, "Init VL53L0X pin 0x%hhx @ 0x%hhx", pin0, addr0);
  boolean rc = lox0.begin(addr0, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  if (!rc) {
    ESP_LOGE(TAG, "Failed to init VL53L0X pin 0x%hhx @ 0x%hhx", pin0, addr0);
    return rc;
  }

  delay(10);
  ESP_LOGD(TAG, "Activate VL53L0X pin 0x%hhx", pin1);
  digitalWrite(pin1, HIGH);
  delay(10);

  ESP_LOGD(TAG, "Init VL53L0X pin 0x%hhx @ 0x%hhx", pin1, addr1);
  rc = lox1.begin(addr1, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  if (!rc) {
    ESP_LOGE(TAG, "Failed to init VL53L0X pin 0x%hhx @ 0x%hhx", pin1, addr1);
  }
  return rc;
}

/**
    Creates the lidar controller
  */
Lidar::Lidar(const uint8_t frontXShutPin,
             const uint8_t rearXShutPin,
             const uint8_t frontAddr,
             const uint8_t rearAddr)
  : _frontAddr(frontAddr), _rearAddr(rearAddr), _frontXShutPin(frontXShutPin), _rearXShutPin(rearXShutPin) {
  _timer.interval(DEFAULT_INTERVAL);
  _timer.continuous(true);
  _timer.onNext([](void* context, const unsigned long counter) {
    ESP_LOGD(TAG, "Read lidar");
    ((Lidar*)context)->readMeasure();
  },
                this);
}

/**
    Begins the controller returning true if success
  */
const boolean Lidar::begin(void) {
  ESP_LOGI(TAG, "Begin front(pin %hhu, I2C@0x%hhx), rear(pin %hhu, I2C@0x%hhx)", _frontXShutPin, _frontAddr, _rearXShutPin, _rearAddr);
  _active = true;
  _timer.start();

#ifdef SIMULATED
  ESP_LOGW(TAG, "Simulated");
  return true;
#else
  return (_rearXShutPin == 0)
           ? setupSingle(_loxFront, _frontXShutPin, _frontAddr)
           : setupDual(_loxFront, _frontXShutPin, _frontAddr, _loxRear, _rearXShutPin, _rearAddr);
#endif
}

/**
  * Reads measure returning the VL53L0X return code (0 = success)
  */
const VL53L0X_Error Lidar::readMeasure(void) {
#ifdef SIMULATED
  if (_onRange) {
    _onRange(_onRangeContext, *this, frontDistance(), rearDistance());
  }
  return 0;
#else
  VL53L0X_Error rc = _loxFront.rangingTest(&_frontMeasure, false);
  if (rc) {
    ESP_LOGE(TAG, "Error 0x%x reading front lidar", (unsigned int)rc);
    return rc;
  }
  if (hasRear()) {
    rc = _loxRear.rangingTest(&_rearMeasure, false);
    if (rc) {
      ESP_LOGE(TAG, "Error 0x%x reading rear lidar", (unsigned int)rc);
      return rc;
    }
  }
  if (_onRange) {
    _onRange(_onRangeContext, *this, frontDistance(), rearDistance());
  }
  return rc;
#endif
}

/**
    Polls the controller
  */
void Lidar::polling(const unsigned long t0) {
  if (_active) {
    _timer.polling(t0);
  }
}

/**
  * Returns the front distance (mm)
  */
const uint16_t Lidar::frontDistance(void) const {
  return _frontMeasure.RangeStatus != 4 ? _frontMeasure.RangeMilliMeter : 0;
}

/**
  * Returns the rear distance (mm)
  */
const uint16_t Lidar::rearDistance(void) const {
  return _rearMeasure.RangeStatus != 4 ? _rearMeasure.RangeMilliMeter : 0;
}
