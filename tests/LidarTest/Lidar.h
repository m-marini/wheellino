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

#ifndef Lidar_h
#define Lidar_h

#include "Arduino.h"
#include "Adafruit_VL53L0X.h"
#include "Timer.h"

#define FRONT_LIDAR_ADDR 0x30
#define REAR_LIDAR_ADDR 0x31

/**
  Lidar (Light Detection and Ranging)
*/

class Lidar;

typedef void (*LidarRangeCallback_t)(void* context, Lidar& lidar, const uint16_t frontRange, const uint16_t rearRange);

/**
* Controls two Lidar sensors placed in the front and in the rear of rotating head
*/
class Lidar {
private:
  uint8_t _frontXShutPin;
  uint8_t _rearXShutPin;
  uint8_t _frontAddr;
  uint8_t _rearAddr;
  bool _active;
  LidarRangeCallback_t _onRange;
  void* _onRangeContext;
  Timer _timer;
  Adafruit_VL53L0X _loxFront;
  Adafruit_VL53L0X _loxRear;
  VL53L0X_RangingMeasurementData_t _frontMeasure;
  VL53L0X_RangingMeasurementData_t _rearMeasure;

public:

  /**
    Creates the lidar controller
  */
  Lidar(const uint8_t frontXShutPin = 0,
        const uint8_t rearXShutPin = 0,
        const uint8_t frontAddr = FRONT_LIDAR_ADDR,
        const uint8_t rearAddr = REAR_LIDAR_ADDR);

  /**
    Begins the controller returning true if success
  */
  const boolean begin(void);

  /**
  * Reads measure returning the VL53L0X return code (0 = success)
  */
  const VL53L0X_Error readMeasure(void);

  /**
  * Sets the on range callback
  *
  * @param callback the call back
  * @param context the context
  */
  void onRange(LidarRangeCallback_t callback, void* context = NULL) {
    _onRange = callback;
    _onRangeContext = context;
  }

  /**
  * Activates the lidar
  */
  void activate(void) {
    _active = true;
  }

  /**
  * Inactivates the lidar
  */
  void inactivate(void) {
    _active = false;
  }

  /**
  * Sets the measure interval
  */
  void interval(const unsigned long interval) {
    _timer.interval(interval);
  }

  /**
    Polls the controller
  */
  void polling(const unsigned long t0 = millis());

  /**
    Returns true if has rear lidar
  */
  const boolean hasRear(void) const {
    return _rearXShutPin > 0;
  }

  /**
  * Returns the front measure
  */
  const VL53L0X_RangingMeasurementData_t& frontMeasure(void) const {
    return _frontMeasure;
  }

  /**
  * Returns the rear measure
  */
  const VL53L0X_RangingMeasurementData_t& rearMeasure(void) const {
    return _rearMeasure;
  }

  /**
  * Returns the front distance (mm)
  */
  const uint16_t frontDistance(void) const;

  /**
  * Returns the rear distance (mm)
  */
  const uint16_t rearDistance(void) const;

  /**
  * Returns true if the lidar is active
  */
  const bool active(void) const {return _active;}
};

#endif