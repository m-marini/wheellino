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
    Creates the Lidar
  */
  Lidar(const uint8_t frontXShutPin = 0,
        const uint8_t rearXShutPin = 0,
        const uint8_t frontAddr = FRONT_LIDAR_ADDR,
        const uint8_t rearAddr = REAR_LIDAR_ADDR)
    : _frontAddr(frontAddr), _rearAddr(rearAddr), _frontXShutPin(frontXShutPin), _rearXShutPin(rearXShutPin) {
  }

  /**
    Begins the lidar returning true if success
  */
  const boolean begin(void);

  /**
  * Reads measure returning the VL53L0X return code
  */
  const VL53L0X_Error readMeasure(void);

  /**
  * Sets the on range callback
  * @param callback the call back
  * @param context the context
  */
  void onRange(LidarRangeCallback_t callback, void* context = NULL) {
    _onRange = callback;
    _onRangeContext = context;
  }

  /**
  * Activate the lidar
  */
  void activate(void) {
    _active = true;
  }

  /**
  * Activate the lidar
  */
  void inactivate(void) {
    _active = false;
  }

  /**
  * set the measure interval
  */
  void interval(const unsigned long interval) {
    _timer.interval(interval);
  }

  /**
    Pooling the lidar
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
  * Returns the front measure
  */
  const VL53L0X_RangingMeasurementData_t& rearMeasure(void) const {
    return _rearMeasure;
  }

  /**
  * Returns the front distance (mm)
  */
  const uint16_t frontDistance(void) {
    return _frontMeasure.RangeMilliMeter;
  }

  /**
  * Returns the rear distance (mm)
  */
  const uint16_t rearDistance(void) {
    return _rearMeasure.RangeMilliMeter;
  }

  const bool active(void) const {return _active;}
};

#endif