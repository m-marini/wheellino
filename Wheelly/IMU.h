#ifndef IMU_H
#define IMU_H

#include "MPU6050_6Axis_MotionApps20.h"

#define IMU_RUNNING_STATUS  0
#define IMU_FAILURE_STATUS  10

/*

*/
class IMU {
  public:
    IMU(MPU6050& mpu);
    void begin();
    void calibrate(int steps = 6);
    void enableDMP();
    void polling(unsigned long clockMillis = millis(), unsigned long clockMicros = micros());
    void reset();
    void kickAt(unsigned long time) {
      _watchDogTime = time;
    }
    void kick() {
      _watchDogTime = micros() + _watchDogInterval;
    }

    void onData(void (*callback)(void* context)) {
      _onData = callback;
    }
    void onWatchDog(void (*callback)(void* context)) {
      _onWatchDog = callback;
    }
    void context(void * ctx) {
      _context = ctx;
    }
    MPU6050& mpu() const {
      return _mpu;
    }

    const uint8_t status() const {
      return _devStatus;
    }
    const float* ypr() const {
      return _ypr;
    }
  private:
    MPU6050& _mpu;
    uint8_t _devStatus;
    uint16_t _packetSize;
    unsigned long _prevTime;
    unsigned long _lastTime;
    unsigned long _watchDogInterval;
    unsigned long _watchDogTime;
    void *_context;
    float _dt;
    float _ypr[3];

    void (*_onData)(void*);
    void (*_onWatchDog)(void*);
    boolean _readFifo(unsigned long, unsigned long);
};

#endif
