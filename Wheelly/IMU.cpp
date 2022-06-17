#include "IMU.h"

#define G_VALUE             9.80665
#define ACC_SCALE           (4.0 * G_VALUE / 32768.0)
#define GYRO_SCALE           (1.0 / 32768.0)
#define WATCH_DOG_INTERVAL  100000ul

IMU::IMU(MPU6050& mpu) : _mpu {mpu} {
  _watchDogInterval = WATCH_DOG_INTERVAL;
  _devStatus = IMU_FAILURE_STATUS;
}

void IMU::begin() {
  _devStatus = IMU_FAILURE_STATUS;
  _mpu.initialize();

  int rc;
  if (!(rc = _mpu.testConnection())) {
    Serial.println(F("!! Connection IMU failed."));
    return;
  }
  _devStatus = _mpu.dmpInitialize();
  if (_devStatus) {
    Serial.print(F("!! DMP initialize failed: "));
    Serial.print(_devStatus);
    Serial.println();
  }
}

/*

*/
void IMU::enableDMP() {
  if (_devStatus == 0) {
    // turn on the DMP, now that it's ready
    _mpu.setDMPEnabled(true);

    // get expected DMP packet size for later comparison
    _packetSize = _mpu.dmpGetFIFOPacketSize();
  }
}

/*

*/
void IMU::reset() {
  if (_devStatus == 0) {
    _mpu.resetFIFO();
    _prevTime = micros();
    kickAt(_prevTime + _watchDogInterval);
  }
}

/*

*/
void IMU::polling(unsigned long clockMillis, unsigned long clockMicros) {
  if (_devStatus == 0) {
    if (_mpu.getFIFOCount() >= _packetSize) {
      if (_readFifo(clockMillis, clockMicros) && _onData != NULL) {
        _onData(_context);
      }
    } else if (clockMicros >= _watchDogTime && _onWatchDog != NULL) {
      kickAt(clockMicros + _watchDogInterval);
      _onWatchDog(_context);
    }
  }
}

boolean IMU::_readFifo(unsigned long clockMillis, unsigned long clockMicros) {
  // read a packet from FIFO
  uint8_t _fifoBuffer[64]; // FIFO storage buffer
  _mpu.getFIFOBytes(_fifoBuffer, _packetSize);
  _mpu.resetFIFO();
  kickAt(clockMicros + _watchDogInterval);

  _dt = (float)(clockMicros - _prevTime) * 1e-6;
  if (_dt > 0 && _dt < 1.0) {
    _lastTime = clockMillis;
    _prevTime = clockMicros;
    VectorFloat gravity;    // [x, y, z]            gravity vector
    Quaternion _q;           // [w, x, y, z]         quaternion container
    _mpu.dmpGetQuaternion(&_q, _fifoBuffer);
    _mpu.dmpGetGravity(&gravity, &_q);
    _mpu.dmpGetYawPitchRoll(_ypr, &_q, &gravity);
    return true;
  }
  return false;
}

/*

*/
void IMU::calibrate(int steps) {
  if (_devStatus == 0) {
    _mpu.setXAccelOffset(0);
    _mpu.setYAccelOffset(0);
    _mpu.setZAccelOffset(0);

    _mpu.setXGyroOffset(0);
    _mpu.setYGyroOffset(0);
    _mpu.setZGyroOffset(0);

    // Calibration Time: generate offsets and calibrate our MPU6050
    _mpu.CalibrateAccel(steps);
    _mpu.CalibrateGyro(steps);
#ifdef DEBUG
    _mpu.PrintActiveOffsets();
#endif
  }
}
