#ifndef mpu6050mm_h
#define mpu6050mm_h
#include <Arduino.h>
#include <Wire.h>
#include "num.h"

typedef enum {
  READ_LEN_ERROR = 6
} mpu_errors_t;

typedef enum {
  MPU_ADDRESS = 0x68,
} mpu_address_t;

typedef enum {
  SMPRT_DIV_REG = (0x19),
  CONFIG_REG = (0x1a),
  GYRO_CONFIG_REG = (0x1b),
  ACCEL_CONFIG_REG = (0x1c),
  FIFO_EN_REG = (0x23),
  INT_PIN_CFG_REG = (0x37),
  INT_ENABLE_REG = (0x38),
  INT_STATUS_REG = (0x3a),
  ACCEL_REG = (0x3b),
  SIGNAL_PATH_RESET_REG = (0x68),
  USER_CTRL_REG = (0x6a),
  PWR_MGMT_1_REG = (0x6b),
  FIFO_COUNT_REG = (0x72),
  FIFO_DATA_REG = (0x74),
} mpu_reg_t;

typedef enum {
  NONE_FIFO_EN = (0),
  SLV0_FIFO_EN = (0x01),
  SLV1_FIFO_EN = (0x02),
  SLV2_FIFO_EN = (0x04),
  ACCEL_FIFO_EN = (0x08),
  ZG_FIFO_EN = (0x10),
  YG_FIFO_EN = (0x20),
  XG_FIFO_EN = (0x40),
  TEMP_FIFO_EN = (0x80),
} mpu_fifo_en_t;

typedef enum {
  NONE_UC = (0),
  SIG_COND_RESET_UC = (0x01),
  I2C_MST_RESET_UC = (0x02),
  FIFO_RESET_UC = (0x04),
  I2C_IF_DIS_UC = (0x10),
  I2C_MST_EN_UC = (0x20),
  FIFO_EN_UC = (0x40),
} mpu_user_ctrl_t;

typedef enum {
  NONE_INT_PIN_CFG = (0),
  I2C_BYPASS_CFG = (0x02),
  FSYNC_INT_EN = (0x04),
  FSYNC_INT_LEVEL = (0x08),
  INT_RD_CLEAR = (0x10),
  LATCH_INT_EN = (0x20),
  INT_OPEN = (0x40),
  INT_LEVEL = (0x80)
} mpu_int_pin_cfg_t;

typedef enum {
  FS_250 = (0),
  FS_500 = (1),
  FS_1000 = (2),
  FS_2000 = (3)
} mpu_fs_sel_t;

typedef enum {
  AFS_2G = (0),
  AFS_4G = (1),
  AFS_8G = (2),
  AFS_16G = (3)
} mpu_afs_sel_t;

typedef enum {
  DLPF_0 = (0),
  DLPF_1 = (1),
  DLPF_2 = (2),
  DLPF_3 = (3),
  DLPF_4 = (4),
  DLPF_5 = (5),
  DLPF_6 = (6),
} mpu_dlpf_cfg_t;

typedef enum {
  CLKSEL_INTERNAL = (0),
  CLKSEL_X_AXIS = (1),
  CLKSEL_Y_AXIS = (2),
  CLKSEL_Z_AXIS = (3),
  CLKSEL_32768HZ = (4),
  CLKSEL_19200KHZ = (5),
  CLKSEL_STOP = (7),
} mpu_clksel_t;

typedef enum {
  NONE_INT_EN = (0),
  FIFO_OFLOW_EN = (0x10),
  I2C_MST_EN = (0x08),
  DATA_RDY_EN = (0x01)
} mpu_int_enable_t;

typedef enum {
  FIFO_OFLOW_INT = (0x10),
  I2C_MST_INT = (0x08),
  DATA_RDY_INT = (0x01)
} mpu_int_status_t;

class MPU6050 {
  public:
    MPU6050(mpu_address_t address = MPU_ADDRESS);
    const mpu_errors_t begin(
      mpu_fs_sel_t fs_sel = FS_2000,
      mpu_afs_sel_t afs_sel = AFS_16G,
      uint8_t sampling_div = 7,
      mpu_dlpf_cfg_t dlpf_cfg = DLPF_1,
      mpu_clksel_t clksel = CLKSEL_Z_AXIS);
    void polling(unsigned long clockTime = millis());
    const mpu_errors_t calibrate(unsigned int minNoSamples = 100, unsigned long warmup = 500);
    void onData(void (*callback)(void* context), void *context = NULL) {
      _onData = callback;
      _context = context;
    }

    const mpu_errors_t rc() const {
      return _rc;
    }
    const float accScale() const {
      return _accScale;
    }
    const float gyroScale() const {
      return _gyroScale;
    }
    const Vector3& gyroOffset() const {
      return _gyroOffset;
    }
    const bool fifoOverflow() const {
      return (_intStatus & FIFO_OFLOW_INT) != 0;
    }
    const bool dataReady() const {
      return (_intStatus & DATA_RDY_INT) != 0;
    }
    const float sampleRate() const {
      return _sampleRate;
    }
    const Quaternion& quat() const {
      return _quat;
    }
    const unsigned long numSamples() const {
      return _numSamples;
    }
    const float yaw() const {
      return _quat.yaw();
    }
    const float pitch() const {
      return _quat.pitch();
    }
    const float roll() const {
      return _quat.roll();
    }
    const Vector3 ypr() const {
      return _quat.ypr();
    }
    const Vector3& gravity() const {
      return _gravity;
    }
    const mpu_errors_t readAcc(Vector3& acc);

  private:
    const mpu_address_t _address;
    boolean _monitoring;
    boolean _calibrating;
    float _sampleRate;
    float _accScale;
    float _gyroScale;
    unsigned long _numSamples;
    Vector3 _gyroOffset;
    Vector3 _gravity;
    Quaternion _quat;
    mpu_errors_t _rc;
    uint8_t _intStatus;
    unsigned int _normalizationCountdown;
    void *_context;
    void (*_onData)(void*);

    const mpu_errors_t mpuRegWrite(mpu_reg_t reg, uint8_t value);
    const mpu_errors_t mpuRegWrite(uint8_t *data, size_t len);
    const mpu_errors_t mpuRegRead(mpu_reg_t reg, uint8_t *bfr, uint8_t len);

    const mpu_errors_t readIntStatus();
    const mpu_errors_t readFifo(uint8_t *bfr, uint16_t len);
    const mpu_errors_t readFifoBlock(Vector3& gyro);
    const uint16_t readFifoCount();
    void parseFifoBlock(Vector3& acc, Vector3& gyro, const uint8_t *block) const;
    const mpu_errors_t flushFifo();
    void applyData(Vector3& gyro);
    void startMonitoring();
};

#endif
