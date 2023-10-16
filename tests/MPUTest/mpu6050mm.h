#ifndef mpu6050mm_h
#define mpu6050mm_h

#include "num.h"

/*
   MPU errors
*/
#define READ_LEN_ERROR 6

/*
   MPU address
*/
#define MPU_ADDRESS 0x68

/*
   MPU registers
*/
#define SMPRT_DIV_REG         0x19
#define CONFIG_REG            0x1a
#define GYRO_CONFIG_REG       0x1b
#define ACCEL_CONFIG_REG      0x1c
#define FIFO_EN_REG           0x23
#define INT_PIN_CFG_REG       0x37
#define INT_ENABLE_REG        0x38
#define INT_STATUS_REG        0x3a
#define ACCEL_REG             0x3b
#define SIGNAL_PATH_RESET_REG 0x68
#define USER_CTRL_REG         0x6a
#define PWR_MGMT_1_REG        0x6b
#define FIFO_COUNT_REG        0x72
#define FIFO_DATA_REG         0x74
#define WHO_AM_I_REG          0x75

/*
   MPU FIFO enables
*/
#define NONE_FIFO_EN  0
#define SLV0_FIFO_EN  0x01
#define SLV1_FIFO_EN  0x02
#define SLV2_FIFO_EN  0x04
#define ACCEL_FIFO_EN 0x08
#define ZG_FIFO_EN    0x10
#define YG_FIFO_EN    0x20
#define XG_FIFO_EN    0x40
#define TEMP_FIFO_EN  0x80

/*
   MPU user controls
*/
#define NONE_UC           0
#define SIG_COND_RESET_UC 0x01
#define I2C_MST_RESET_UC  0x02
#define FIFO_RESET_UC     0x04
#define I2C_IF_DIS_UC     0x10
#define I2C_MST_EN_UC     0x20
#define FIFO_EN_UC        0x40

/*
   MPU interupt pin configurations
*/
#define NONE_INT_PIN_CFG  0
#define I2C_BYPASS_CFG    0x02
#define FSYNC_INT_EN      0x04
#define FSYNC_INT_LEVEL   0x08
#define INT_RD_CLEAR      0x10
#define LATCH_INT_EN      0x20
#define INT_OPEN          0x40
#define INT_LEVEL         0x80

/*
   MPU FS (gyroscope full scale) selections
*/
#define FS_250  0
#define FS_500  1
#define FS_1000 2
#define FS_2000 3

/*
   MPU AFS (accelerator full scale) selections
*/
#define AFS_2G  0
#define AFS_4G  1
#define AFS_8G  2
#define AFS_16G 3

/*
   MPU DLPF (digital low pass filter) configurations
*/
#define DLPF_0 0
#define DLPF_1 1
#define DLPF_2 2
#define DLPF_3 3
#define DLPF_4 4
#define DLPF_5 5
#define DLPF_6 6

/*
   MPU clock selections
*/
#define CLKSEL_INTERNAL 0
#define CLKSEL_X_AXIS   1
#define CLKSEL_Y_AXIS   2
#define CLKSEL_Z_AXIS   3
#define CLKSEL_32768HZ  4
#define CLKSEL_19200KHZ 5
#define CLKSEL_STOP     7

/*
   MPU interupt enablers
*/
#define NONE_INT_EN   0
#define FIFO_OFLOW_EN 0x10
#define I2C_MST_EN    0x08
#define DATA_RDY_EN   0x01

/*
   MPU status
*/
#define FIFO_OFLOW_INT  0x10
#define I2C_MST_INT     0x08
#define DATA_RDY_INT    0x01

/*
   MPU power managements
*/
#define DEVICE_RESET  0x80

/*
   MPU buffer constants
*/
#define MPU_FIFO_SIZE 1024
#define MPU_BLOCK_SIZE 6
#define MPU_BUFFER_SIZE (((MPU_FIFO_SIZE + MPU_BLOCK_SIZE - 1) / MPU_BLOCK_SIZE) * MPU_BLOCK_SIZE)

/*
   MPU (Motion Processor Unit)
*/
class MPU6050Class {
  public:
    /*
       Creates the MPU controller
       @param address the I2C address
    */
    MPU6050Class(const uint8_t address = MPU_ADDRESS);

    /**
       Initializes the MPU
    */
    const uint8_t begin(
      const uint8_t fs_sel = FS_2000,
      const uint8_t afs_sel = AFS_16G,
      const uint8_t sampling_div = 7,
      const uint8_t dlpf_cfg = DLPF_1,
      const uint8_t clksel = CLKSEL_Z_AXIS);

    /**
       Polls the MPU
    */
    void polling(const unsigned long clockTime = millis());

    /**
       Calibrates the MPU
    */
    const uint8_t calibrate(const unsigned int minNoSamples = 400, unsigned long warmup = 5000);

    /**
       Sets on data callback
    */
    void onData(void (*callback)(void* context), void *context = NULL) {
      _onData = callback;
      _context = context;
    }

    /*
       Sets error callback
    */
    void onError(void(*callback)(void*context, const char* error), void *context = NULL) {
      _onError = callback;
      _errorContext = context;
    }

    /**
       Returns the last return code
    */
    const uint8_t rc(void) const {
      return _rc;
    }

    /**
       Returns the acceleration scale
    */
    const float accScale(void) const {
      return _accScale;
    }

    /**
       Returns the gyroscope scale
    */
    const float gyroScale(void) const {
      return _gyroScale;
    }

    /**
       Returns the gyroscope offset
    */
    const Vector3& gyroOffset(void) const {
      return _gyroOffset;
    }

    /**
       Returns the int status
    */
    const uint8_t intStatus(void) const {
      return _intStatus;
    }

    /**
       Returns true if fifo overflow
    */
    const bool fifoOverflow(void) const {
      return (_intStatus & FIFO_OFLOW_INT) != 0;
    }

    /**
       Returns true if data is ready
    */
    const bool dataReady(void) const {
      return (_intStatus & DATA_RDY_INT) != 0;
    }

    /**
       Returns the sample rate
    */
    const float sampleRate(void) const {
      return _sampleRate;
    }

    /**
       Returns the quaternion
    */
    const Quaternion& quat(void) const {
      return _quat;
    }

    /**
       Returns the yaw value (RAD)
    */
    const float yaw(void) const {
      return _quat.yaw();
    }

    /**
       Returns the pitch value (RAD)
    */
    const float pitch(void) const {
      return _quat.pitch();
    }

    /**
       Returns the roll value (RAD)
    */
    const float roll(void) const {
      return _quat.roll();
    }

    /**
       Returns the yaw, pitch, roll vector (RAD)
    */
    const Vector3 ypr(void) const {
      return _quat.ypr();
    }

    /**
       Returns the gravity vector
    */
    const Vector3& gravity(void) const {
      return _gravity;
    }

    /**
       Returns true if devices is resetting
    */
    const boolean isDeviceResetting(void) const {
      return (_pwr_mgmt_1 & DEVICE_RESET) != 0;
    }

    /*
       Reads the acceleration
       @return the return code
    */
    const uint8_t readAcc(Vector3& acc);

    /*
       Reads the int status
       @return the return code
    */
    const uint8_t readIntStatus(void);

    /*
       Reads the power management
       @return the return code
    */
    const uint8_t readPowerManagement(void);

    /*
       Resets the MPU
       @return the return code
    */
    const uint8_t reset(void);

    /*
       Writes a value in a registry
       @param reg the registry
       @param value the value
       @return the return code
    */
    const uint8_t mpuRegWrite(const uint8_t reg, const uint8_t value);

    /*
       Writes the buffer of values
       @param data the data buffer
       @param len the number of bytes
       @return the return code
    */
    const uint8_t mpuRegWrite(const uint8_t *data, size_t len);

    /*
       Reads from the registry
       @param reg the registry
       @param bfr the result buffer
       @param len the number of bytes to read
       @return the return code
    */
    const uint8_t mpuRegRead(const uint8_t reg, uint8_t *bfr, const uint8_t len);

    /*
       Flushes fifo
       @return the return code
    */
    const uint8_t flushFifo(void);

    /**
       Reads the fifo buffer
       @return the return code
    */
    const uint8_t readFifo(uint8_t *bfr, const uint16_t len);

    /**
       Returns the number of bytes in fifo
    */
    const uint16_t readFifoCount(void);

    /**
       Returns true if data available
       @param gyro the output data
    */
    const boolean getGyro(Vector3& gyro);

  private:
    const uint8_t _address;
    boolean _monitoring;
    boolean _calibrating;
    float _sampleRate;
    float _accScale;
    float _gyroScale;
    Vector3 _gyroOffset;
    Vector3 _gravity;
    Quaternion _quat;
    uint8_t _rc;
    uint8_t _intStatus;
    uint8_t _pwr_mgmt_1;
    unsigned int _normalizationCountdown;
    void *_context;
    void (*_onData)(void*);
    void *_errorContext;
    void (*_onError)(void*, const char*);
    uint8_t* _readPtr;
    uint8_t* _writePtr;
    size_t _available;
    uint8_t _buffer[MPU_BUFFER_SIZE];

    void applyData(Vector3& gyro);
    void startMonitoring(void);
    void throwError(const char *message) {
      if (_onError) {
        _onError(_errorContext, message);
      }
    }
    void fillBuffer(void);
};

#endif
