#include <Arduino.h>
#include <Wire.h>
#include "mpu6050mm.h"

//#define DEBUG
#ifdef DEBUG

//#define DUMP_FIFO
//#define DEBUG_MPU_WRITE
//#define DEBUG_MPU_READ

#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTF(x, y) Serial.print(x, y)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTLNF(x, y) Serial.println(x, y)

#else

#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(x, y)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNF(x, y)

#endif

#define REVALIDATION_SAMPLE_COUNT 1000
//#define FIFO_BLOCK_SIZE 6
#define STATUS_BLOCK_SIZE 14
#define RESET_TIMEOUT 1

static const float GYRO_SCALE[] = {
  250.0 / 32768,
  500.0 / 32768,
  1000.0 / 32768,
  2000.0 / 32768,
};

static const float ACC_SCALE[] = {
  2.0 / 32768,
  4.0 / 32768,
  8.0 / 32768,
  16.0 / 32768,
};

/*
   Dumps the data buffer
   @param data the data buffer
   @param len the number of bytes of data buffer
*/
static void dumpData(const uint8_t *data, const size_t len) {
#ifdef DEBUG
  for (size_t n = len; n-- > 0;) {
    Serial.print(" ");
    Serial.print(*data++, HEX);
  }
  Serial.println();
#endif
}

/*
   Returns the integer value of 2 bytes MSB
*/
static const int16_t toInt(const uint8_t *data) {
  int16_t value = ((uint16_t)(data[0]) << 8) | ((uint16_t)(data[1]));

  DEBUG_PRINT("// toInt 0x");
  DEBUG_PRINTF(data[0], HEX);
  DEBUG_PRINT(" 0x");
  DEBUG_PRINTF(data[1], HEX);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINT(value);
  DEBUG_PRINT(" 0x");
  DEBUG_PRINTF(value, HEX);
  DEBUG_PRINTLN();

  return value;
}

/*
   Returns Vector3 from mpu buffer.
   Note the x, y axis inversion due to mpu mounting on robot
   @param data the mpu buffer
   @param scale the scale
*/
static Vector3 fromBytes(const uint8_t *data, const float scale) {
  return Vector3(((float)toInt(data + 2)) * (-scale),
                 ((float)toInt(data)) * scale,
                 ((float)toInt(data + 4)) * scale);
}

/*
   Creates the MPU controller
   @param address the I2C address
*/
MPU6050Class::MPU6050Class(const uint8_t address) : _address(address),
  _readPtr(_buffer),
  _writePtr(_buffer) {
}

/*
   Initializes the MPU controller
   @param afsSel the AFS selection
   @param fsSel the FS selection
   @param samplingDic the sampling divider
   @param dlpfCfg the slpg configuration
   @param clkSel the clock selection
*/
const uint8_t MPU6050Class::begin(
  const uint8_t afsSel,
  const uint8_t fsSel,
  const uint8_t samplingDiv,
  const uint8_t dlpfCfg,
  const uint8_t clkSel) {

  _gyroScale = GYRO_SCALE[fsSel];
  _accScale = ACC_SCALE[afsSel];
  float goRate = dlpfCfg == DLPF_0 ? 8000.0 : 1000.0;
  _sampleRate = goRate / (1 + samplingDiv);

  if (mpuRegWrite(SIGNAL_PATH_RESET_REG, 0x07) != 0) {
    return _rc;
  }
  if (mpuRegWrite(SIGNAL_PATH_RESET_REG, 0) != 0) {
    return _rc;
  }
  if (mpuRegWrite(PWR_MGMT_1_REG, clkSel) != 0) {
    return _rc;
  }
  if (mpuRegWrite(SMPRT_DIV_REG, samplingDiv) != 0) {
    return _rc;
  }
  if (mpuRegWrite(CONFIG_REG, dlpfCfg) != 0) {
    return _rc;
  }
  if (mpuRegWrite(GYRO_CONFIG_REG, fsSel << 3) != 0) {
    return _rc;
  }
  if (mpuRegWrite(ACCEL_CONFIG_REG, afsSel << 3) != 0) {
    return _rc;
  }
  if (mpuRegWrite(INT_PIN_CFG_REG, INT_LEVEL | LATCH_INT_EN | INT_RD_CLEAR) != 0) {
    return _rc;
  }
  if (mpuRegWrite(FIFO_EN_REG, XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN) != 0) {
    return _rc;
  }
  if (mpuRegWrite(INT_ENABLE_REG, FIFO_OFLOW_EN | I2C_MST_EN | DATA_RDY_EN) != 0) {
    return _rc;
  }
  if (mpuRegWrite(USER_CTRL_REG, FIFO_EN_UC) != 0) {
    return _rc;
  }
  if (flushFifo() != 0) {
    return _rc;
  }
  return _rc;
}

/*
   Flushes the fifo
*/
const uint8_t MPU6050Class::flushFifo() {
  _available = 0;
  _readPtr = _writePtr = _buffer;
  if (mpuRegWrite(USER_CTRL_REG, FIFO_RESET_UC) != 0) {
    return _rc;
  }
  if (mpuRegWrite(USER_CTRL_REG, FIFO_EN_UC) != 0) {
    return _rc;
  }
  if (readIntStatus()) {
    return _rc;
  }
  return _rc;
}

/*
   Calibrates the MPU
*/
const uint8_t MPU6050Class::calibrate(unsigned int minNoSamples, unsigned long warmup) {
  _gyroOffset = Vector3();
  Serial.println("// MPU Calibrating ...");

  unsigned long timeout = millis() + warmup;
  _rc = 0;
  DEBUG_PRINTLN("// MPU Warming up ...");
  // Waits for warmup duration just polling the MPU
  while (millis() <= timeout) {
    getGyro(_gyroOffset);
    if (_rc) {
      return _rc;
    }
  }

  _calibrating = true;
  int numSamples = 0;
  int numAccSamples = 0;
  Vector3 acc;
  DEBUG_PRINTLN("// MPU Start calibration ...");
  while (_rc == 0 && !(numAccSamples >= minNoSamples)) {
    // Processes gyro samples
    Vector3 gyro;
    while (getGyro(gyro)) {
      numSamples++;
      _gyroOffset += gyro;
    }
    if (_rc) {
      break;
    }

    // Polls MPU for acceleration values
    if (readIntStatus() == 0 && dataReady()) {
      Vector3 accel;
      if (readAcc(accel) == 0) {
        acc += accel;
        numAccSamples++;
      }
    }
  }
  _calibrating = false;
  if (_rc) {
    return _rc;
  }

  _gyroOffset /= (float)numSamples;
  acc /= (float) numAccSamples;
  Vector3 accVersus = acc.unit();
  float angle = acosf(accVersus * Vector3(0, 0, 1));
  Vector3 axis = Vector3(0, 0, 1).cross(accVersus);
  Quaternion q = Quaternion::rot(angle, axis);
  _quat = q;
  _gravity = acc;

  Serial.print("// MPU Calibration completed:");
  Serial.println();
  Serial.print("//   ");
  Serial.print(numSamples);
  Serial.print(" gyroscope samples");
  Serial.println();
  Serial.print("//   offset: ");
  Serial.print(_gyroOffset.toString());
  Serial.println();

  Serial.print("//   ");
  Serial.print(numAccSamples);
  Serial.print(" accelerometer samples");
  Serial.println();
  Serial.print("//   rotation angle (DEG): ");
  Serial.print(angle * 180 / PI);
  Serial.println();
  Serial.print("//   vector: ");
  Serial.print(_gravity.toString());
  Serial.println();

  return _rc;
}

/*
   Polls the MPU
*/
void MPU6050Class::polling(unsigned long clockTime) {

#ifdef DEBUG
  if (readIntStatus() == 0 && fifoOverflow()) {
    DEBUG_PRINTLN("!! Fifo Overflow");
  }
#endif

  Vector3 gyro;
  boolean dataReady = false;
  while (getGyro(gyro)) {
    applyData(gyro);
    dataReady = true;
  }
  if (dataReady && _onData != NULL) {
    _onData(_context);
  }
}

/*
   Starts the monitoring
*/
void MPU6050Class::startMonitoring() {
  _monitoring = true;
  _normalizationCountdown = REVALIDATION_SAMPLE_COUNT;
}

/*
   Applies the gyroscope vector read from MPU
  @param gyro the gyroscope vector
*/
void MPU6050Class::applyData(Vector3 & gyro) {
  if (_normalizationCountdown == 0) {
    _quat = _quat.unit();
    _normalizationCountdown = REVALIDATION_SAMPLE_COUNT;
  }
  _normalizationCountdown--;
  Vector3 angle = (gyro - _gyroOffset) / sampleRate() * (PI / 180);
  _quat *= Quaternion::rot(angle);
}

/*
   Resets the MPU
*/
const uint8_t MPU6050Class::reset() {
  return mpuRegWrite(PWR_MGMT_1_REG, DEVICE_RESET);
}

/*
   Reads the power management
*/
const uint8_t MPU6050Class::readPowerManagement() {
  return mpuRegRead(PWR_MGMT_1_REG, &_pwr_mgmt_1, 1);
}

/*
   Reads the in status
*/
const uint8_t MPU6050Class::readIntStatus() {
  return mpuRegRead(INT_STATUS_REG, &_intStatus, 1);
}

/*
   Reads and returns the fifo count
*/
const uint16_t MPU6050Class::readFifoCount() {
  uint8_t bfr[2];
  if (mpuRegRead(FIFO_COUNT_REG, bfr, sizeof(bfr)) != 0) {
    return 0;
  }
  return ((uint16_t) (bfr[0]) << 8) | bfr[1];
}

/*
   Reads the fifo.
   Returns the errors from read
   @param bfr the buffer
   @param len the length of buffer
*/
const uint8_t MPU6050Class::readFifo(uint8_t *bfr, const uint16_t len) {
  _rc = 0;
  for (uint16_t i = 0; i < len; i++) {
    if (mpuRegRead(FIFO_DATA_REG, bfr++, 1) != 0) {
      return _rc;
    }
  }
  return _rc;
}

/*
   Writes a value in the registry
   Returns the error
   @param reg the registry
   @param value the writting value
*/
const uint8_t MPU6050Class::mpuRegWrite(const uint8_t reg, const uint8_t value) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(value);
  _rc = Wire.endTransmission();
  if (_rc != 0) {
    char msg[256];
    sprintf(msg, "!! Error writing register 0x%02hx @0x%02hx: rc=%hu",
            (const unsigned short) reg,
            (const unsigned short) _address,
            (const unsigned short) _rc);
    DEBUG_PRINTLN(msg);
    throwError(msg);
    return _rc;
  }
  return _rc;
}

/*
   Reads MPU registry
   @param reg the registry
   @param bfr the the data buffer
   @param len the number of bytes to read
*/
const uint8_t MPU6050Class::mpuRegRead(const uint8_t reg, uint8_t *bfr, const uint8_t len) {
  char msg[256];
#ifdef DEBUG_MPU_READ
  DEBUG_PRINT("// MPU read 0x");
  DEBUG_PRINTF(reg, HEX);
  DEBUG_PRINTLN();
#endif
  Wire.beginTransmission(_address);
  Wire.write(reg);
  _rc = Wire.endTransmission(false);
  if (_rc != 0) {
    sprintf(msg, "!! Error writing read registry 0x%02hx @0x%02hx: rc=%hu",
            (const unsigned short)reg,
            (const unsigned short)_address,
            (const unsigned short)_rc);
    DEBUG_PRINTLN(msg);
    throwError(msg);
    return _rc;
  }
  uint8_t n = Wire.requestFrom(_address, len);
  if (n != len) {
    sprintf(msg, "!! Error reading: len=%hu, required= %hu",
            (const unsigned short)n,
            (const unsigned short)len);
    DEBUG_PRINTLN(msg);
    _rc = READ_LEN_ERROR;
    throwError(msg);
    return _rc;
  }
  uint8_t* ptr = bfr;
  for (uint8_t i = 0; i < n; ++i) {
    *ptr++ = Wire.read();
  }
#ifdef DEBUG_MPU_READ
  ptr = bfr;
  for (uint8_t i = 0; i < n; ++i) {
    DEBUG_PRINT(" ");
    DEBUG_PRINTF(*ptr++, HEX);
  }
  DEBUG_PRINTLN();
#endif
  return _rc;
}

/*
   Writes consecutive data to MPU (registry, value)
   @param data the data buffer
   @param len the nunmber of byte to write
*/
const uint8_t MPU6050Class::mpuRegWrite(const uint8_t *data, const size_t len) {
  _rc = 0;
  for (size_t i = len; i >= 2; i -= 2) {
    if (mpuRegWrite(*data++, *data++) != 0) {
      return _rc;
    }
  }
  return _rc;
}

/*
   Reads the acceleration data from MPU
   @param acc the result acceleration vector
*/
const uint8_t MPU6050Class::readAcc(Vector3 & acc) {
  uint8_t block[6];
  if (mpuRegRead(ACCEL_REG, block, sizeof(block)) == 0) {
    acc = fromBytes(block, _accScale);
    DEBUG_PRINT("// Read acceleration ");
    DEBUG_PRINT(acc.toString());
    DEBUG_PRINTLN();
  }
  return _rc;
}

/**
   Returns true if data available
   @param gyro the output data
*/
const boolean MPU6050Class::getGyro(Vector3& gyro) {
  if (_available < MPU_BLOCK_SIZE) {
    // If no data available fill buffer from fifo
    fillBuffer();
  }
  if (_available < MPU_BLOCK_SIZE) {
    // No data available
    return false;
  }
  // Reads data
  gyro = fromBytes(_readPtr, _gyroScale);

  // Decrements the available data
  _available -= MPU_BLOCK_SIZE;

  // Updates read pointer
  _readPtr += MPU_BLOCK_SIZE;
  if (_readPtr - _buffer >= MPU_BUFFER_SIZE) {
    // Rolls the read pointer
    _readPtr = _buffer;
  }
  return true;
}

/**
   Fills the buffer from fifo
*/
void MPU6050Class::fillBuffer(void) {
  uint16_t len = readFifoCount();
  // Repeats read util no more data or full buffer size
  while (len > 0 && _available < MPU_BUFFER_SIZE) {
    // Computes write buffer block size
    size_t writeSize = _writePtr >= _readPtr
                       ? _buffer - _writePtr + MPU_BUFFER_SIZE
                       : _readPtr - _writePtr;
    if (writeSize > len) {
      writeSize = len;
    }

    // Reads fifo block
    if (readFifo(_writePtr, writeSize)) {
      // Errors reading fifo
      break;
    }

    // Increments the available data
    _available += writeSize;
    len -= writeSize;

    // Moves the write pointer
    _writePtr += writeSize;
    if (_writePtr - _buffer >= MPU_BUFFER_SIZE) {
      // Rolls the pointer
      _writePtr = _buffer;
    }
  }
}
