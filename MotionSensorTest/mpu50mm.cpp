#include "mpu6050mm.h"

//#define DEBUG

#ifdef DEBUG

//#define DUMP_FIFO
//#define DEBUG_MPU_WRITE

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
#define FIFO_BLOCK_SIZE 6
#define STATUS_BLOCK_SIZE 14

#define toInt(data) (((data)[0] << 8) | (data)[1])

const float GYRO_SCALE[] PROGMEM = {
  250.0 / 32768,
  500.0 / 32768,
  1000.0 / 32768,
  2000.0 / 32768,
};

const float ACC_SCALE[] PROGMEM = {
  2.0 / 32768,
  4.0 / 32768,
  8.0 / 32768,
  16.0 / 32768,
};

void dumpData(uint8_t *data, size_t len) {
#ifdef DEBUG
  while (len-- > 0) {
    Serial.print(F(" "));
    Serial.print(*data++, HEX);
  }
  Serial.println();
#endif
}

/*
   Returns Vector3 from mpu buffer
   Note the x, y axis inversion due to mpu mounting on robot
*/
Vector3 fromBytes(uint8_t *data, const float scale) {
  return Vector3(((float)toInt(data + 2)) * (-scale),
                 ((float)toInt(data)) * scale,
                 ((float)toInt(data + 4)) * scale);
}

MPU6050::MPU6050(mpu_address_t address) : _address(address) {
}

const mpu_errors_t MPU6050::begin(
  mpu_fs_sel_t afsSel,
  mpu_afs_sel_t fsSel,
  uint8_t samplingDiv,
  mpu_dlpf_cfg_t dlpfCfg,
  mpu_clksel_t clkSel) {
  Wire.begin();
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
  if (mpuRegWrite(INT_ENABLE_REG, FIFO_OFLOW_EN | I2C_MST_EN | DATA_RDY_EN) != 0) {
    return _rc;
  }
  if (mpuRegWrite(USER_CTRL_REG, FIFO_EN_UC) != 0) {
    return _rc;
  }
  if (mpuRegWrite(SIGNAL_PATH_RESET_REG, 0x07) != 0) {
    return _rc;
  }
  if (mpuRegWrite(PWR_MGMT_1_REG, clkSel) != 0) {
    return _rc;
  }
  if (mpuRegWrite(FIFO_EN_REG, XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN) != 0) {
    return _rc;
  }
  if (flushFifo() != 0) {
    return _rc;
  }

  memcpy_P(&_gyroScale, GYRO_SCALE + fsSel, sizeof(float));
  memcpy_P(&_accScale, ACC_SCALE + afsSel, sizeof(float));
  float goRate = dlpfCfg == DLPF_0 ? 8000.0 : 1000.0;
  _sampleRate = goRate / (1 + samplingDiv);
  _numSamples = 0;
  return _rc;
}

const mpu_errors_t MPU6050::flushFifo() {
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

const mpu_errors_t MPU6050::calibrate(unsigned int minNoSamples, unsigned long warmup) {
  _gyroOffset = Vector3();
  unsigned long timeout = millis() + warmup;
  _rc = 0;
  DEBUG_PRINTLN(F("Warming up ..."));
  while (_rc == 0 && millis() <= timeout) {
    polling();
  }

  DEBUG_PRINTLN(F("Calibrating ..."));
  _numSamples = 0;
  _calibrating = true;
  int numAccSamples = 0;
  Vector3 acc;
  while (_rc == 0 && !(_numSamples >= minNoSamples && numAccSamples >= minNoSamples)) {
    polling();
    if (readIntStatus() == 0 && dataReady()) {
      Vector3 accel;
      if (readAcc(accel) == 0) {
        acc += accel;
        numAccSamples++;
      }
    }
  }
  _calibrating = false;
  if (_rc == 0) {
    _gyroOffset /= (float)_numSamples;
    acc /= (float) numAccSamples;
    Vector3 accVersus = acc.unit();
    float angle = acosf(accVersus * Vector3(0, 0, 1));
    Vector3 axis = Vector3(0, 0, 1).cross(accVersus);
    Quaternion q = Quaternion::rot(angle, axis);
    _quat = q;
    _gravity = acc;

#ifdef DEBUG
    DEBUG_PRINT(F("Calibrated gyro offset: "));
    _gyroOffset.print();
    DEBUG_PRINTLN();

    DEBUG_PRINT(F("gravity: "));
    _gravity.print();
    DEBUG_PRINTLN();
#endif

    startMonitoring();
  }
  return _rc;
}

void MPU6050::polling(unsigned long clockTime) {

#ifdef DEBUG
  if (readIntStatus() == 0 && fifoOverflow()) {
    DEBUG_PRINTLN(F("Fifo Overflow"));
  }
#endif

  Vector3 gyro;
  uint16_t len = readFifoCount();
  if (len >= FIFO_BLOCK_SIZE) {
    boolean dataReady = false;
    while (len >= FIFO_BLOCK_SIZE) {
      if (readFifoBlock(gyro) != 0) {
        return;
      }
      applyData(gyro);
      dataReady = true;
      len -= FIFO_BLOCK_SIZE;
    }
    if (dataReady && _onData != NULL) {
      _onData(_context);
    }
  }
}

void MPU6050::startMonitoring() {
  _monitoring = true;
  _normalizationCountdown = REVALIDATION_SAMPLE_COUNT;
}

void MPU6050::applyData(Vector3& gyro) {
  _numSamples++;
  if (_monitoring) {
    if (_normalizationCountdown == 0) {
      _quat = _quat.unit();
      _normalizationCountdown = REVALIDATION_SAMPLE_COUNT;
    }
    _normalizationCountdown--;
    Vector3 angle = (gyro - _gyroOffset) / sampleRate() * (PI / 180);
    _quat *= Quaternion::rot(angle);
  }
  if (_calibrating) {
    _gyroOffset += gyro;
  }
}

const mpu_errors_t MPU6050::readIntStatus() {
  return mpuRegRead(INT_STATUS_REG, &_intStatus, 1);
}

const uint16_t MPU6050::readFifoCount() {
  uint8_t bfr[2];
  if (mpuRegRead(FIFO_COUNT_REG, bfr, sizeof(bfr)) != 0) {
    return _rc;
  }
  return ((uint16_t) (bfr[0]) << 8) | bfr[1];
}

const mpu_errors_t MPU6050::readFifo(uint8_t *bfr, uint16_t len) {
  _rc = 0;
  for (uint16_t i = 0; i < len; i++) {
    if (mpuRegRead(FIFO_DATA_REG, bfr++, 1) != 0) {
      return _rc;
    }
  }
  return _rc;
}

const mpu_errors_t MPU6050::mpuRegWrite(mpu_reg_t reg, uint8_t value) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(value);
  _rc = Wire.endTransmission();
  if (_rc != 0) {
    DEBUG_PRINT(F("Error writing register 0x0"));
    DEBUG_PRINTF(reg, HEX);
    DEBUG_PRINT(F(" @0x0"));
    DEBUG_PRINTF(_address, HEX);
    DEBUG_PRINT(F(": rc="));
    DEBUG_PRINT(_rc);
    DEBUG_PRINTLN();
    return _rc;
  }

#ifdef DEBUG_MPU_WRITE
  DEBUG_PRINT(F("Write 0x0"));
  DEBUG_PRINTF(reg, HEX);
  DEBUG_PRINT(F(" @0x0"));
  DEBUG_PRINTF(_address, HEX);
  DEBUG_PRINT(F(" = 0x0"));
  DEBUG_PRINTF(value, HEX);
  DEBUG_PRINT(F(" 0b0"));
  DEBUG_PRINTF(value, BIN);
  DEBUG_PRINTLN();
#endif
  return _rc;
}

const mpu_errors_t MPU6050::mpuRegRead(const mpu_reg_t reg, uint8_t *bfr, const uint8_t len) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  _rc = Wire.endTransmission(false);
  if (_rc != 0) {
    DEBUG_PRINT(F("Error writing read registry 0x0"));
    DEBUG_PRINTF(reg, HEX);
    DEBUG_PRINT(F(" @0x0"));
    DEBUG_PRINTF(_address, HEX);
    DEBUG_PRINT(F(": rc="));
    DEBUG_PRINT(_rc);
    DEBUG_PRINTLN();
    return _rc;
  }
  uint8_t n = Wire.requestFrom(_address, len);
  if (n != len) {
    DEBUG_PRINT(F("Error reading: len="));
    DEBUG_PRINT(n);
    DEBUG_PRINT(F(", required="));
    DEBUG_PRINT(len);
    DEBUG_PRINTLN();
    _rc = READ_LEN_ERROR;
    return _rc;
  }
  for (uint8_t i = 0; i < n; ++i) {
    *bfr++ = Wire.read();
  }
  return _rc;
}

const mpu_errors_t MPU6050::mpuRegWrite(uint8_t *data, size_t len) {
  _rc = 0;
  while (len >= 2) {
    mpuRegWrite(data[0], data[1]);
    if (_rc != 0) {
      return _rc;
    }
    len -= 2;
    data += 2;
  }
  return _rc;
}

const mpu_errors_t MPU6050::readFifoBlock(Vector3& gyro) {
  uint8_t block[FIFO_BLOCK_SIZE];
  if (readFifo(block, sizeof(block)) == 0) {
    gyro = fromBytes(block, _gyroScale);

#ifdef DUMP_FIFO
    DEBUG_PRINTLN(F("FIFO"));
    dumpData(block, sizeof(block));
#endif

  }
  return _rc;
}

const mpu_errors_t MPU6050::readAcc(Vector3& acc) {
  uint8_t block[6];
  if (mpuRegRead(ACCEL_REG, block, sizeof(block)) == 0) {
    acc = fromBytes(block, _accScale);
  }
  return _rc;
}
