#include "Arduino.h"
#include <Wire.h>

#include "mpu6050mm.h"

#define SERIAL_BPS  115200
#define WIRE_CLOCK  400000

#define EXPECTED_FREQUENCE  125.0
#define FREQUENCE_THRESHOLD 10.0
#define GYRO_THRESHOLD      4.0
#define ACC_THRESHOLD_G     0.1
#define YAW_THRESHOLD_DEG   3

#define GYRO_TEST_DURATION_MS 1000

#define READ_ACC_TIMEOUT_US   10000
#define READ_FIFO_TIMEOUT_US  10000

MPU6050Class Mpu;
static int sampleCounter;

void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.println("");
  Wire.begin();
  Wire.setClock(WIRE_CLOCK);
  Wire.setTimeOut(50);
  int n = 0;
  int err = 0;
  if (!testRead(++n, "WHO_AM_I_REG", WHO_AM_I_REG, 0x68)) {
    err++;
  }
  if (!testWriteRead(++n, "PWR_MGMT_1_REG", PWR_MGMT_1_REG, CLKSEL_Z_AXIS, CLKSEL_Z_AXIS)) {
    err++;
  }
  if (!testWriteRead(++n, "SMPRT_DIV_REG", SMPRT_DIV_REG, 7, 7)) {
    err++;
  }
  if (!testWriteRead(++n, "CONFIG_REG", CONFIG_REG, DLPF_1, DLPF_1)) {
    err++;
  }
  if (!testWriteRead(++n, "GYRO_CONFIG_REG", GYRO_CONFIG_REG,
                     FS_2000 << 3, FS_2000 << 3)) {
    err++;
  }
  if (!testWriteRead(++n, "ACCEL_CONFIG_REG", ACCEL_CONFIG_REG,
                     AFS_16G << 3, AFS_16G << 3)) {
    err++;
  }
  if (!testWriteRead(++n, "FIFO_EN_REG",
                     FIFO_EN_REG, XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN,
                     XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN)) {
    err++;
  }
  if (!testWriteRead(++n, "INT_PIN_CFG_REG", INT_PIN_CFG_REG,
                     INT_LEVEL | LATCH_INT_EN | INT_RD_CLEAR,
                     INT_LEVEL | LATCH_INT_EN | INT_RD_CLEAR)) {
    err++;
  }
  if (!testWriteRead(++n, "INT_ENABLE_REG", INT_ENABLE_REG,
                     FIFO_OFLOW_EN | I2C_MST_EN | DATA_RDY_EN,
                     FIFO_OFLOW_EN | I2C_MST_EN | DATA_RDY_EN)) {
    err++;
  }
  if (!testWriteRead(++n, "USER_CTRL_REG", USER_CTRL_REG, FIFO_EN_UC, FIFO_EN_UC)) {
    err++;
  }

  if (!testReadAcc(++n)) {
    err++;
  }

  if (!testReadFifo(++n)) {
    err++;
  }
  if (!testCalibration(++n)) {
    err++;
  }
  if (!testGyro(++n)) {
    err++;
  }
  Serial.println();
  Serial.print("Failed ");
  Serial.print(err);
  Serial.print(", success ");
  Serial.print(n - err);
  Serial.print(", total ");
  Serial.print(n);
  Serial.println();
}

void loop() {
  delay(10000);
}

static const boolean testGyro(const int n) {
  char name[128];
  sprintf(name, "Test gyroscope");
  Serial.println();
  Serial.print(n);
  Serial.print(". ");
  Serial.print(name);
  Serial.println();

  uint8_t rc = Mpu.reset();
  if (rc != 0) {
    Serial.print("KO reset rc=");
    printRc(rc);
    Serial.println();
    return false;
  }
  rc = Mpu.begin();
  if (rc != 0) {
    Serial.print("KO begin rc=");
    printRc(rc);
    Serial.println();
    return false;
  }
  rc = Mpu.calibrate();
  if (rc != 0) {
    Serial.print("KO calibration rc=");
    printRc(rc);
    Serial.println();
    return false;
  }

  const unsigned long t0 = millis();
  const unsigned long timeout = t0 + GYRO_TEST_DURATION_MS;
  unsigned long t1;
  int yaw;
  sampleCounter = 0;
  Mpu.onData([](void *ctx) {
    sampleCounter++;
  });
  do {
    t1 = millis();
    Mpu.polling(t1);
    rc = Mpu.rc();
    if (rc) {
      break;
    }
    yaw = roundf(Mpu.yaw() * 180 / PI);
  } while (abs(yaw) <= YAW_THRESHOLD_DEG && t1 < timeout);

  if (rc != 0) {
    Serial.print("KO polling rc=");
    printRc(rc);
    Serial.println();
    return false;
  }

  float freq = 1000.0 * sampleCounter / (t1 - t0);
  if (abs(freq - EXPECTED_FREQUENCE) > FREQUENCE_THRESHOLD) {
    Serial.print("KO ");
    Serial.print(name);
    Serial.print(" unexpected frequence: ");
    Serial.print(freq, 2);
    Serial.print(", expected: ");
    Serial.print(EXPECTED_FREQUENCE);
    Serial.println();
    return false;
  }

  Serial.print("OK ");
  Serial.print(name);
  Serial.print(", yaw: ");
  Serial.print(yaw);
  Serial.print(", sample counter: ");
  Serial.print(sampleCounter);
  Serial.println();
  return true;
}

static const boolean testCalibration(const int n) {
  char name[128];
  sprintf(name, "Test calibration");
  Serial.println();
  Serial.print(n);
  Serial.print(". ");
  Serial.print(name);
  Serial.println();

  uint8_t rc = Mpu.reset();
  if (rc != 0) {
    Serial.print("KO reset rc=");
    printRc(rc);
    Serial.println();
    return false;
  }
  rc = Mpu.begin();
  if (rc != 0) {
    Serial.print("KO begin rc=");
    printRc(rc);
    Serial.println();
    return false;
  }
  rc = Mpu.calibrate();
  if (rc != 0) {
    Serial.print("KO calibration rc=");
    printRc(rc);
    Serial.println();
    return false;
  }

  const Vector3& grav = Mpu.gravity();

  if (abs(grav[0]) > ACC_THRESHOLD_G
      || abs(grav[1]) > ACC_THRESHOLD_G
      || abs(grav[2]) < 1 - ACC_THRESHOLD_G
      || abs(grav[2]) > 1 + ACC_THRESHOLD_G) {
    Serial.print("KO gravity out of range, ");
    Serial.print(grav.toString());
    Serial.println();
    return false;
  }

  const Vector3& gyroOffset = Mpu.gyroOffset();
  if (abs(gyroOffset[0]) > GYRO_THRESHOLD
      || abs(gyroOffset[1]) > GYRO_THRESHOLD
      || abs(gyroOffset[2]) > GYRO_THRESHOLD) {
    Serial.print("KO gyroOffset out of range, ");
    Serial.print(gyroOffset.toString());
    Serial.println();
    return false;
  }

  Serial.print("OK ");
  Serial.print(name);
  Serial.print(" gravity: ");
  Serial.print(grav.toString());
  Serial.print(", offset: ");
  Serial.print(gyroOffset.toString());
  Serial.println();
  return true;
}

static const boolean testReadFifo(const int n) {
  char name[128];
  sprintf(name, "Test mpu read fifo");
  // Write divider
  Serial.println();
  Serial.print(n);
  Serial.print(". ");
  Serial.print(name);
  Serial.println();

  uint8_t rc = Mpu.reset();
  if (rc != 0) {
    Serial.print("KO reset rc=");
    printRc(rc);
    Serial.println();
    return false;
  }
  rc = Mpu.begin();
  if (rc != 0) {
    Serial.print("KO begin rc=");
    printRc(rc);
    Serial.println();
    return false;
  }
  rc = Mpu.readIntStatus();
  if (rc != 0) {
    Serial.print("KO readStatus rc=");
    printRc(rc);
    Serial.println();
    return false;
  }

  // Wait for fifo data
  unsigned long t0 = micros();
  unsigned long timout = t0 + READ_FIFO_TIMEOUT_US;
  unsigned long t1;
  uint16_t ct;
  do {
    ct = Mpu.readFifoCount();
    t1 = micros();
  } while (ct < 6 && t1 <= timout);
  if (ct < 6) {
    Serial.print("KO timed out fifo count: ");
    Serial.print(ct);
    Serial.println();
    return false;
  }
  // Wait for data ready
  rc = Mpu.readIntStatus();
  if (rc != 0) {
    Serial.print("KO readStatus rc=");
    printRc(rc);
    Serial.println();
    return false;
  }
  if (Mpu.fifoOverflow()) {
    Serial.print("KO fifo overflowed");
    Serial.println();
    return false;
  }

  Vector3 gyro;
  rc = Mpu.readFifoBlock(gyro);
  if (rc != 0) {
    Serial.print("KO readFifoBlock rc=");
    printRc(rc);
    Serial.println();
    return false;
  }

  Serial.print("OK ");
  Serial.print(name);
  Serial.print(", ");
  Serial.print(gyro.toString());
  Serial.print(" in ");
  Serial.print(t1 - t0);
  Serial.print(" us, fifo count: ");
  Serial.print(ct);
  Serial.println();
  return true;
}

static const boolean testReadAcc(const int n) {
  char name[128];
  sprintf(name, "Test mpu read acceleration");
  // Write divider
  Serial.println();
  Serial.print(n);
  Serial.print(". ");
  Serial.print(name);
  Serial.println();

  uint8_t rc = Mpu.reset();
  if (rc != 0) {
    Serial.print("KO reset rc=");
    printRc(rc);
    Serial.println();
    return false;
  }

  rc = Mpu.begin();
  if (rc != 0) {
    Serial.print("KO begin rc=");
    printRc(rc);
    Serial.println();
    return false;
  }

  // Wait for data ready
  unsigned long t0 = micros();
  unsigned long timout = t0 + READ_ACC_TIMEOUT_US;
  unsigned long t1;
  do {
    rc = Mpu.readIntStatus();
    if (rc != 0) {
      Serial.print("KO readStatus rc=");
      printRc(rc);
      Serial.println();
      return false;
    }
    t1 = micros();
  } while (!Mpu.dataReady() && t1 <= timout);

  if (!Mpu.dataReady()) {
    Serial.print("KO data ready timed out");
    Serial.println();
    return false;
  }

  Vector3 acc;
  rc = Mpu.readAcc(acc);
  if (rc != 0) {
    Serial.print("KO readAcc rc=");
    printRc(rc);
    Serial.println();
    return false;
  }

  Serial.print("OK ");
  Serial.print(name);
  Serial.print(", ");
  Serial.print(acc.toString());
  Serial.print(" in ");
  Serial.print(t1 - t0);
  Serial.print(" us");
  Serial.println();
  return true;
}

static const boolean testWriteRead(const int n, const char* regName, const uint8_t reg, const uint8_t value, const uint8_t expected) {
  return testWrite(n, regName, reg, value)
         && testRead(n, regName, reg, expected);
}

static const boolean testRead(const int n, const char* regName, const uint8_t reg, const uint8_t expected) {
  char name[128];
  sprintf(name, "Test mpu read registry %s (0x%hx)", regName, reg);
  // Write divider
  Serial.println();
  Serial.print(n);
  Serial.print(". ");
  Serial.print(name);
  Serial.println();
  uint8_t bfr;
  uint8_t rc = Mpu.mpuRegRead(reg, &bfr, 1);
  if (rc != 0) {
    Serial.print("KO rc=");
    printRc(rc);
    Serial.println();
    return false;
  }
  if (bfr != expected) {
    Serial.print("KO wrong value 0x");
    Serial.print(bfr, HEX);
    Serial.print(", expected 0x");
    Serial.print(expected, HEX);
    Serial.println();
    return false;
  }
  Serial.print("OK ");
  Serial.print(name);
  Serial.println();
  return true;
}


static const boolean testWrite(const int n, const char*regName, const uint8_t reg, const uint8_t value) {
  char name[128];
  sprintf(name, "Test mpu write registry %s (0x%hx) value 0x%hx", regName, reg, value);
  // Write divider
  Serial.println();
  Serial.print(n);
  Serial.print(". ");
  Serial.print(name);
  Serial.println();
  uint8_t rc = Mpu.mpuRegWrite(reg, value);
  if (rc != 0) {
    Serial.print("KO rc=");
    printRc(rc);
    Serial.println();
    return false;
  }
  Serial.print("OK ");
  Serial.print(name);
  Serial.println();
  return true;
}

static void printRc(const uint8_t rc) {
  Serial.print(rc);
  switch (rc) {
    case 0:
      Serial.print(" OK");
      break;
    case 1:
      Serial.print(" data too long to fit in transmit buffer");
      break;
    case 2:
      Serial.print(" received NACK on transmit of address");
      break;
    case 3:
      Serial.print(" received NACK on transmit of data");
      break;
    case 4:
      Serial.print(" other error");
      break;
    case 5:
      Serial.print(" timeout");
      break;
    case READ_LEN_ERROR:
      Serial.print(" read length error");
      break;
  }
}

void sendReply(const char* data) {}
