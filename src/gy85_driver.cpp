// gy85_driver.cpp
#include "gy85_driver.h"

/* ── I2C helpers ──────────────────────────────────────────── */
static void writeReg(uint8_t dev, uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t readReg(uint8_t dev, uint8_t reg)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(dev, (uint8_t)1);
    return Wire.read();
}

static bool readBurst(uint8_t dev, uint8_t startReg,
                      uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(dev);
    Wire.write(startReg);
    Wire.endTransmission(false);          // repeated start
    uint8_t got = Wire.requestFrom(dev, len);
    if (got != len) return false;
    for (uint8_t i = 0; i < len; i++)
        buf[i] = Wire.read();
    return true;
}

/* ── Gyroscope (ITG3205) ──────────────────────────────────── */
bool checkGyroscope()
{
    uint8_t id = readReg(ITG3205_ADDR, ITG3205_WHO_AM_I) & 0x7E;
    if (id != ITG3205_ADDR) return false;

    writeReg(ITG3205_ADDR, ITG3205_PWR_MGM, ITG3205_RESET);
    delay(10);
    writeReg(ITG3205_ADDR, ITG3205_DLPF_FS, 0x1B); // FS=3, DLPF=3 (42Hz)
    writeReg(ITG3205_ADDR, 0x15, 0x13);             // sample rate divider
    writeReg(ITG3205_ADDR, ITG3205_PWR_MGM, 0x03);  // PLL with Z gyro ref
    return true;
}

/* ── Accelerometer (ADXL345) ──────────────────────────────── */
bool checkAccelerometer()
{
    if (readReg(ADXL345_ADDR, ADXL345_DEVID) != ADXL345_DEVICE_ID)
        return false;

    writeReg(ADXL345_ADDR, ADXL345_POWER_CTL,   0x08); // measure mode
    writeReg(ADXL345_ADDR, ADXL345_DATA_FORMAT,  0x09); // FULL_RES, ±4g
    writeReg(ADXL345_ADDR, ADXL345_BW_RATE,      0x09); // 50 Hz ODR
    return true;
}

/* ── Magnetometer (HMC5883L) ──────────────────────────────── */
bool checkMagnetometer()
{
    writeReg(HMC5883L_ADDR, HMC5883L_REG_B, HMC5883L_GAIN);
    writeReg(HMC5883L_ADDR, HMC5883L_REG_A, 0x18); // 75 Hz, 8 samples avg
    writeReg(HMC5883L_ADDR, HMC5883L_MODE,  0x01); // single measurement
    return true;  // HMC5883L không có WHO_AM_I đáng tin
}

/* ── Init tổng ────────────────────────────────────────────── */
bool initImu()
{
    Wire.setSDA(PB9);
    Wire.setSCL(PB8);
    Wire.begin();
    Wire.setClock(400000);
    delay(100);  // đợi sensor power-on

    bool ok = true;
    ok &= checkAccelerometer();
    ok &= checkGyroscope();
    ok &= checkMagnetometer();
    return ok;
}

/* ── Read all ─────────────────────────────────────────────── */
bool readImu(ImuData &d)
{
    uint8_t buf[6];

    /* --- Accelerometer --- */
    if (!readBurst(ADXL345_ADDR, ADXL345_DATAX0, buf, 6)) return false;
    // ADXL345: little-endian, X0/X1/Y0/Y1/Z0/Z1
    int16_t raw[3];
    raw[ACC_X_AXIS] = (int16_t)((buf[1] << 8) | buf[0]);
    raw[ACC_Y_AXIS] = (int16_t)((buf[3] << 8) | buf[2]);
    raw[ACC_Z_AXIS] = (int16_t)((buf[5] << 8) | buf[4]);
    d.ax = (float)raw[0] / ADXL345_SCALE * 9.80665f;
    d.ay = (float)raw[1] / ADXL345_SCALE * 9.80665f;
    d.az = (float)raw[2] / ADXL345_SCALE * 9.80665f;

    /* --- Gyroscope --- */
    if (!readBurst(ITG3205_ADDR, ITG3205_DATA_REG, buf, 6)) return false;
    // ITG3205: big-endian
    d.gx = (float)(int16_t)((buf[2*GYRO_X_AXIS]   << 8) | buf[2*GYRO_X_AXIS+1]) * ITG3205_SCALE;
    d.gy = (float)(int16_t)((buf[2*GYRO_Y_AXIS]   << 8) | buf[2*GYRO_Y_AXIS+1]) * ITG3205_SCALE;
    d.gz = (float)(int16_t)((buf[2*GYRO_Z_AXIS]   << 8) | buf[2*GYRO_Z_AXIS+1]) * ITG3205_SCALE;

    /* --- Magnetometer (re-trigger single-shot sau mỗi lần đọc) --- */
    if (!readBurst(HMC5883L_ADDR, HMC5883L_DATAX0, buf, 6)) return false;
    // HMC5883L byte order: X_MSB X_LSB Z_MSB Z_LSB Y_MSB Y_LSB (!)
    int16_t mx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t mz = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t my = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t m[3]; m[MAG_X_AXIS]=mx; m[MAG_Y_AXIS]=my; m[MAG_Z_AXIS]=mz;
    d.mx = (float)m[0] * HMC5883L_SCALE;
    d.my = (float)m[1] * HMC5883L_SCALE;
    d.mz = (float)m[2] * HMC5883L_SCALE;
    // re-trigger next single measurement
    writeReg(HMC5883L_ADDR, HMC5883L_MODE, 0x01);

    return true;
}