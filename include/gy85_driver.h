// gy85_driver.h
#ifndef GY85_DRIVER_H
#define GY85_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

/* ── ITG3205 Gyroscope ─────────────────────────────── */
#define ITG3205_ADDR        0x68
#define ITG3205_WHO_AM_I    0x00
#define ITG3205_PWR_MGM     0x3E
#define ITG3205_DLPF_FS     0x16
#define ITG3205_DATA_REG    0x1D
#define ITG3205_RESET       0x80
#define ITG3205_SCALE       0.00121414209f   // LSB → rad/s

/* ── ADXL345 Accelerometer ─────────────────────────── */
#define ADXL345_ADDR        0x53
#define ADXL345_DEVID       0x00
#define ADXL345_DEVICE_ID   0xE5
#define ADXL345_BW_RATE     0x2C
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0      0x32
#define ADXL345_SCALE       256.0f           // LSB/(m/s²) at ±4g full-res
                                             // 1g = 256 LSB → /256 = g → ×9.80665 = m/s²
/* ── HMC5883L Magnetometer ─────────────────────────── */
#define HMC5883L_ADDR       0x1E
#define HMC5883L_REG_A      0x00
#define HMC5883L_REG_B      0x01
#define HMC5883L_MODE       0x02
#define HMC5883L_DATAX0     0x03
#define HMC5883L_GAIN       0x20
#define HMC5883L_SCALE      0.92f            // mG/LSB

/* ── Axis remap (giữ nguyên từ Rikibase) ───────────── */
#define GYRO_X_AXIS 1
#define GYRO_Y_AXIS 0
#define GYRO_Z_AXIS 2
#define ACC_X_AXIS  0
#define ACC_Y_AXIS  1
#define ACC_Z_AXIS  2
#define MAG_X_AXIS  0
#define MAG_Y_AXIS  2
#define MAG_Z_AXIS  1

struct ImuData {
    float ax, ay, az;   // m/s²
    float gx, gy, gz;   // rad/s
    float mx, my, mz;   // mG
};

/* Trả về true nếu tất cả 3 sensor đều được tìm thấy */
bool  initImu();

/* Đọc cả 3 sensor một lần, ghi vào struct */
bool  readImu(ImuData &data);

/* Kiểm tra từng sensor riêng */
bool  checkGyroscope();
bool  checkAccelerometer();
bool  checkMagnetometer();

#endif /* GY85_DRIVER_H */