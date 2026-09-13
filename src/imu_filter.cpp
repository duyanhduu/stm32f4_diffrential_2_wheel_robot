#include "imu_filter.h"
#include <math.h>

Madgwick filter;

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

void initImuFilter(float sampleFrequency) {
    filter.begin(sampleFrequency); // Ví dụ: 50.0f cho 50Hz
}

void updateImuFilter(const ImuData &rawData) {
    // 1. Chuyển đổi đơn vị từ hệ SI sang đơn vị Madgwick yêu cầu
    // Gyro: rad/s -> deg/s (nhân với 180/PI)
    float gx_deg = rawData.gx * 57.2957795f;
    float gy_deg = rawData.gy * 57.2957795f;
    float gz_deg = rawData.gz * 57.2957795f;

    // Accel: m/s^2 -> g (chia cho 9.80665)
    float ax_g = rawData.ax / 9.80665f;
    float ay_g = rawData.ay / 9.80665f;
    float az_g = rawData.az / 9.80665f;

    // Mag: mG (giữ nguyên, thư viện tự chuẩn hóa vector từ trường)
    float mx = rawData.mx;
    float my = rawData.my;
    float mz = rawData.mz;

    // 2. Chạy thuật toán lọc (Lưu ý thứ tự truyền vào: gx, gy, gz, ax, ay, az, mx, my, mz)
    filter.update(gx_deg, gy_deg, gz_deg, ax_g, ay_g, az_g, mx, my, mz);

    // 3. Trích xuất Quaternion
    // Thư viện Madgwick dùng q0, q1, q2, q3 tương ứng với w, x, y, z
    q0 = filter.q0;
    q1 = filter.q1;
    q2 = filter.q2;
    q3 = filter.q3;
}