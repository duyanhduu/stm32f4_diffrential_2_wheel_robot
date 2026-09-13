#include "odometry.h"
#include <math.h>

float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;
float odom_vx = 0.0f;
float odom_vth = 0.0f;

static long prev_left_ticks = 0;
static long prev_right_ticks = 0;
static bool odom_initialized = false;

void initOdometry() {
    odom_x = 0.0f;
    odom_y = 0.0f;
    odom_theta = 0.0f;
    odom_vx = 0.0f;
    odom_vth = 0.0f;
    odom_initialized = false;
}

void updateOdometry(long left_ticks, long right_ticks, float dt) {
    if (!odom_initialized) {
        prev_left_ticks = left_ticks;
        prev_right_ticks = right_ticks;
        odom_initialized = true;
        return;
    }

    // 1. Tính số tick thay đổi (Delta ticks)
    long d_left_ticks = left_ticks - prev_left_ticks;
    long d_right_ticks = right_ticks - prev_right_ticks;

    prev_left_ticks = left_ticks;
    prev_right_ticks = right_ticks;

    // 2. Chuyển đổi tick sang mét
    float d_left_dist = (float)d_left_ticks / TICKS_PER_METER;
    float d_right_dist = (float)d_right_ticks / TICKS_PER_METER;

    // 3. Tính quãng đường trung tâm và góc xoay thay đổi
    float d_center_dist = (d_left_dist + d_right_dist) / 2.0f;
    float d_theta = (d_right_dist - d_left_dist) / WHEEL_SEPARATION;

    // 4. Tính vận tốc (Twist)
    if (dt > 0.0f) {
        odom_vx = d_center_dist / dt;
        odom_vth = d_theta / dt;
    } else {
        odom_vx = 0.0f;
        odom_vth = 0.0f;
    }

    // 5. Cập nhật vị trí (Pose) bằng phương pháp xấp xỉ Mid-point (Runge-Kutta bậc 2)
    float mid_theta = odom_theta + (d_theta / 2.0f);
    odom_x += d_center_dist * cosf(mid_theta);
    odom_y += d_center_dist * sinf(mid_theta);
    odom_theta += d_theta;

    // 6. Chuẩn hóa góc theta về khoảng [-PI, PI]
    odom_theta = atan2f(sinf(odom_theta), cosf(odom_theta));
}