#include "odometry.h"
#include <math.h>

float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;
float odom_vx = 0.0f;
float odom_vth = 0.0f;

// Khai báo biến với tên mới
float odom_q0 = 1.0f;
float odom_q1 = 0.0f;
float odom_q2 = 0.0f;
float odom_q3 = 0.0f;

static long prev_left_ticks = 0;
static long prev_right_ticks = 0;
static bool odom_initialized = false;

void initOdometry() {
    odom_x = 0.0f;
    odom_y = 0.0f;
    odom_theta = 0.0f;
    odom_vx = 0.0f;
    odom_vth = 0.0f;
    
    odom_q0 = 1.0f;
    odom_q1 = 0.0f;
    odom_q2 = 0.0f;
    odom_q3 = 0.0f;
    
    odom_initialized = false;
}

void updateOdometry(long left_ticks, long right_ticks, float dt) {
    if (!odom_initialized) {
        prev_left_ticks = left_ticks;
        prev_right_ticks = right_ticks;
        odom_initialized = true;
        return;
    }

    long d_left_ticks = left_ticks - prev_left_ticks;
    long d_right_ticks = right_ticks - prev_right_ticks;

    prev_left_ticks = left_ticks;
    prev_right_ticks = right_ticks;

    float d_left_dist = (float)d_left_ticks / TICKS_PER_METER;
    float d_right_dist = (float)d_right_ticks / TICKS_PER_METER;

    float d_center_dist = (d_left_dist + d_right_dist) / 2.0f;
    float d_theta = (d_right_dist - d_left_dist) / WHEEL_SEPARATION;

    if (dt > 0.0f) {
        odom_vx = d_center_dist / dt;
        odom_vth = d_theta / dt;
    } else {
        odom_vx = 0.0f;
        odom_vth = 0.0f;
    }

    float mid_theta = odom_theta + (d_theta / 2.0f);
    odom_x += d_center_dist * cosf(mid_theta);
    odom_y += d_center_dist * sinf(mid_theta);
    odom_theta += d_theta;

    odom_theta = atan2f(sinf(odom_theta), cosf(odom_theta));
    
    // Cập nhật vào biến mới
    odom_q0 = cosf(odom_theta * 0.5f);
    odom_q1 = 0.0f;
    odom_q2 = 0.0f;
    odom_q3 = sinf(odom_theta * 0.5f);
}