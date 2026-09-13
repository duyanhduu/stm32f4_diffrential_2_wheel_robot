#ifndef ODOMETRY_H
#define ODOMETRY_H

/* Cấu hình vật lý của robot */
#define WHEEL_SEPARATION     0.17f    // Khoảng cách 2 bánh (mét)
#define TICKS_PER_METER      6567.0f  // Số xung trên 1 mét

/* Các biến trạng thái toàn cục để truyền qua UART sau này */
extern float odom_x;
extern float odom_y;
extern float odom_theta;
extern float odom_vx;
extern float odom_vth;

void initOdometry();
void updateOdometry(long left_ticks, long right_ticks, float dt);

#endif /* ODOMETRY_H */