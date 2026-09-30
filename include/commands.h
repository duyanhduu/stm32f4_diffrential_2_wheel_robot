#ifndef COMMANDS_H
#define COMMANDS_H

/* ==========================================================
 * ĐỊNH NGHĨA CÁC MESSAGE ID (DÙNG CHO PROTOCOL NHỊ PHÂN)
 * ==========================================================*/
#define MSG_CMD_VEL       0x01  // Nhận từ PC (thay cho 'm')
#define MSG_IMU_DATA      0x02  // Gửi lên PC (thay cho 'i')
#define MSG_ODOM_DATA     0x03  // Gửi lên PC (thay cho 'q')
#define MSG_UPDATE_PID    0x04  // Nhận từ PC (thay cho 'u')
#define MSG_RESET_ENC     0x05  // Nhận từ PC (thay cho 'r')

/* ==========================================================
 * CẤU TRÚC DỮ LIỆU PAYLOAD
 * ==========================================================*/
// Bắt buộc sử dụng pack(push, 1) để ép trình biên dịch không chèn thêm 
// byte trống (padding) vào giữa các biến. Đảm bảo dữ liệu sát nhau.
#pragma pack(push, 1)

// Payload cho Topic /cmd_vel (8 bytes)
typedef struct {
    float linear_x;
    float angular_z;
} CmdVelPayload;

// Payload cho Topic /imu/data (36 bytes - 9 biến float)
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
} ImuPayload;

// Payload cho Topic /odom/unfiltered (36 bytes - 9 biến float)
typedef struct {
    float x, y, theta;
    float vx, vth;
} OdomPayload;

// Payload cho việc Tune PID (16 bytes - 4 biến float hoặc int tùy bạn)
typedef struct {
    int kp;
    int kd;
    int ki;
    int ko;
} PidPayload;

#pragma pack(pop) // Trả lại chế độ căn lề bộ nhớ mặc định

/* (Tùy chọn) Giữ lại define cho bánh xe nếu các hàm khác vẫn cần */
#define LEFT            0
#define RIGHT           1

#endif