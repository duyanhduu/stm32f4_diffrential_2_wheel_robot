#ifndef DIFF_CONTROLLER_H
#define DIFF_CONTROLLER_H

#include <Arduino.h>

// Class PID giữ nguyên bản chất "Gia số" (Incremental) của code cũ
class IncrementalPID {
public:
    // Constructor khởi tạo giới hạn và hệ số độc lập cho từng bánh
    IncrementalPID(int min_val, int max_val, int kp, int ki, int kd, int ko);
    
    // Tính toán PWM dựa trên mục tiêu và giá trị encoder hiện hành
    int compute(double targetTicksPerFrame, long currentEncoder);
    
    // Cập nhật thông số PID lúc runtime
    void updateConstants(int kp, int ki, int kd, int ko);
    
    // Khôi phục trạng thái ban đầu, xóa tích lũy (Chống giật khi khởi động)
    void reset(long currentEncoder);
    
    // Lấy giá trị PWM lưu trữ hiện tại
    int getPrevInput() const { return prevInput_; }

private:
    int min_val_;
    int max_val_;
    int kp_;
    int ki_;
    int kd_;
    int ko_;
    
    long prevEnc_;
    int prevInput_;
    int ITerm_;
    long output_;
};

// Khai báo 2 đối tượng PID cho 2 động cơ để dùng ở main.cpp
extern IncrementalPID leftPID;
extern IncrementalPID rightPID;

// Trạng thái vận hành
extern double targetTicksLeft;
extern double targetTicksRight;
extern unsigned char moving;

// Khai báo các hàm quản lý điều khiển
void resetDiffController();
void updateDiffController();

#endif