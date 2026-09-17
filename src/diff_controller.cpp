#include "diff_controller.h"
#include "encoder_driver.h" // Chứa hàm readEncoder()
#include "motor_driver.h"   // Chứa hàm setMotorSpeeds()
#include "commands.h"       // Chứa định nghĩa MAX_PWM

#ifndef MAX_PWM
#define MAX_PWM 255
#endif
// Khởi tạo 2 object PID độc lập, giới hạn output từ -MAX_PWM đến MAX_PWM.
// Sử dụng bộ số mặc định gốc của bạn: Kp=5, Ki=0, Kd=1, Ko=50
IncrementalPID leftPID(-MAX_PWM, MAX_PWM, 4, 0, 1, 50);
IncrementalPID rightPID(-MAX_PWM, MAX_PWM, 4, 0, 1, 50);

double targetTicksLeft = 0.0;
double targetTicksRight = 0.0;
unsigned char moving = 0;

// ----------------------------------------------------
// CÀI ĐẶT LỚP INCREMENTAL PID
// ----------------------------------------------------
IncrementalPID::IncrementalPID(int min_val, int max_val, int kp, int ki, int kd, int ko) :
    min_val_(min_val), max_val_(max_val), kp_(kp), ki_(ki), kd_(kd), ko_(ko),
    prevEnc_(0), prevInput_(0), ITerm_(0), output_(0) {}

int IncrementalPID::compute(double targetTicksPerFrame, long currentEncoder) {
    long input = currentEncoder - prevEnc_;
    long Perror = targetTicksPerFrame - input;

    // XỬ LÝ ĐIỂM DỪNG TUYỆT ĐỐI (Zero Setpoint/Error Handling)
    // Dọn sạch giá trị tích lũy khi lệnh yêu cầu dừng và xe đã dừng hẳn
    if (targetTicksPerFrame == 0.0 && Perror == 0) {
        ITerm_ = 0;
    }

    // Công thức nguyên bản của bạn:
    long out = (kp_ * Perror - kd_ * (input - prevInput_) + ITerm_) / ko_;
    prevEnc_ = currentEncoder;
    
    // Tính chất gia số: cộng dồn lượng thay đổi vào output hiện tại
    out += output_; 

    // Kiểm soát giới hạn linh hoạt (Flexible Output Constraints)
    if (out >= max_val_) {
        out = max_val_;
    } else if (out <= min_val_) {
        out = min_val_;
    } else {
        // Chỉ tích lũy I-term khi output chưa bão hòa (Anti-windup)
        ITerm_ += ki_ * Perror;
    }

    output_ = out;
    prevInput_ = input;

    return output_;
}

void IncrementalPID::updateConstants(int kp, int ki, int kd, int ko) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    ko_ = ko;
}

void IncrementalPID::reset(long currentEncoder) {
    prevEnc_ = currentEncoder;
    prevInput_ = 0;
    ITerm_ = 0;
    output_ = 0;
}

// ----------------------------------------------------
// CÁC HÀM QUẢN LÝ GIAO TIẾP VỚI MAIN.CPP
// ----------------------------------------------------
void resetDiffController() {
    targetTicksLeft = 0.0;
    targetTicksRight = 0.0;
    
    // Truyền encoder hiện tại vào để chống startup spikes
    leftPID.reset(readEncoder(LEFT));
    rightPID.reset(readEncoder(RIGHT));
}

void updateDiffController() {
    // Nếu hệ thống không di chuyển
    if (!moving) {
        // Kiểm tra PrevInput như proxy xem đã reset chưa (giống code cũ)
        if (leftPID.getPrevInput() != 0 || rightPID.getPrevInput() != 0) {
            resetDiffController();
        }
        return;
    }

    // Tính toán PID cho từng bánh riêng biệt
    int outLeft = leftPID.compute(targetTicksLeft, readEncoder(LEFT));
    int outRight = rightPID.compute(targetTicksRight, readEncoder(RIGHT));

    // Xuất ra Motor Driver
    setMotorSpeeds(outLeft, outRight);
}