/*********************************************************************
 *  ROSArduinoBridge — STM32F401 port (Binary Protocol Optimized)
 *
 *  Đã được viết lại để sử dụng giao thức Nhị phân (Binary Protocol)
 *  thay vì parse chuỗi ASCII, tối ưu hóa cho tốc độ và bộ nhớ.
 *********************************************************************/

#define USE_BASE      // Enable the base controller code
//#undef USE_BASE

#ifdef USE_BASE
  #define STM32_ENC_COUNTER   // hardware quadrature timer (TIM2/TIM4)
  #define L298_MOTOR_DRIVER
#endif

#undef USE_SERVOS             // servos not used

/* ── Serial port selection ─────────────────────────────────────── */
#define ROS_SERIAL  Serial    // resolved to USART2 via build flag
#define BAUDRATE   115200 
#define MAX_PWM    255

/* ── Core includes ───────────────────────────────────────────── */
#include <Arduino.h>
#include "commands.h"
#include "sensors.h"
#include "gy85_driver.h"
#include "odometry.h"    // Module tính toán động học
#include "imu_filter.h"  // Module lọc IMU (Madgwick)

#ifdef USE_BASE
  #include "motor_driver.h"
  #include "encoder_driver.h"
  #include "diff_controller.h" // Sử dụng module mới
  #define PID_RATE           50  // Tần số PID 50Hz (20ms)
  const int PID_INTERVAL   = 1000 / PID_RATE;
  unsigned long nextPID    = PID_INTERVAL;
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand    = AUTO_STOP_INTERVAL;
#endif

/* ── IMU timing ── */
#define IMU_RATE           50    // Tần số IMU 50Hz (20ms)
const int IMU_INTERVAL   = 1000 / IMU_RATE;
unsigned long nextIMU    = IMU_INTERVAL;
static ImuData imuData;
static bool    imuReady  = false;

/* ── Khai báo biến giao thức Nhị Phân ────────────────────────── */
enum RxState { WAIT_H1, WAIT_H2, READ_ID, READ_LEN, READ_PAYLOAD, READ_CHK };
RxState rx_state = WAIT_H1;
uint8_t rx_id, rx_len, rx_index, rx_checksum;
uint8_t rx_buffer[64]; // Bộ đệm chứa payload

/* ── Hàm hỗ trợ đóng gói & gửi ───────────────────────────────── */
uint8_t calculateChecksum(uint8_t* payload, uint8_t length) {
    uint8_t sum = 0;
    for (int i = 0; i < length; i++) sum += payload[i];
    return sum;
}

void sendBinaryFrame(uint8_t id, uint8_t* payload, uint8_t length) {
    uint8_t header[2] = {0xA5, 0x5A};
    uint8_t checksum = calculateChecksum(payload, length);
    
    ROS_SERIAL.write(header, 2);
    ROS_SERIAL.write(id);
    ROS_SERIAL.write(length);
    ROS_SERIAL.write(payload, length);
    ROS_SERIAL.write(checksum);
}

/* ── Hàm thực thi lệnh sau khi nhận đủ Frame ─────────────────── */
void executeCommand(uint8_t id, uint8_t* payload) {
#ifdef USE_BASE
    if (id == MSG_CMD_VEL) {
        lastMotorCommand = millis();
        CmdVelPayload* cmd = (CmdVelPayload*)payload;
        
        if (cmd->linear_x == 0 && cmd->angular_z == 0) {
            setMotorSpeeds(0, 0);
            resetDiffController();
            moving = 0;
        } else {
            moving = 1;
        }
        
        // Gán vào biến targetTicks hiện có. 
        // LƯU Ý: PC Node sẽ chịu trách nhiệm tính toán Kinematics 
        // (chuyển đổi từ m/s và rad/s sang số Ticks/Frame) và gửi xuống đây.
        targetTicksLeft = cmd->linear_x;  
        targetTicksRight = cmd->angular_z; 
    }
    else if (id == MSG_RESET_ENC) {
        resetEncoders();
        resetDiffController();
        initOdometry();
    }
    else if (id == MSG_UPDATE_PID) {
        PidPayload* pid = (PidPayload*)payload;
        leftPID.updateConstants(pid->kp, pid->ki, pid->kd, pid->ko);
        rightPID.updateConstants(pid->kp, pid->ki, pid->kd, pid->ko);
    }
#endif
}

/* ── Máy trạng thái xử lý từng Byte nhận được ────────────────── */
void processSerial() {
    while (ROS_SERIAL.available() > 0) {
        uint8_t c = ROS_SERIAL.read();
        
        switch (rx_state) {
            case WAIT_H1:      if (c == 0xA5) rx_state = WAIT_H2; break;
            case WAIT_H2:      if (c == 0x5A) rx_state = READ_ID; else rx_state = WAIT_H1; break;
            case READ_ID:      rx_id = c; rx_state = READ_LEN; break;
            case READ_LEN:
                rx_len = c; rx_index = 0; rx_checksum = 0;
                if (rx_len > 64) rx_state = WAIT_H1; // Chống tràn
                else rx_state = READ_PAYLOAD;
                break;
            case READ_PAYLOAD:
                rx_buffer[rx_index++] = c;
                rx_checksum += c;
                if (rx_index >= rx_len) rx_state = READ_CHK;
                break;
            case READ_CHK:
                if (c == rx_checksum) {
                    executeCommand(rx_id, rx_buffer); // Khung truyền chuẩn xác
                }
                rx_state = WAIT_H1;
                break;
        }
    }
}

/* ── setup ───────────────────────────────────────────────────── */
void setup()
{
  /* Mở port ROS bridge */
  ROS_SERIAL.begin(BAUDRATE);
  delay(500);  // Đợi serial ổn định
  
#ifdef USE_BASE
  #ifdef STM32_ENC_COUNTER
    initEncoders();           // TIM2 + TIM4
  #endif

  initMotorController();    // TIM3 PWM
  resetDiffController();
  initOdometry();           // Khởi tạo Odom
#endif /* USE_BASE */
  
  initImu(); 
  initImuFilter(50.0f);     // Lọc Madgwick 50Hz
}

/* ── loop ────────────────────────────────────────────────────── */
void loop()
{
  /* 1. Xử lý UART liên tục (Non-blocking) */
  processSerial();

#ifdef USE_BASE
  /* 2. Cập nhật PID và Gửi Odometry (50Hz) */
  if (millis() > nextPID)
  {
    updateDiffController();
    
    // Cập nhật Odom
    float dt = (float)PID_INTERVAL / 1000.0f; // Luôn là 0.02s
    updateOdometry(readEncoder(LEFT), readEncoder(RIGHT), dt);
    
  // GÓI VÀ BẮN DỮ LIỆU ODOMETRY VỀ PC
    OdomPayload odom;
    odom.x = odom_x; odom.y = odom_y; odom.theta = odom_theta;
    odom.vx = odom_vx; odom.vth = odom_vth;
    // odom.q0 = odom_q0; odom.q1 = odom_q1; odom.q2 = odom_q2; odom.q3 = odom_q3;
    sendBinaryFrame(MSG_ODOM_DATA, (uint8_t*)&odom, sizeof(OdomPayload));

    nextPID += PID_INTERVAL;
  }
  
  /* 3. Tự động dừng xe nếu mất kết nối PC */
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif /* USE_BASE */

  /* 4. Đọc IMU, Lọc Madgwick và Gửi dữ liệu (50Hz) */
  if (millis() > nextIMU)
  {
    imuReady = readImu(imuData);
    if (imuReady) {
      // updateImuFilter(imuData);
      
      // GÓI VÀ BẮN DỮ LIỆU IMU VỀ PC
      ImuPayload imu;
      imu.ax = imuData.ax; imu.ay = imuData.ay; imu.az = imuData.az;
      imu.gx = imuData.gx; imu.gy = imuData.gy; imu.gz = imuData.gz;
      imu.mx = imuData.mx; imu.my = imuData.my; imu.mz = imuData.mz;
      sendBinaryFrame(MSG_IMU_DATA, (uint8_t*)&imu, sizeof(ImuPayload));
    }
    nextIMU += IMU_INTERVAL;
  }
}