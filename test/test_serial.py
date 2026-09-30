#!/usr/bin/env python3
import serial
import struct
import threading
import time

# --- CẤU HÌNH CỔNG SERIAL ---
SERIAL_PORT = '/dev/ttyUSB0'  # <-- SỬA LẠI CHO ĐÚNG VỚI MÁY CỦA BẠN (VD: 'COM3')
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Đã kết nối thành công tới {SERIAL_PORT}")
except Exception as e:
    print(f"Lỗi kết nối: {e}")
    exit()

# --- HÀM ĐỌC VÀ GIẢI MÃ LIÊN TỤC TỪ STM32 ---
def read_serial_loop():
    state = 0
    payload_len = 0
    msg_id = 0
    payload = bytearray()
    
    while True:
        if ser.in_waiting > 0:
            c = ser.read(1)[0]
            
            # State Machine giải mã y hệt trên STM32
            if state == 0:
                if c == 0xA5: state = 1
            elif state == 1:
                if c == 0x5A: state = 2
                else: state = 0
            elif state == 2:
                msg_id = c
                state = 3
            elif state == 3:
                payload_len = c
                payload = bytearray()
                state = 4
            elif state == 4:
                payload.append(c)
                if len(payload) >= payload_len:
                    state = 5
            elif state == 5:
                checksum = c
                calc_checksum = sum(payload) % 256 # Tính lại tổng các byte
                
                if checksum == calc_checksum:
                    # Gói tin đúng! Giải nén bằng struct
                    if msg_id == 0x02: # MSG_IMU_DATA (9 biến float)
                        d = struct.unpack('<fffffffff', payload)
                        print(f"[IMU] ax:{d[0]:.2f} | ay:{d[1]:.2f} | az:{d[2]:.2f}")

                    
                    elif msg_id == 0x03: # MSG_ODOM_DATA (5 biến float)
                        data = struct.unpack('<fffff', payload)
                        print(f"[ODOM] x:{data[0]:.2f} | y:{data[1]:.2f} | theta:{data[2]:.2f}")
                else:
                    print(f"[GÓI TIN KHÁC] Nhận được msg_id: {hex(msg_id)}, độ dài: {len(payload)}")
                state = 0

# --- HÀM GÓI VÀ GỬI LỆNH XUỐNG STM32 ---
def send_cmd_vel(linear_x, angular_z):
    # '<ff' nghĩa là nén 2 biến float theo chuẩn Little-Endian
    payload = struct.pack('<ff', linear_x, angular_z) 
    
    header = bytes([0xA5, 0x5A])
    msg_id = bytes([0x01]) # MSG_CMD_VEL
    length = bytes([len(payload)])
    checksum = bytes([sum(payload) % 256])
    
    packet = header + msg_id + length + payload + checksum
    ser.write(packet)
    print(f"\n>>> Đã gửi CMD_VEL: v={linear_x}, w={angular_z} \n")

# --- CHẠY CHƯƠNG TRÌNH ---
# Chạy hàm đọc ở một luồng (thread) riêng biệt để không chặn vòng lặp chính
t = threading.Thread(target=read_serial_loop, daemon=True)
t.start()

# Vòng lặp chính: Cứ 3 giây gửi lệnh chạy, 3 giây sau gửi lệnh dừng
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Đã dừng test.")
    ser.close()