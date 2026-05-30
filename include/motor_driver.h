/* *************************************************************
   Motor driver for STM32F401 — L298N, TIM3 PWM
   Replaces: motor_driver.h (Arduino pin number defines)

   PWM pins — all on TIM3, 8-bit resolution (ARR = 255):
     PB4  RIGHT_MOTOR_BACKWARD   TIM3_CH1  AF2
     PB5  RIGHT_MOTOR_FORWARD    TIM3_CH2  AF2
     PB0  LEFT_MOTOR_BACKWARD    TIM3_CH3  AF2
     PB1  LEFT_MOTOR_FORWARD     TIM3_CH4  AF2

   GPIO enable pins (digital out, active-HIGH):
     PB12  LEFT_MOTOR_ENABLE
     PB13  RIGHT_MOTOR_ENABLE

   All four channels share one timer → guaranteed same PWM
   frequency and zero channel-to-channel phase jitter.
   ************************************************************ */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <HardwareTimer.h>

/* ── Pin definitions ──────────────────────────────────────────
   Using PinName (PA0-style) so stm32duino maps them correctly
   to their alternate-function timer channels.
   ─────────────────────────────────────────────────────────── */
#define RIGHT_MOTOR_BACKWARD  PB4   // TIM3_CH1
#define RIGHT_MOTOR_FORWARD   PB5   // TIM3_CH2
#define LEFT_MOTOR_BACKWARD   PB0   // TIM3_CH3
#define LEFT_MOTOR_FORWARD    PB1   // TIM3_CH4

#define LEFT_MOTOR_ENABLE     PB12  // GPIO output
#define RIGHT_MOTOR_ENABLE    PB13  // GPIO output

/* ── PWM resolution ───────────────────────────────────────────
   8-bit (0–255) matches the original Arduino analogWrite range.
   Frequency = 84 MHz / (prescaler+1) / (ARR+1)
             = 84 MHz /  1            /  256   ≈ 328 kHz
   Too high for L298N audible whine; lower to ~20 kHz:
             prescaler = 15  →  84 MHz / 16 / 256 ≈ 20.5 kHz
   ─────────────────────────────────────────────────────────── */
#define MOTOR_PWM_PRESCALER   15    // 84 MHz / 16 / 256 ≈ 20.5 kHz
#define MOTOR_PWM_MAX         255   // keep same scale as original code

/* ── Timer instance (shared across .h and .cpp) ────────────── */
extern HardwareTimer *MotorTimer;   // TIM3

/* ── Public API — identical signatures to original ─────────── */
void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

#endif /* MOTOR_DRIVER_H */