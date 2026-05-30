/* *************************************************************
   Encoder driver for STM32F401 - Hardware Quadrature Timer Mode
   Replaces: ARDUINO_ENC_COUNTER (pin-change ISR) implementation

   Left  encoder: TIM2  — PA0 (CH1), PA1 (CH2)  [32-bit timer]
   Right encoder: TIM4  — PB6 (CH1), PB7 (CH2)  [16-bit timer]

   Uses TIM_ENCODERMODE_TI12 (count on both edges of CH1 and CH2)
   → 4x resolution vs single-edge counting.
   No ISR needed — counter register updated by hardware automatically.
   ************************************************************ */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>
#include <HardwareTimer.h>

/* ---------------------------------------------------------------
   Timer instances
   TIM2 is 32-bit on STM32F401 → no overflow management needed
     for typical robot odometry distances.
   TIM4 is 16-bit → we track overflow in software (see .cpp).
   --------------------------------------------------------------- */
extern HardwareTimer *EncTimerLeft;   // TIM2
extern HardwareTimer *EncTimerRight;  // TIM4

/* ---------------------------------------------------------------
   Public API — same signatures as the original Arduino version
   so diff_controller.h and ROSArduinoBridge need zero changes.
   --------------------------------------------------------------- */
void    initEncoders();
long    readEncoder(int i);
void    resetEncoder(int i);
void    resetEncoders();

#endif /* ENCODER_DRIVER_H */