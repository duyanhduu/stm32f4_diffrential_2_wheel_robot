/* *************************************************************
   Encoder driver implementation for STM32F401
   Framework : Arduino (via PlatformIO + stm32duino)

   Hardware timer quadrature decode — replaces the Arduino
   pin-change ISR approach entirely.

   Wiring
   ──────
   Left  encoder A → PA0   (TIM2_CH1, AF1)
   Left  encoder B → PA1   (TIM2_CH2, AF1)
   Right encoder A → PB6   (TIM4_CH1, AF2)
   Right encoder B → PB7   (TIM4_CH2, AF2)

   How it works
   ────────────
   STM32 timers support "encoder interface mode" natively.
   In TI12 mode the timer counts up/down automatically based on
   the phase relationship of CH1 and CH2 — no CPU interrupts at
   all.  readEncoder() just reads the 32-bit (TIM2) or
   16-bit (TIM4) counter register.

   TIM4 overflow handling
   ──────────────────────
   TIM4 is 16-bit (0..65535).  We attach an overflow/underflow
   interrupt and keep a 32-bit accumulator so readEncoder(RIGHT)
   always returns a full 32-bit signed value — same behaviour as
   the Arduino left_enc_pos volatile long.
   ************************************************************ */

#include "encoder_driver.h"
#include "commands.h"   // LEFT / RIGHT defines

/* ── Timer instances ─────────────────────────────────────────── */
HardwareTimer *EncTimerLeft  = nullptr;   // TIM2  PA0/PA1
HardwareTimer *EncTimerRight = nullptr;   // TIM4  PB6/PB7

/* ── TIM4 overflow accumulator (16-bit timer only) ───────────── */
static volatile int32_t right_enc_overflow = 0;
static uint16_t         right_enc_last     = 0;

/* ── Reset offsets (applied in readEncoder) ──────────────────── */
static int32_t left_enc_offset  = 0;
static int32_t right_enc_offset = 0;

/* ── TIM4 overflow/underflow ISR ─────────────────────────────── */
static void onRightEncoderOverflow()
{
    /* Determine direction from counter movement relative to last
       known value.  In encoder mode the timer fires the update
       event on overflow (65535→0) and underflow (0→65535). */
    uint16_t now = (uint16_t)EncTimerRight->getCount();
    if (now < right_enc_last)
        right_enc_overflow += 65536;   // overflowed upward
    else
        right_enc_overflow -= 65536;   // underflowed downward
    right_enc_last = now;
}

/* ── initEncoders ────────────────────────────────────────────── */
void initEncoders()
{
    /* ---- LEFT encoder : TIM2 (32-bit, PA0=CH1, PA1=CH2) ------ */
    TIM_TypeDef *tim2Instance = TIM2;
    EncTimerLeft = new HardwareTimer(tim2Instance);

    /* ---------- ĐOẠN CODE ĐƯỢC SỬA ---------- */
    // Xóa pinMode và pinmap_pinout cũ vì gây lỗi gán nhầm sang TIM5
    // Thay bằng cấu hình cứng HAL_GPIO_Init để đảm bảo ăn vào TIM2 (AF1)
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* ---------------------------------------- */

    /* Set encoder interface mode on TIM2 */
    TIM_Encoder_InitTypeDef encCfg2 = {};
    encCfg2.EncoderMode   = TIM_ENCODERMODE_TI12;   // count both edges
    encCfg2.IC1Polarity   = TIM_ICPOLARITY_RISING;
    encCfg2.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    encCfg2.IC1Prescaler  = TIM_ICPSC_DIV1;
    encCfg2.IC1Filter     = 0x0F;   // digital noise filter (max)
    encCfg2.IC2Polarity   = TIM_ICPOLARITY_RISING;
    encCfg2.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    encCfg2.IC2Prescaler  = TIM_ICPSC_DIV1;
    encCfg2.IC2Filter     = 0x0F;

    TIM_HandleTypeDef *htim2 = EncTimerLeft->getHandle();
    htim2->Init.Period       = 0xFFFFFFFF;  // full 32-bit range
    htim2->Init.Prescaler    = 0;
    htim2->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2->Init.CounterMode   = TIM_COUNTERMODE_UP;
    HAL_TIM_Encoder_Init(htim2, &encCfg2);
    HAL_TIM_Encoder_Start(htim2, TIM_CHANNEL_ALL);

    /* ---- RIGHT encoder : TIM4 (16-bit, PB6=CH1, PB7=CH2) ---- */
    TIM_TypeDef *tim4Instance = TIM4;
    EncTimerRight = new HardwareTimer(tim4Instance);

    /* ---------- ĐOẠN CODE ĐƯỢC SỬA ---------- */
    // Để đồng bộ và an toàn, ta cũng dùng HAL cho bánh phải
    
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4; // PB6, PB7 dùng AF2 cho TIM4
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* ---------------------------------------- */

    TIM_Encoder_InitTypeDef encCfg4 = {};
    encCfg4.EncoderMode   = TIM_ENCODERMODE_TI12;
    encCfg4.IC1Polarity   = TIM_ICPOLARITY_RISING;
    encCfg4.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    encCfg4.IC1Prescaler  = TIM_ICPSC_DIV1;
    encCfg4.IC1Filter     = 0x0F;
    encCfg4.IC2Polarity   = TIM_ICPOLARITY_RISING;
    encCfg4.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    encCfg4.IC2Prescaler  = TIM_ICPSC_DIV1;
    encCfg4.IC2Filter     = 0x0F;

    TIM_HandleTypeDef *htim4 = EncTimerRight->getHandle();
    htim4->Init.Period       = 0xFFFF;   // 16-bit
    htim4->Init.Prescaler    = 0;
    htim4->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4->Init.CounterMode   = TIM_COUNTERMODE_UP;
    HAL_TIM_Encoder_Init(htim4, &encCfg4);
    HAL_TIM_Encoder_Start(htim4, TIM_CHANNEL_ALL);

    /* Attach overflow callback for TIM4 accumulator */
    EncTimerRight->attachInterrupt(onRightEncoderOverflow);
    EncTimerRight->resume();

    right_enc_last = (uint16_t)EncTimerRight->getCount();
}

/* ── readEncoder ─────────────────────────────────────────────── */
/*
 * Returns signed 32-bit tick count from power-on (or last reset).
 * TIM2 is 32-bit so raw counter cast to int32_t is sufficient.
 * TIM4 combines the 16-bit counter with the overflow accumulator.
 */
long readEncoder(int i)
{
    if (i == LEFT)
    {
        /* TIM2: 32-bit, read directly, subtract offset */
        int32_t raw = (int32_t)__HAL_TIM_GET_COUNTER(
                          EncTimerLeft->getHandle());
        return (long)(raw - left_enc_offset);
    }
    else
    {
        /* TIM4: 16-bit + overflow accumulator */
        uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(
                           EncTimerRight->getHandle());
        int32_t combined = right_enc_overflow + (int32_t)cnt;
        return (long)(combined - right_enc_offset);
    }
}

/* ── resetEncoder ────────────────────────────────────────────── */
/*
 * Stores the current position as offset so readEncoder() returns 0.
 * Does NOT reset the hardware counter — avoids a brief glitch where
 * the counter resets mid-edge and causes a spurious count.
 */
void resetEncoder(int i)
{
    if (i == LEFT)
    {
        left_enc_offset = (int32_t)__HAL_TIM_GET_COUNTER(
                              EncTimerLeft->getHandle());
    }
    else
    {
        uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(
                           EncTimerRight->getHandle());
        right_enc_offset = right_enc_overflow + (int32_t)cnt;
    }
}

/* ── resetEncoders ───────────────────────────────────────────── */
void resetEncoders()
{
    resetEncoder(LEFT);
    resetEncoder(RIGHT);
}