/* *************************************************************
   Motor driver implementation for STM32F401
   Framework : Arduino (via PlatformIO + stm32duino)

   L298N dual H-bridge, driven by TIM3 hardware PWM.

   Why one timer for all four channels?
   ─────────────────────────────────────
   Arduino's analogWrite() picks whatever timer owns each pin
   independently.  When two channels share a timer but are
   initialised separately, their PWM frequencies can differ and
   the prescaler of the second init may silently clobber the
   first.  Using TIM3 explicitly for all four channels avoids
   this — one prescaler, one ARR, four CCR registers.

   L298N truth table (per side):
     IN1  IN2   Action
      PWM  0    Forward  (speed = PWM duty)
      0    PWM  Backward (speed = PWM duty)
      0    0    Coast / free-wheel
      1    1    Brake    (not used here)

   We use the same "one side PWM, other side 0" logic as the
   original Arduino code, so PID behaviour is unchanged.
   ************************************************************ */

#include "motor_driver.h"
#include "commands.h"   // LEFT / RIGHT

/* ── Timer instance ──────────────────────────────────────────── */
HardwareTimer *MotorTimer = nullptr;

/* ── Channel map helpers ─────────────────────────────────────── */
/* Returns the TIM3 channel number (1-based) for a given pin.
   stm32duino's STM_PIN_CHANNEL() macro does the lookup via
   PinMap_TIM[], but we hard-code here for clarity and to avoid
   a second pinmap walk at runtime.                              */
static inline uint32_t pinToChannel(uint32_t pin)
{
    if (pin == RIGHT_MOTOR_BACKWARD) return TIM_CHANNEL_1;  // PB4
    if (pin == RIGHT_MOTOR_FORWARD)  return TIM_CHANNEL_2;  // PB5
    if (pin == LEFT_MOTOR_BACKWARD)  return TIM_CHANNEL_3;  // PB0
    if (pin == LEFT_MOTOR_FORWARD)   return TIM_CHANNEL_4;  // PB1
    return TIM_CHANNEL_1;  // fallback (should never hit)
}

/* ── setPWM — write a duty cycle to one TIM3 channel ─────────── */
static void setPWM(uint32_t pin, uint16_t duty)
{
    /* Clamp to 8-bit range */
    if (duty > MOTOR_PWM_MAX) duty = MOTOR_PWM_MAX;

    /* Write directly to the CCR register via HAL macro.
       This is equivalent to analogWrite() but guaranteed to use
       the correct timer/channel without re-initialising TIM3.  */
    __HAL_TIM_SET_COMPARE(MotorTimer->getHandle(),
                          pinToChannel(pin),
                          duty);
}

/* ── initMotorController ─────────────────────────────────────── */
void initMotorController()
{
    /* ---- Enable pins: digital output, start HIGH ---- */
    pinMode(LEFT_MOTOR_ENABLE,  OUTPUT);
    pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
    digitalWrite(LEFT_MOTOR_ENABLE,  HIGH);
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);

    /* ---- Configure TIM3 for PWM on all four channels ---- */
    TIM_TypeDef *tim3Instance = TIM3;
    MotorTimer = new HardwareTimer(tim3Instance);

    /* Set prescaler and period for ~20.5 kHz PWM
       (above L298N audible range, well within its switching spec) */
    MotorTimer->setPrescaleFactor(MOTOR_PWM_PRESCALER + 1);
    MotorTimer->setOverflow(MOTOR_PWM_MAX + 1, TICK_FORMAT);

    /* Configure each pin as TIM3 alternate function output,
       then set PWM mode 1 (high when CNT < CCR) on its channel. */
    const uint32_t pwmPins[] = {
        RIGHT_MOTOR_BACKWARD,   // CH1 PB4
        RIGHT_MOTOR_FORWARD,    // CH2 PB5
        LEFT_MOTOR_BACKWARD,    // CH3 PB0
        LEFT_MOTOR_FORWARD      // CH4 PB1
    };
    const uint32_t channels[] = {
        TIM_CHANNEL_1, TIM_CHANNEL_2,
        TIM_CHANNEL_3, TIM_CHANNEL_4
    };

    for (int i = 0; i < 4; i++)
    {
        /* Wire pin to timer alternate function.
           digitalPinToPinName() converts the Arduino uint32_t pin
           number to the PinName enum pinmap_pinout() expects.     */
        pinmap_pinout(digitalPinToPinName(pwmPins[i]), PinMap_TIM);

        /* Configure channel in PWM mode 1, no preload needed */
        TIM_OC_InitTypeDef ocCfg = {};
        ocCfg.OCMode      = TIM_OCMODE_PWM1;
        ocCfg.Pulse       = 0;                  // start at 0% duty
        ocCfg.OCPolarity  = TIM_OCPOLARITY_HIGH;
        ocCfg.OCFastMode  = TIM_OCFAST_DISABLE;
        HAL_TIM_PWM_ConfigChannel(MotorTimer->getHandle(),
                                  &ocCfg, channels[i]);
        HAL_TIM_PWM_Start(MotorTimer->getHandle(), channels[i]);
    }

    /* Ensure motors start at rest */
    setMotorSpeeds(0, 0);
}

/* ── setMotorSpeed ───────────────────────────────────────────── */
/*
 * spd range : -255 … +255
 *   positive → forward
 *   negative → backward
 *   zero     → coast (both INx = 0)
 *
 * Logic mirrors the original Arduino implementation exactly so
 * the PID output scale requires no recalibration.
 */
void setMotorSpeed(int i, int spd)
{
    uint8_t reverse = 0;

    if (spd < 0)
    {
        spd     = -spd;
        reverse = 1;
    }
    if (spd > MOTOR_PWM_MAX)
        spd = MOTOR_PWM_MAX;

    if (i == LEFT)
    {
        if (reverse == 0)
        {
            setPWM(LEFT_MOTOR_FORWARD,  (uint16_t)spd);
            setPWM(LEFT_MOTOR_BACKWARD, 0);
        }
        else
        {
            setPWM(LEFT_MOTOR_BACKWARD, (uint16_t)spd);
            setPWM(LEFT_MOTOR_FORWARD,  0);
        }
    }
    else  /* RIGHT */
    {
        if (reverse == 0)
        {
            setPWM(RIGHT_MOTOR_FORWARD,  (uint16_t)spd);
            setPWM(RIGHT_MOTOR_BACKWARD, 0);
        }
        else
        {
            setPWM(RIGHT_MOTOR_BACKWARD, (uint16_t)spd);
            setPWM(RIGHT_MOTOR_FORWARD,  0);
        }
    }
}

/* ── setMotorSpeeds ──────────────────────────────────────────── */
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
    setMotorSpeed(LEFT,  leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
}