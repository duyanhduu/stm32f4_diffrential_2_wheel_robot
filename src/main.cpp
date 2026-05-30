/*********************************************************************
 *  ROSArduinoBridge — STM32F401 port
 *
 *  Ported from the original Arduino version by Patrick Goebel /
 *  James Nugen.  Logic, command protocol, and PID structure are
 *  unchanged.  Only the hardware-layer calls have been updated for
 *  STM32F401 + stm32duino (PlatformIO).
 *
 *  What changed vs the original
 *  ─────────────────────────────
 *  1. ARDUINO_ENC_COUNTER  →  STM32_ENC_COUNTER
 *     The DDRD/PORTD/PCMSK/PCICR register block in setup() is
 *     replaced with a single initEncoders() call.  All ISR logic
 *     lives in encoder_driver.cpp (hardware quadrature timer mode).
 *
 *  2. Serial  →  ROS_SERIAL  (macro, resolves to Serial2)
 *     USART2 on PA2(TX)/PA3(RX) is the ROS bridge port.
 *     USART1 on PA9/PA10 is left untouched for ESP32.
 *     platformio.ini sets -DSERIAL_UART_INSTANCE=2 so that the
 *     Arduino "Serial" object itself maps to USART2; the ROS_SERIAL
 *     macro below makes this explicit and easy to change.
 *
 *  3. NULL  →  '\0'  in argv termination
 *     Suppresses -Wconversion warnings on ARM GCC.
 *
 *  Everything else — command parser, PID loop, auto-stop, sensor
 *  commands — is identical to the original.
 *********************************************************************/

#define USE_BASE      // Enable the base controller code
//#undef USE_BASE

#ifdef USE_BASE
  #define STM32_ENC_COUNTER   // hardware quadrature timer (TIM2/TIM4)
  #define L298_MOTOR_DRIVER
#endif

#undef USE_SERVOS             // servos not used

/* ── Serial port selection ───────────────────────────────────────
   ROS_SERIAL = the UART that talks to the host PC / ROS node.
   With -DSERIAL_UART_INSTANCE=2 in platformio.ini, stm32duino
   maps the Arduino "Serial" object to USART2 (PA2 TX, PA3 RX).
   We alias it here so the intent is explicit in the source.
   USART1 (PA9/PA10) is deliberately left alone for ESP32.        */
#define ROS_SERIAL  Serial    // resolved to USART2 via build flag

/* ── Baud rate & PWM ceiling ─────────────────────────────────── */
#define BAUDRATE   115200
#define MAX_PWM    255

/* ── Core includes ───────────────────────────────────────────── */
#include <Arduino.h>
#include "commands.h"
#include "sensors.h"

#ifdef USE_BASE
  #include "motor_driver.h"
  #include "encoder_driver.h"
  #include "diff_controller.h"

  /* PID loop timing */
  #define PID_RATE              30        // Hz
  const int PID_INTERVAL      = 1000 / PID_RATE;
  unsigned long nextPID       = PID_INTERVAL;

  /* Auto-stop if no movement command received within this window */
  #define AUTO_STOP_INTERVAL    2000      // ms
  long lastMotorCommand       = AUTO_STOP_INTERVAL;
#endif

/* ── Serial parser state ─────────────────────────────────────── */
int  arg   = 0;
int  idx   = 0;       // renamed from 'index' — avoids clash with
                      // <string.h> index() on some ARM toolchains
char chr;
char cmd;
char argv1[16];
char argv2[16];
long arg1;
long arg2;

/* ── resetCommand ────────────────────────────────────────────── */
void resetCommand()
{
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg  = 0;
  idx  = 0;
}

/* ── runCommand ──────────────────────────────────────────────── */
int runCommand()
{
  int   i = 0;
  char *p = argv1;
  char *str;
  int   pid_args[4];

  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd)
  {
  /* ── General GPIO / info commands ── */
  case GET_BAUDRATE:
    ROS_SERIAL.println(BAUDRATE);
    break;

  case ANALOG_READ:
    ROS_SERIAL.println(analogRead(arg1));
    break;

  case DIGITAL_READ:
    ROS_SERIAL.println(digitalRead(arg1));
    break;

  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    ROS_SERIAL.println("OK");
    break;

  case DIGITAL_WRITE:
    if      (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    ROS_SERIAL.println("OK");
    break;

  case PIN_MODE:
    if      (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    ROS_SERIAL.println("OK");
    break;

  case PING:
    ROS_SERIAL.println(Ping(arg1));
    break;

  /* ── Base controller commands ── */
#ifdef USE_BASE
  case READ_ENCODERS:
    ROS_SERIAL.print(readEncoder(LEFT));
    ROS_SERIAL.print(" ");
    ROS_SERIAL.println(readEncoder(RIGHT));
    break;

  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    ROS_SERIAL.println("OK");
    break;

  case MOTOR_SPEEDS:
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0)
    {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else
    {
      moving = 1;
    }
    leftPID.TargetTicksPerFrame  = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    ROS_SERIAL.println("OK");
    break;

  case MOTOR_RAW_PWM:
    lastMotorCommand = millis();
    resetPID();
    moving = 0;           // temporarily bypass PID
    setMotorSpeeds(arg1, arg2);
    ROS_SERIAL.println("OK");
    break;

  case UPDATE_PID:
    /* Format: "u Kp:Kd:Ki:Ko"  e.g. "u 20:12:0:50" */
    while ((str = strtok_r(p, ":", &p)) != NULL)
    {
      pid_args[i] = atoi(str);
      i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    ROS_SERIAL.println("OK");
    break;
#endif /* USE_BASE */

  default:
    ROS_SERIAL.println("Invalid Command");
    break;
  }

  return 0;
}

/* ── setup ───────────────────────────────────────────────────── */
void setup()
{
  /* Open the ROS bridge serial port (USART2 via build flag) */
  ROS_SERIAL.begin(BAUDRATE);

#ifdef USE_BASE
  /* STM32: hardware quadrature timers replace the AVR
     DDRD/PCMSK/PCICR register block from the original file.    */
#ifdef STM32_ENC_COUNTER
  initEncoders();           // TIM2 (left, 32-bit) + TIM4 (right, 16-bit)
#endif

  initMotorController();    // TIM3 PWM + enable GPIO
  resetPID();
#endif /* USE_BASE */
}

/* ── loop ────────────────────────────────────────────────────── */
void loop()
{
  /* ── Serial command parser ── */
  while (ROS_SERIAL.available() > 0)
  {
    chr = ROS_SERIAL.read();

    if (chr == '\r')          // CR terminates a command
    {
      if (arg == 1) argv1[idx] = '\0';
      else if (arg == 2) argv2[idx] = '\0';
      runCommand();
      resetCommand();
    }
    else if (chr == ' ')      // space delimits command / arguments
    {
      if (arg == 0)
      {
        arg = 1;
      }
      else if (arg == 1)
      {
        argv1[idx] = '\0';
        arg = 2;
        idx = 0;
      }
      continue;
    }
    else
    {
      if (arg == 0)
      {
        cmd = chr;            // first token: single-letter command
      }
      else if (arg == 1)
      {
        argv1[idx++] = chr;
      }
      else if (arg == 2)
      {
        argv2[idx++] = chr;
      }
    }
  }

  /* ── PID update at fixed rate ── */
#ifdef USE_BASE
  if (millis() > nextPID)
  {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  /* ── Auto-stop watchdog ── */
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif /* USE_BASE */
}