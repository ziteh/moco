/**
 * @file   main.cpp
 * @brief  SimpleFOC BLDC driver firmware for moco-SC1 (STSPIN32G4 / STM32G4341VBx3).
 * @author ZiTe (honmonoh@gmail.com)
 */

#define SERIAL_UART_INSTANCE (1)

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include "stspin32g4.h"

/* Motor selection. */
// #define T_MOTOR_U8
// #define T_MOTOR_U10II
#define QM4208

// #define PWM_MODE_3 /* Comment out to use 6x PWM mode. */
#define OPENLOOP /* Comment out to use close-loop control. */
// #define ANGLE_CONTROL /* Comment out to use velocity control. */

/* Power. */
#define VOLTAGE_SUPPLY (6 * 3.7) /* Unit in V. */
#define CURRENT_LIMIT (3)        /* Unit in A. */

#define BAUDRATE (115200) /* Serial port baudrate. */
#define DEFAULT_TARGET (6)

/* Pinmap. Select board/MCU by PlatformIO env (platformio.ini). */
#if defined(NUCLEO_G431RB)
  #define PWML_A (PE_8)  /* TIM1_CH1N. */
  #define PWMH_A (PE_9)  /* TIM1_CH1. */
  #define PWML_B (PE_10) /* TIM1_CH2N. */
  #define PWMH_B (PE_11) /* TIM1_CH2. */
  #define PWML_C (PE_12) /* TIM1_CH3N. */
  #define PWMH_C (PE_13) /* TIM1_CH3. */

  #define DRIVER_I2C_SCL (PC8) /* I2C3. */
  #define DRIVER_I2C_SDA (PC9) /* I2C3. */

  #define DRIVER_FAULT (PE_15) /* TIM1_BKIN. */
  #define DRIVER_READY (PE_14) /* TIM1_BKIN2. */
  #define DRIVER_WAKE (PE_7)

  #define OPA_INP_A (PA1) /* OPAMP1_VINP. */
  #define OPA_INM_A (PA3) /* OPAMP1_VINM. */
  #define OPA_OUT_A (PA2) /* OPAMP1_VOUT, ADC1_IN3. */

  #define OPA_INP_B (PA7) /* OPAMP2_VINP. */
  #define OPA_INM_B (PC5) /* OPAMP2_VINM. */
  #define OPA_OUT_B (PA6) /* OPAMP2_VOUT, ADC2_IN3. */

  #define OPA_INP_C (PB0) /* OPAMP3_VINP. */
  #define OPA_INM_C (PB2) /* OPAMP3_VINM. */
  #define OPA_OUT_C (PB1) /* OPAMP3_VOUT, ADC1_IN12. */

  #define SENSOR_SPI_SCLK (PB3) /* SPI1/3. */
  #define SENSOR_SPI_MISO (PB4) /* SPI1/3. */
  #define SENSOR_SPI_MOSI (PB5) /* SPI1/3. */
  #define SENSOR_SPI_CS (PB6)

  #define UART_TX (PA9)  /* USART1. */
  #define UART_RX (PA10) /* USART1. */

  #define CAN_TX (PA12) /* FDCAN1. */
  #define CAN_RX (PA11) /* FDCAN1. */

  #define USER_LED (PA0)   /* TIM2_CH1. */
  #define VBUS_SENSE (PA4) /* ADC2_IN17. */
#elif defined(MOCO_SC1)
  #error
#else
  #error No Board Selected
#endif

/* Motor parameters. */
#if defined(T_MOTOR_U8)
  #define MOTOR_POLE_PAIRS (21)
  #define MOTOR_PHASE_RESISTANCE (NOT_SET) /* Unit in ohm. */
  #define MOTOR_KV (135)                   /* Unit in rpm/V. */
#elif defined(T_MOTOR_U10II)
  #define MOTOR_POLE_PAIRS (21)
  #define MOTOR_PHASE_RESISTANCE (NOT_SET) /* Unit in ohm. */
  #define MOTOR_KV (100)                   /* Unit in rpm/V. */
#elif defined(QM4208)
  #define MOTOR_POLE_PAIRS (7)
  #define MOTOR_PHASE_RESISTANCE (NOT_SET) /* Unit in ohm. */
  #define MOTOR_KV (380)                   /* Unit in rpm/V. */
#else
  #error No Motor Selected
#endif

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

#ifdef PWM_MODE_3
BLDCDriver3PWM driver = BLDCDriver3PWM(PWMH_A, PWMH_B, PWMH_C);
#else
BLDCDriver6PWM driver = BLDCDriver6PWM(PWMH_A, PWML_A, PWMH_B, PWML_B, PWMH_C, PWML_C);
#endif

// HardwareSerial Serial1(USART1); /* Change Serial instance from platformio.ini. */
HardwareSerial Serial1(UART_RX, UART_TX);
Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }

void setup()
{
  /* Communication setup. https://docs.simplefoc.com/commander_interface */
  Serial.begin(BAUDRATE);
  motor.useMonitoring(Serial);
  command.add('M', onMotor, "motor");

#ifndef OPENLOOP
  /* Configure angle/Position sensor. https://docs.simplefoc.com/magnetic_sensor_spi */
  angleSensor.spi_mode = SPI_MODE1; /* CPOL=0, CPHA=1. */
  angleSensor.clock_speed = 1e6;    /* 10 MHz max. */
  angleSensor.init();
  motor.linkSensor(&angleSensor);
#endif

  /* Configure driver. */
  driver.voltage_power_supply = VOLTAGE_SUPPLY;
// driver.pwm_frequency = 2e4;
#ifndef PWM_MODE_3
  // driver.dead_zone = 0.05;
#endif
  driver.init();
  motor.linkDriver(&driver);

  /* Configure motor parameters. https://docs.simplefoc.com/bldcmotor */
  // motor.phase_resistance = MOTOR_PHASE_RESISTANCE;
  // motor.KV_rating = MOTOR_KV * 1.5; /* SimpleFOC suggest to set the KV value provided to the library to 50-70% higher than the one given in the datasheet.. */
#ifdef OPENLOOP
  motor.voltage_limit = VOLTAGE_SUPPLY * 0.17;
#else
  motor.voltage_limit = VOLTAGE_SUPPLY;
#endif
  motor.current_limit = CURRENT_LIMIT;
  // motor.motion_downsample = 5;

  /* Algorithms and controllers setup. */
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // motor.torque_controller = TorqueControlType::voltage;
#if defined(OPENLOOP) && defined(ANGLE_CONTROL)
  motor.controller = MotionControlType::angle_openloop;
#elif defined(OPENLOOP) && !defined(ANGLE_CONTROL)
  motor.controller = MotionControlType::velocity_openloop;
#elif !defined(OPENLOOP) && defined(ANGLE_CONTROL)
  motor.controller = MotionControlType::angle;
#elif !defined(OPENLOOP) && !defined(ANGLE_CONTROL)
  motor.controller = MotionControlType::velocity;
#endif

  /* Velocity control loop setup. */
  // motor.PID_velocity.P = 0.2;
  // motor.PID_velocity.I = 20;
  // motor.PID_velocity.D = 0.001;
  // motor.PID_velocity.output_ramp = 500; /* Unit in volts/s. */
  motor.LPF_velocity.Tf = 0.01;
  // motor.velocity_limit = 15; /* Unit in rad/s. */

  /* Angle/Position control loop setup. */
  // motor.P_angle.P = 5;
  // motor.P_angle.I = 0.5;
  // motor.P_angle.D = 0.0;
  // motor.P_angle.output_ramp = 500; /* Acceleration limit(?), unit in rad/s^2. */

  motor.init();    /* Initialize motor. */
  motor.initFOC(); /* Start FOC and aligh encoder. */

#ifndef OPENLOOP
  angleSensor.update();
  motor.target = angleSensor.getAngle(); /* Set the initial target value. */
#else
  motor.target = DEFAULT_TARGET;
#endif

  Serial.print("All Ready! ");
  Serial.println(motor.target, 3);
  _delay(1000);
}

void loop()
{
  motor.loopFOC(); /* Main FOC algorithm. */
  motor.move();    /* Motion control. */

  command.run();
}
