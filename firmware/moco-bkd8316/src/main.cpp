/**
 * @file   main.cpp
 * @brief  SimpleFOC BLDC driver firmware for moco-BKD8316 (a DRV8316 breakout board).
 * @author ZiTe (honmonoh@gmail.com)
 * @note   Ref: https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/examples/drivers/drv8316
 */

#define SERIAL_UART_INSTANCE (2)

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "drivers/drv8316/drv8316.h"

/* Motor selection. */
// #define T_MOTOR_U8
// #define T_MOTOR_U10II
#define QM4208

// #define PWM_MODE_3 /* Comment out to use 6x PWM mode. */
#define OPENLOOP   /* Comment out to use close-loop control. */
// #define ANGLE_CONTROL /* Comment out to use velocity control. */

#define BAUDRATE (115200) /* Serial port baudrate. */
#define DEFAULT_TARGET (10)

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

/*
 * Pinmap.
 *
 * Select board/MCU by PlatformIO env (platformio.ini).
 * Arduino SPI:
 * - SCLK: D13.
 * - MISO: D12.
 * - MOSI: D11.
 */
#if defined(NUCLEO_F446RE) || defined(NUCLEO_F401RE)
  #define PWMH_A (PA8)  /* TIM1_CH1. */
  #define PWMH_B (PA9)  /* TIM1_CH2. */
  #define PWMH_C (PA10) /* TIM1_CH3. */

  #define PWML_A (PB13) /* TIM1_CH1N. */
  #define PWML_B (PB14) /* TIM1_CH2N. */
  #define PWML_C (PB15) /* TIM1_CH3N. */

  #define DRIVER_SPI_CS (PB6)
  #define DRIVER_FAULT (PB8)
#elif defined(NUCLEO_L432KC)
  #error
#else
  #error No Board Selected
#endif

/* Power. */
#define VOLTAGE_SUPPLY (6 * 3.7) /* Unit in V. */
#define CURRENT_LIMIT (1)      /* Unit in A. */

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

#ifdef PWM_MODE_3
DRV8316Driver3PWM driver = DRV8316Driver3PWM(PWMH_A, PWMH_B, PWMH_C, DRIVER_SPI_CS, false, NOT_SET, DRIVER_FAULT);
#else
DRV8316Driver6PWM driver = DRV8316Driver6PWM(PWMH_A, PWML_A, PWMH_B, PWML_B, PWMH_C, PWML_C, DRIVER_SPI_CS, false, NOT_SET, DRIVER_FAULT);
#endif

HardwareSerial Serial1(USART1); /* Change Serial instance from platformio.ini. */
Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }

void printDRV8316Status(void);
void setup()
{
  /* Communication setup. https://docs.simplefoc.com/commander_interface */
  Serial.begin(BAUDRATE);
  motor.useMonitoring(Serial);
  command.add('M', onMotor, "motor");

  // pinMode(PWMH_A, OUTPUT);
  // digitalWrite(PWMH_A, 1);
  // pinMode(PWMH_B, OUTPUT);
  // digitalWrite(PWMH_B, 1);
  // pinMode(PWMH_C, OUTPUT);
  // digitalWrite(PWMH_C, 1);

#ifndef OPENLOOP
  /* Configure angle/Position sensor. https://docs.simplefoc.com/magnetic_sensor_spi */
  angleSensor.spi_mode = SPI_MODE1; /* CPOL=0, CPHA=1. */
  angleSensor.clock_speed = 1e6;    /* 10 MHz max. */
  angleSensor.init();
  motor.linkSensor(&angleSensor);
#endif

  /* Configure driver. https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/examples/drivers/drv8316 */
  driver.voltage_power_supply = VOLTAGE_SUPPLY;
driver.pwm_frequency = 2e4;
#ifndef PWM_MODE_3
  // driver.dead_zone = 0.05;
#endif
  driver.init();
  motor.linkDriver(&driver);

  /* Configure motor parameters. https://docs.simplefoc.com/bldcmotor */
  // motor.phase_resistance = MOTOR_PHASE_RESISTANCE;
  // motor.KV_rating = MOTOR_KV * 1.5; /* SimpleFOC suggest to set the KV value provided to the library to 50-70% higher than the one given in the datasheet.. */
  motor.voltage_limit = VOLTAGE_SUPPLY * 0.17;
  motor.current_limit = CURRENT_LIMIT;
  // motor.motion_downsample = 5;

  /* Algorithms and controllers setup. */
  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
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

  motor.init(); /* Initialize motor. */
  // motor.initFOC(); /* Start FOC and aligh encoder. */

#ifndef OPENLOOP
  angleSensor.update();
  motor.target = angleSensor.getAngle(); /* Set the initial target value. */
#else
  motor.target = DEFAULT_TARGET;
#endif

  Serial.println(motor.target, 3);

  Serial.println("All Ready!");
  _delay(1000);
  driver.clearFault();
  printDRV8316Status();
  // motor.enable();
  // driver.setDriverOffEnabled(true);
  driver.setDriverOffEnabled(false);
  // driver.enable();
}

void loop()
{
  driver.clearFault();
  // motor.loopFOC(); /* Main FOC algorithm. */
  motor.move(DEFAULT_TARGET); /* Motion control. */

  driver.clearFault();
  command.run();
}

void printDRV8316Status(void)
{
  DRV8316Status status = driver.getStatus();
  Serial.println("DRV8316 Status:");
  Serial.print("Fault: ");
  Serial.println(status.isFault());
  Serial.print("Buck Error: ");
  Serial.print(status.isBuckError());
  Serial.print("  Undervoltage: ");
  Serial.print(status.isBuckUnderVoltage());
  Serial.print("  OverCurrent: ");
  Serial.println(status.isBuckOverCurrent());
  Serial.print("Charge Pump UnderVoltage: ");
  Serial.println(status.isChargePumpUnderVoltage());
  Serial.print("OTP Error: ");
  Serial.println(status.isOneTimeProgrammingError());
  Serial.print("OverCurrent: ");
  Serial.print(status.isOverCurrent());
  Serial.print("  Ah: ");
  Serial.print(status.isOverCurrent_Ah());
  Serial.print("  Al: ");
  Serial.print(status.isOverCurrent_Al());
  Serial.print("  Bh: ");
  Serial.print(status.isOverCurrent_Bh());
  Serial.print("  Bl: ");
  Serial.print(status.isOverCurrent_Bl());
  Serial.print("  Ch: ");
  Serial.print(status.isOverCurrent_Ch());
  Serial.print("  Cl: ");
  Serial.println(status.isOverCurrent_Cl());
  Serial.print("OverTemperature: ");
  Serial.print(status.isOverTemperature());
  Serial.print("  Shutdown: ");
  Serial.print(status.isOverTemperatureShutdown());
  Serial.print("  Warning: ");
  Serial.println(status.isOverTemperatureWarning());
  Serial.print("OverVoltage: ");
  Serial.println(status.isOverVoltage());
  Serial.print("PowerOnReset: ");
  Serial.println(status.isPowerOnReset());
  Serial.print("SPI Error: ");
  Serial.print(status.isSPIError());
  Serial.print("  Address: ");
  Serial.print(status.isSPIAddressError());
  Serial.print("  Clock: ");
  Serial.print(status.isSPIClockFramingError());
  Serial.print("  Parity: ");
  Serial.println(status.isSPIParityError());
  if (status.isFault())
    driver.clearFault();
  delayMicroseconds(1); // ensure 400ns delay
  DRV8316_PWMMode val = driver.getPWMMode();
  Serial.print("PWM Mode: ");
  Serial.println(val);
  delayMicroseconds(1); // ensure 400ns delay
  bool lock = driver.isRegistersLocked();
  Serial.print("Lock: ");
  Serial.println(lock);
}
