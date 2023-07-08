
// #define SKIP_ALIGNMENT_POSITION
#define MODE_3PWM
// #define MODE_6PWM

#include "DRV8301.h"
#include <Arduino.h>
#include <SimpleFOC.h>

#define GPIO_INH_A (PA8)  /* Motor A phase PWM pin. */
#define GPIO_INH_B (PA9)  /* Motor B phase PWM pin. */
#define GPIO_INH_C (PA10) /* Motor C phase PWM pin. */
#define GPIO_INL_A (PC13) /* Motor A phase PWM pin. */
#define GPIO_INL_B (PA12) /* Motor B phase PWM pin. */
#define GPIO_INL_C (PB15) /* Motor C phase PWM pin. */

#define GPIO_EN_GATE (PB3) /* Motor enable pin. */
#define GPIO_FAULT (PB10)
#define GPIO_LED_1 (PB0)
#define GPIO_LED_2 (PB1)

#define GPIO_SPI_SCLK (PA5)
#define GPIO_SPI_MISO (PA6)
#define GPIO_SPI_MOSI (PA7)
#define GPIO_SPI_CS_DRIVER (PB13)
#define GPIO_SPI_CS_POS_SENSOR (PB14)

/* T-Motor U8 KV135. */
#define MOTOR_POLE_PAIRS (21)
#define MOTOR_PHASE_RESISTANCE (0.137) /* ohm. */
// #define MOTOR_KV (135)                 /* rpm/V. */
#define MOTOR_CURRENT_LIMIT (4)
#define MOTOR_VLOTAGE (24)

/* A2212. */
#define MOTOR_POLE_PAIRS (6)
// #define MOTOR_PHASE_RESISTANCE (0.137) /* ohm. */
// #define MOTOR_KV (135)                 /* rpm/V. */
#define MOTOR_CURRENT_LIMIT (3)
#define MOTOR_VLOTAGE (24)

// BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

#if defined(MODE_3PWM)
BLDCDriver3PWM driver = BLDCDriver3PWM(GPIO_INH_A, GPIO_INH_B, GPIO_INH_C);
  #define DRV8301_MODE (PWM_INPUT_MODE_3PWM)
#else
BLDCDriver6PWM driver = BLDCDriver6PWM(GPIO_INH_A, GPIO_INH_B, GPIO_INH_C, GPIO_INL_A, GPIO_INL_B, GPIO_INL_C);
  #define DRV8301_MODE (PWM_INPUT_MODE_6PWM)
#endif

DRV8301 gateDriver = DRV8301(GPIO_SPI_MOSI, GPIO_SPI_MISO, GPIO_SPI_SCLK, GPIO_SPI_CS_DRIVER, GPIO_EN_GATE, GPIO_FAULT);

/* ams AS5047P. */
MagneticSensorSPI positionSensor = MagneticSensorSPI(GPIO_SPI_CS_POS_SENSOR, 14, 0x3FFF);

Commander commander = Commander(Serial);
void onMotor(char *cmd) { commander.motor(&motor, cmd); }

void onReadAngle(char *)
{
  positionSensor.update();
  Serial.println(positionSensor.getAngle(), 3);
}

void onReadFault(char *)
{
  int code = gateDriver.read_fault();
  Serial.printf("%d\n", code);
}

void drv8302Setup(void);

void setup()
{
  // pinMode(GPIO_LED_1, OUTPUT);
  // digitalWrite(GPIO_LED_1, HIGH);
  // pinMode(GPIO_LED_2, OUTPUT);
  // digitalWrite(GPIO_LED_2, HIGH);

  /* Position sensor setup. */
  // positionSensor.spi_mode = SPI_MODE1; /* CPOL=0, CPHA=1. */
  // positionSensor.clock_speed = 1e6;    /* 10 MHz max. */
  // positionSensor.init();
  // motor.linkSensor(&positionSensor);

  gateDriver.begin(DRV8301_MODE);
  if (gateDriver.is_fault())
  {
    Serial.println("Error");
  }
  else
  {
    Serial.println("OK");
  }

  /* Driver setup. */
  driver.voltage_power_supply = MOTOR_VLOTAGE;
  driver.pwm_frequency = 12000;
  // driver.dead_zone = 0.005;
  driver.init();
  // driver.enable();
  motor.linkDriver(&driver);

  /* Power setup. */
  motor.voltage_limit = MOTOR_VLOTAGE;
  motor.current_limit = MOTOR_CURRENT_LIMIT;

  /* Controller setup. */
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  // motor.controller = MotionControlType::angle_openloop;
  motor.controller = MotionControlType::velocity_openloop;

  /* Velocity controller setup. */
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 15;
  // motor.PID_velocity.D = 0.001;
  // motor.PID_velocity.output_ramp = 200; /* Unit in volts/s. */
  motor.LPF_velocity.Tf = 0.01;
  // motor.velocity_limit = 10; /* Unit in rad/s. */

  /* Angle/Posision controller setup. */
  motor.P_angle.P = 5;
  motor.P_angle.output_ramp = 500; /* Acceleration control, unit in rad/s^2. */

  /* Communication setup. */
  // Serial.setTx(PA2);
  // Serial.setRx(PA3);
  Serial.begin(19200);
  motor.useMonitoring(Serial);

#ifdef SKIP_ALIGNMENT_POSITION
  motor.zero_electric_angle = 0;
  motor.sensor_direction = Direction::CCW;
#endif

  motor.init();
  motor.initFOC();

  /* Set target to present position. */
  // positionSensor.update();
  // motor.target = positionSensor.getAngle();


  // if (gateDriver.is_fault())
  // {
  //   digitalWrite(GPIO_LED_2, LOW);
  // }

  motor.target = 10;

  commander.add('M', onMotor, "motor");
  // commander.add('S', onReadAngle, "sensor");
  commander.add('F', onReadFault, "fault");

  _delay(1000);

}

void loop()
{
  motor.loopFOC(); /* Main FOC algorithm. */
  motor.move();    /* Motion control. */

  commander.run();
}

/**
 * @brief DRV8302 specific setup.
 */
void drv8302Setup(void)
{
  // pinMode(GPIO_DRV8302_M_PWM, OUTPUT);
  // digitalWrite(GPIO_DRV8302_M_PWM, HIGH); /* Enable 3PWM mode. */

  // pinMode(GPIO_DRV8302_M_OC, OUTPUT);
  // digitalWrite(GPIO_DRV8302_M_OC, LOW); /* Enable over current protection. */

  // pinMode(GPIO_DRV8302_OC_ADJ, OUTPUT);
  // digitalWrite(GPIO_DRV8302_OC_ADJ, HIGH); /* Set the maximum over current limit possible. */
}
