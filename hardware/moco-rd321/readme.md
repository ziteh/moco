# moco-RD321

- 6.7V~35V, 12A
- 3-Phase low-side current sensing. OP-Amp Gain=6.74, Shunt resistor=20mΩ
- Overcurrent detection. Ith=30A @ OC_COMPth=500mV
- Input voltage sensing
- 32x32mm square PCB, 4xM2 mounting holes
- [STSPIN32F0A](https://www.st.com/en/motor-drivers/stspin32f0a.html) 3-phase gate driver with embdded MCU SiP
  - STM32F031C6 48MHz, 4KB SRAM, 32KB Flash
  - 600mA sink/source gate drivers, integrated bootstrap diodes
  - 12V LDO and 3.3V buck
- [SiZ250DT](https://www.vishay.com/en/product/77227/) Dual N-MOSFET, 60V, 38A, 3.3x3.3mm
- MA702 magnetic encoder, SPI
- 2x04 2.54mm Pin header for debug, JST SM03B-SRSS-TB 1x03 for Serial and I2C

## Pinmap

| Description              | Analog IC    | STM32 | STM32 Func       |
| ------------------------ | ------------ | ----- | ---------------- |
| High-side U              | HS1          | PA8   | TIM1_CH1         |
| High-side V              | HS2          | PA9   | TIM1_CH2         |
| High-side W              | HS3          | PA10  | TIM1_CH3         |
| Low-side U               | LS1          | PB13  | TIM1_CH1N        |
| Low-side V               | LS2          | PB14  | TIM1_CH2N        |
| Low-side W               | LS3          | PB15  | TIM1_CH3N        |
| OC threshold select 1    | OC_TH_STBY1  | PF7   | Push-pull output |
| OC threshold select 2    | OC_TH_STBY2  | PF6   | Push-pull output |
| OC protection selection  | OC_SEL       | PA11  | Push-pull output |
| OC comparator output     | OC_COMP_INT  | PB12  | TIM1_BKIN        |
| OC comparator output     | OC_COMP_INT2 | PA12  | TIM1_ETR         |
| Current sense U          | OP3O         | PA0   | ADC_IN0          |
| Current sense V          | OP2O         | PA1   | ADC_IN1          |
| Current sense W          | OP1O         | PB1   | ADC_IN9          |
| VM sense                 | --           | PA4   | ADC_IN4          |
| SPI SCLK                 | --           | PA5   | SPI1             |
| SPI MISO                 | --           | PA6   | SPI1             |
| SPI MOSI                 | --           | PA7   | SPI1             |
| SPI nCS MA702            | --           | PA15  |                  |
| USART Tx                 | --           | PA2   | USART1           |
| USART Rx                 | --           | PA3   | USART1           |
| I2C SCL                  | --           | PB6   | I2C1             |
| I2C SDA                  | --           | PB7   | I2C1             |
| SWDIO                    | --           | PA13  |                  |
| SWCLK                    | --           | PA14  |                  |
| Connected to GND         | --           | BOOT0 |                  |
| User LED (Red)           | --           | PF0   | Active HIGH      |
| 12V indicator LED (Blue) | --           | --    |                  |

> - Unused GPIOs inside the package should be config to output low: PB0, PB2, PB3, PB4, PB5, PB8, PB9, PB10, PB11, PC13, PC14, PC15.
> - OC_TH_STBY1 and OC_TH_STBY2 should be HIGH for 500mV OC threshold.
> - For analog IC, the GPIO input config without pull-up or -down is always allowed.

## PCB

### Jumper

If input voltage (i.e. VM)<15V then close `SB1` to connect VM pin and VREG12 pin.

### Board Characteristics

- Copper layer count: 4
- Board overall dimensions: 32 x 32 mm
- Min track/spacing: 0.13 mm / 0.13 mm
- Min hole diameter: 0.3 mm
- Castellated pads: No
- Edge card connectors: No
- Plated board edge: No
- 1oz copper

## SPI

MA702 encoder SPI:
- Mode:
  - 0 (CPOL=0, CPHA=0) or 1 (CPOL=1, CPHA=1)
  - Data sampled on first edge
- 16-bit data frame
- MSB first
- nCS pin active low
- 25MHz clock

## MCU

| STSPIN32*x*                         | [F0](https://www.st.com/en/motor-drivers/stspin32f0.html) | [F0***A***](https://www.st.com/en/motor-drivers/stspin32f0a.html) | [F0***B***](https://www.st.com/en/motor-drivers/stspin32f0b.html) |
| ----------------------------------- | --------------------------------------------------------- | ----------------------------------------------------------------- | ----------------------------------------------------------------- |
| Operating voltage                   | 8~45V                                                     | 6.7~45V                                                           | 6.7~45V                                                           |
| OP-Amps                             | 4                                                         | 3                                                                 | 1                                                                 |
| GPIOs                               | 15                                                        | 16                                                                | 20                                                                |
| BOOT0 pin                           | --                                                        | Yes                                                               | Yes                                                               |
| 3FG decoding logic for Hall sensors | Yes                                                       | --                                                                | --                                                                |
| Comparator                          | OCP                                                       | OCP and current control                                           | OCP and current control                                           |
| Package                             | QFN 7x7mm 1EP                                             | QFN 7x7mm 1EP                                                     | QFN 7x7mm 1EP                                                     |

> [Solved: STSPIN32F0 vs STSPIN32F0A vs STSPIN32F0B - STMicroelectronics Community](https://community.st.com/t5/power-management/stspin32f0-vs-stspin32f0a-vs-stspin32f0b/td-p/273388)


## Reference

- [qwertpas/O32controller: Miniature closed-loop controller for brushless DC motors](https://github.com/qwertpas/O32controller)
- [STspin32G4 support - hardware support - SimpleFOC Community](https://community.simplefoc.com/t/stspin32g4-support/2027/6?page=3)
- ST AN5397 *"Current Sensing in motion control applications"*
- ST AN4999 *"STSPIN32F0/F0A/F0B overcurrent protection"*
- ST DB3364 *"STEVAL-SPIN3202: STSPIN32F0A advanced 3-phase BLDC driver with embedded STM32 MCU single shunt evaluation board"*
- ST DB4049 *"EVALKIT-ROBOT-1: Compact reference design kit for robotics and automation based on STSPIN32F0A"*
- ST DB3798 *"STEVAL-ESC002V1: Electronic Speed Controller reference design based on STSPIN32F0A"*
- TI SLUA887 *"Bootstrap Circuitry Selection for Half-Bridge Configurations"*
- Diodes DN1156 *"Gate Drivers in BLDC Motors"*
