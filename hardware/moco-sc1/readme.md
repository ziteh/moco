# moco-SC1

- 10V~35V, 12A
- 3-Phase low-side current sensing. Op-amp Gain=6.74, shunt resistor=20mΩ
- Input voltage sensing
- 40x40mm square PCB, 4xM3 mounting holes.
- [STSPIN32G4](https://www.st.com/en/motor-drivers/stspin32g4.html) 3-phase gate driver with embdded MCU SiP
  - [STM32G431VBx3](https://www.st.com/en/microcontrollers-microprocessors/stm32g431vb.html) 170MHz, 32KB SRAM, 128KB Flash
  - 1A sink/source gate drivers, integrated bootstrap diodes
  - 8~15V/200mA programmable buck for gate driver and 3.3V/150mA LDO
- [CSD88584Q5DC](https://www.ti.com/product/CSD88584Q5DC) Dual N-MOSFET half-bridge, 40V, 50A, 5x6mm
- [MA702](https://www.monolithicpower.com/en/ma702.html) magnetic encoder, SPI interface
- [TCAN33x](https://www.ti.com/product/TCAN330) 3.3V CAN transceiver
- 2x04 2.54mm Pin header for debug, JST GH 1.25mm 1x03 for CAN bus

## PCB

PCB online preview available [here](https://kicanvas.org/?github=https%3A%2F%2Fgithub.com%2Fziteh%2Fmoco%2Ftree%2Fmain%2Fhardware%2Fmoco-sc1), powered by [KiCanvas](https://github.com/theacodes/kicanvas).

### Jumper

`SB1`: Close to connect CAN bus 120Ω terminal resistor.

### Board Characteristics

- Copper layer count: 4
- Board overall dimensions: 40 x 40 mm
- Min track/spacing: 0.2 mm / 0.2 mm
- Min hole diameter: 0.3 mm
- Castellated pads: No
- Edge card connectors: No
- Plated board edge: No
- 1 or 2oz copper available

## SPI

MA702 encoder SPI:
- Mode:
  - 0 (CPOL=0, CPHA=0) or 3 (CPOL=1, CPHA=1) detected automatically
  - Data sampled on rising edge, transmission on falling
- 16-bit data frame
- MSB first
- nCS pin active low
- 25MHz clock max

## Pinmap

| Description                     | Gate Driver | STM32 | STM32 Func                |
| ------------------------------- | ----------- | ----- | ------------------------- |
| High-side A                     | INH1        | PE9   | TIM1_CH1                  |
| High-side B                     | INH2        | PE11  | TIM1_CH2                  |
| High-side C                     | INH3        | PE13  | TIM1_CH3                  |
| Low-side A                      | INL1        | PE8   | TIM1_CH1N                 |
| Low-side B                      | INL2        | PE10  | TIM1_CH2N                 |
| Low-side C                      | INL3        | PE12  | TIM1_CH3N                 |
| Active high for wakeup          | WAKE        | PE7   | Push-pull output          |
| Device ready signal             | READY       | PE14  | TIM1_BKIN2, pull-up input |
| Fault signal                    | NFAULT      | PE15  | TIM1_BKIN, pull-up input  |
|                                 | SCL         | PC8   | I2C3                      |
|                                 | SDA         | PC9   | I2C3                      |
| Enable short-circuit protection | SCREF       | --    |                           |
| Current sense Op-amp In+ A      | --          | PA1   | OPAMP1_VINP               |
| Current sense Op-amp In- A      | --          | PA3   | OPAMP1_VINM               |
| Current sense Op-amp Out A      | --          | PA2   | OPAMP1_VOUT, ADC1_IN3     |
| Current sense Op-amp In+ B      | --          | PA7   | OPAMP2_VINP               |
| Current sense Op-amp In- B      | --          | PC5   | OPAMP2_VINM               |
| Current sense Op-amp Out B      | --          | PA6   | OPAMP2_VOUT, ADC2_IN3     |
| Current sense Op-amp In+ C      | --          | PB0   | OPAMP3_VINP               |
| Current sense Op-amp In- C      | --          | PB2   | OPAMP3_VINM               |
| Current sense Op-amp Out C      | --          | PB1   | OPAMP3_VOUT, ADC1_IN12    |
| VBUS sense                      | --          | PA4   | ADC2_IN17                 |
| SPI SCLK                        | --          | PB3   | SPI1/3                    |
| SPI MISO                        | --          | PB4   | SPI1/3                    |
| SPI MOSI                        | --          | PB5   | SPI1/3                    |
| SPI nCS MA702                   | --          | PB6   |                           |
| USART TX                        | --          | PA9   | USART1                    |
| USART RX                        | --          | PA10  | USART1                    |
| CAN TX                          | --          | PA12  | FDCAN1                    |
| CAN RX                          | --          | PA11  | FDCAN1                    |
| SWDIO                           | --          | PA13  |                           |
| SWCLK                           | --          | PA14  |                           |
| SWO                             | --          | PA15  |                           |
| HSE OSC                         | --          | PF0   | OSC_IN                    |
| HSE OSC                         | --          | PF1   | OSC_OUT                   |
| Reset                           | --          | PG10  | NRST                      |
| BOOT, connected to GND          | --          | PB8   | BOOT0                     |
| User LED (Red), active high     | --          | PA0   | TIM2_CH1                  |
| VCC indicator LED (Blue)        | --          | --    |                           |

> - For gate driver internal connected GPIO, the input pin config without pull-up or -down is always allowed.
> - It is recommended to set PB8 (BOOT0) in a mode other than analog mode after startup to limit consumption.


## STSPIN32 SiP

| STSPIN32          | [F0][32f0]       | [F0A][32f0a]     | [F0B][32f0b]     | [G4][32g4]       |
| ----------------- | ---------------- | ---------------- | ---------------- | ---------------- |
| MCU               | STM32F031C6      | STM32F031C6      | STM32F031C6      | STM32G431VB      |
| Clock             | 48 MHz           | 48 MHz           | 48 MHz           | 170 MHz          |
| SRAM              | 4 KB             | 4 KB             | 4 KB             | 32 KB            |
| Flash             | 32 KB            | 32 KB            | 32 KB            | 128 KB           |
| Operating voltage | 8~45V            | 6.7~45V          | 6.7~45V          | 5.5~75V          |
| OP-Amps           | 4                | 3                | 1                | 3                |
| GPIOs             | 15               | 16               | 20               | 40               |
| CAN Bus           | 0                | 0                | 0                | 1 (CAN-FD)       |
| USB               | 0                | 0                | 0                | 1 (USB 2.0 FS)   |
| BOOT0 pin         | --               | Yes              | Yes              | Yes              |
| Package           | QFN-48 7x7mm 1EP | QFN-48 7x7mm 1EP | QFN-48 7x7mm 1EP | QFN-64 9x9mm 1EP |

> [Solved: STSPIN32F0 vs STSPIN32F0A vs STSPIN32F0B - STMicroelectronics Community](https://community.st.com/t5/power-management/stspin32f0-vs-stspin32f0a-vs-stspin32f0b/td-p/273388)

[32f0]: https://www.st.com/en/motor-drivers/stspin32f0.html
[32f0a]: https://www.st.com/en/motor-drivers/stspin32f0a.html
[32f0b]: https://www.st.com/en/motor-drivers/stspin32f0b.html
[32g4]: https://www.st.com/en/motor-drivers/stspin32g4.html

## Reference

- [runger1101001/simplefoc_funqi_example: Sample firmware for STSPIN32G4 based BLDC driver](https://github.com/runger1101001/simplefoc_funqi_example)
- [qwertpas/O32controller: Miniature closed-loop controller for brushless DC motors](https://github.com/qwertpas/O32controller)
- [Juanduino/STSPIN32G4-FunQi-Stack: SimpleFoc Format Dev. for potential 8pin PWM stepper use w. 4th half-bridge (also 6pin PWM BLDC use w. extra FET driver for eg. brake-resistor)](https://github.com/Juanduino/STSPIN32G4-FunQi-Stack)
  - [STspin32G4 support - hardware support - SimpleFOC Community](https://community.simplefoc.com/t/stspin32g4-support/2027/6?page=3)
- [EVSPIN32G4 - STSPIN32G4 demonstration board for three-phase brushless motors - STMicroelectronics](https://www.st.com/en/evaluation-tools/evspin32g4.html)
- [TIDA-01516 reference design | TI.com](https://www.ti.com/tool/TIDA-01516)
- [TIDA-00774 reference design | TI.com](https://www.ti.com/tool/TIDA-00774)
- ST AN5397 *"Current Sensing in motion control applications"*
- TI SLUA887 *"Bootstrap Circuitry Selection for Half-Bridge Configurations"*
- Diodes DN1156 *"Gate Drivers in BLDC Motors"*
