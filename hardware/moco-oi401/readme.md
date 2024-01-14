# moco-OI401

- 10V~35V operating voltage
- 8A peak current
- 3x or 6xPWM control
- 3-Phase low-side current sense
- Ø40mm round PCB
- 3xM2 mounting holes, P.C.D Ø35.5mm
- [STM32G431CBU6](https://www.st.com/en/microcontrollers-microprocessors/stm32g431cb.html) 170 MHz MCU
- [DRV8316C](https://www.ti.com/product/DRV8316C) gate driver integrated FETs
- AS5047x magnetic encoder

## Pinmap

| Description              | DRV8316 | STM32 | STM32 Func |
| ------------------------ | ------- | ----- | ---------- |
| PWM-A High               | INHA    | PA10  | TIM1_CH3   |
| PWM-B High               | INHB    | PA9   | TIM1_CH2   |
| PWM-C High               | INHC    | PA8   | TIM1_CH1   |
| PWM-A Low                | INLA    | PC15  | TIM1_CH3N  |
| PWM-B Low                | INLB    | PA12  | TIM1_CH2N  |
| PWM-C Low                | INLC    | PB13  | TIM1_CH1N  |
| Current sense A          | SOA     | PA5   | ADC2_IN13  |
| Current sense B          | SOB     | PA6   | ADC2_IN3   |
| Current sense C          | SOC     | PA7   | ADC2_IN4   |
| nEnable                  | DRVOFF  | PB12  |            |
| nFAULT                   | nFAULT  | PB11  |            |
| SPI SCLK                 | SCLK    | PB3   | SPI1       |
| SPI MISO                 | SDI     | PB4   | SPI1       |
| SPI MOSI                 | SDO     | PB5   | SPI1       |
| SPI nCS DRV8316          | nSCS    | PB9   |            |
| SPI nCS AS5047x          | --      | PB2   |            |
| USART Tx                 | --      | PA2   | USART2     |
| USART Rx                 | --      | PA3   | USART2     |
| User LED 1 (Red)         | --      | PA0   | Active LOW |
| User LED 2 (Blue)        | --      | PA1   | Active LOW |
| 5V indicator LED (Green) | --      | --    |            |

## Jumper
- SB1 (3.3V LDO Bypass)
  - ON: 3.3V LDO bypassed, 3.3V provided through the buck regulator of DRV8316.
  - OFF (Default): 3.3V provided through the LDO.
- SB2 (nSLEEP Reset)
  - ON: Connected DRV8316 `nSLEEP` pin to RESET button.
  - OFF (Default): Disconnected DRV8316 `nSLEEP` pin to RESET button.
- SB3 (Voltage Reference Selection)
  - ON (Default): Connected +3.3V to STM32 `VREF+` and DRV8316 `VREF/LILM`, `VREF+` used as input.
  - OFF: Disconnected +3.3V to STM32 `VREF+` and DRV8316 `VREF/LILM`, voltage reference powered by STM32G4 embed voltage reference buffer, `VREF+` used as output.

## SPI

- Mode=1
  - CPOL=0. Clock is low when idle.
  - CPHA=1. Data sampled on 2nd edge (i.e. falling edge).
- 16-bit data frame.
- MSB first.
- nCS pin active low.
- 5MHz clock (limited by DRV8316).

## Board Characteristics

- Copper layer count: 2
- Board overall dimensions: 40 x 40 mm
- Min track/spacing: 0.13 mm / 0.13 mm
- Min hole diameter: 0.3 mm
- Castellated pads: No
- Edge card connectors: No
- Plated board edge: No
