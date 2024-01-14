# moco-RD501

![](https://i.imgur.com/ffNdd2O.jpg)

STM32G431C8T6 MCU, DRV8301 gate driver, CSD18540Q5B power N-MOSFETs and AS5047x magnetic encoder.

Based on [ODrive](https://github.com/odriverobotics/ODriveHardware) `v3.5` PCB and [SimpleFOC](https://simplefoc.com/) firmware.

> Seems unstable.

| Front                                | Back                                 | Layers                               |
| ------------------------------------ | ------------------------------------ | ------------------------------------ |
| ![](https://i.imgur.com/jKlRSL9.png) | ![](https://i.imgur.com/IBP8MkX.png) | ![](https://i.imgur.com/w53ePka.jpg) |

### Pinmap

| Description            | STM32 Pin |
| ---------------------- | --------- |
| PWM-A High             | PA8       |
| PWM-B High             | PA9       |
| PWM-C High             | PA10      |
| PWM-A Low              | PC13      |
| PWM-B Low              | PA12      |
| PWM-C Low              | PB15      |
| EN_Gate (Enable)       | PB3       |
| nFAULT                 | PB10      |
| Encoder A              | PB5       |
| Encoder B              | PB6       |
| Encoder I              | PB4       |
| SPI SCLK               | PA5       |
| SPI MISO               | PA6       |
| SPI MOSI               | PA7       |
| SPI nCS DRV8301        | PB13      |
| SPI nCS AS5047x        | PB14      |
| SPI nCS CAN-Bus        | PB9       |
| CAN-Bus INT            | PA11      |
| USART Tx               | PA2       |
| USART Rx               | PA3       |
| I2C SCL                | PA15      |
| I2C SDA                | PB7       |
| Current sense 1 (CH-B) | PA0       |
| Current sense 2 (CH-C) | PA1       |
| Temperature sense      | PA4       |
| V_drive sense          | PB12      |
| User LED 1 (Red)       | PB0       |
| User LED 2 (Blue)      | PB1       |
| GPIO PB2               | PB2       |
| GPIO PB11              | PB11      |


### Board Characteristics

- Copper layer count: 4
- Board overall dimensions: 50 x 50 mm
- Min track/spacing: 0.2 mm / 0.2 mm
- Min hole diameter: 0.2 mm
- Castellated pads: No
- Edge card connectors: No
- Plated board edge: No

Support JLCPCB's PCBA service.
