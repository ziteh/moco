
#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "stm32g4xx.h"
// #include "stm32g4xx_hal.h"
// #include "stm32g4xx_ll_adc.h"
// #include "stm32g4xx_ll_i2c.h"
// #include "stm32g4xx_ll_rcc.h"
// #include "stm32g4xx_ll_bus.h"
// #include "stm32g4xx_ll_crs.h"
// #include "stm32g4xx_ll_system.h"
// #include "stm32g4xx_ll_exti.h"
// #include "stm32g4xx_ll_cortex.h"
// #include "stm32g4xx_ll_utils.h"
// #include "stm32g4xx_ll_pwr.h"
// #include "stm32g4xx_ll_dma.h"
// #include "stm32g4xx_ll_spi.h"
// #include "stm32g4xx_ll_tim.h"
// #include "stm32g4xx_ll_usart.h"
// #include "stm32g4xx_ll_gpio.h"

#define SYSCLK_HZ (140000000U)
#define SYSCLK_MHZ (140U)

// STSPIN32G4 gate driver PWM
#define GD_PWM_TIM_INST (TIM1)
#define GD_PWM_TIM_PSC (1 - 1)    // Prescaler
#define GD_PWM_TIM_ARR (2332 - 1) // Auto reload
// #define GD_PWM_TIM_DTG (0x86)     // Dead-time generator, 0x86 = 1usec @ 140MHz
#define GD_PWM_TIM_DTG (0x80)
// #define GD_PWM_TIM_DTG (0x0)

// Main loop timer
#define ML_TIM_INST (TIM3)
#define ML_TIM_PSC (14 - 1)  // Prescaler 140
#define ML_TIM_ARR (500 - 1) // Auto reload
#define ML_TIM_HZ (20000U)
#define ML_TIM_MS (0.05F)

// Hearbeat LED
#define HB_LED_TIM_INST (TIM2)
#define HB_LED_TIM_PSC (2735 - 1) // Prescaler
#define HB_LED_TIM_ARR (256 - 1)  // Auto reload

// V_Bus voltage sense ADC
#define VBUS_ADC_INST (ADC2)

#endif /* PARAMETERS_H_ */
