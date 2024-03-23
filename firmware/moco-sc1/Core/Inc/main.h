/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "parameters.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OPA1_P_Pin LL_GPIO_PIN_1
#define OPA1_P_GPIO_Port GPIOA
#define OPA1_O_Pin LL_GPIO_PIN_2
#define OPA1_O_GPIO_Port GPIOA
#define OPA1_M_Pin LL_GPIO_PIN_3
#define OPA1_M_GPIO_Port GPIOA
#define VBUS_SENSE_Pin LL_GPIO_PIN_4
#define VBUS_SENSE_GPIO_Port GPIOA
#define OPA2_O_Pin LL_GPIO_PIN_6
#define OPA2_O_GPIO_Port GPIOA
#define OPA2_P_Pin LL_GPIO_PIN_7
#define OPA2_P_GPIO_Port GPIOA
#define OPA2_M_Pin LL_GPIO_PIN_5
#define OPA2_M_GPIO_Port GPIOC
#define OPA3_P_Pin LL_GPIO_PIN_0
#define OPA3_P_GPIO_Port GPIOB
#define OPA3_O_Pin LL_GPIO_PIN_1
#define OPA3_O_GPIO_Port GPIOB
#define OPA3_M_Pin LL_GPIO_PIN_2
#define OPA3_M_GPIO_Port GPIOB
#define GD_WAKE_Pin LL_GPIO_PIN_7
#define GD_WAKE_GPIO_Port GPIOE
#define GD_INL1_Pin LL_GPIO_PIN_8
#define GD_INL1_GPIO_Port GPIOE
#define GD_INH1_Pin LL_GPIO_PIN_9
#define GD_INH1_GPIO_Port GPIOE
#define GD_INL2_Pin LL_GPIO_PIN_10
#define GD_INL2_GPIO_Port GPIOE
#define GD_INH2_Pin LL_GPIO_PIN_11
#define GD_INH2_GPIO_Port GPIOE
#define GD_INL3_Pin LL_GPIO_PIN_12
#define GD_INL3_GPIO_Port GPIOE
#define GD_INH3_Pin LL_GPIO_PIN_13
#define GD_INH3_GPIO_Port GPIOE
#define GD_READY_Pin LL_GPIO_PIN_14
#define GD_READY_GPIO_Port GPIOE
#define GD_SCL_Pin LL_GPIO_PIN_8
#define GD_SCL_GPIO_Port GPIOC
#define GD_SDA_Pin LL_GPIO_PIN_9
#define GD_SDA_GPIO_Port GPIOC
#define UART_TX_Pin LL_GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin LL_GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define CAN_RX_Pin LL_GPIO_PIN_11
#define CAN_RX_GPIO_Port GPIOA
#define CAN_TX_Pin LL_GPIO_PIN_12
#define CAN_TX_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define ENC_SPI_MISO_Pin LL_GPIO_PIN_4
#define ENC_SPI_MISO_GPIO_Port GPIOB
#define ENC_SPI_MOSI_Pin LL_GPIO_PIN_5
#define ENC_SPI_MOSI_GPIO_Port GPIOB
#define ENC_SPI_nCS_Pin LL_GPIO_PIN_6
#define ENC_SPI_nCS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
