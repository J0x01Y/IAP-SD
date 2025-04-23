/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
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
#define Fatal__Pin LL_GPIO_PIN_13
#define Fatal__GPIO_Port GPIOC
#define M1_A_Pin LL_GPIO_PIN_0
#define M1_A_GPIO_Port GPIOA
#define M1_B_Pin LL_GPIO_PIN_1
#define M1_B_GPIO_Port GPIOA
#define M6_PWM_Pin LL_GPIO_PIN_3
#define M6_PWM_GPIO_Port GPIOA
#define SPI3_CS__Pin LL_GPIO_PIN_4
#define SPI3_CS__GPIO_Port GPIOA
#define FAN_PWM_Pin LL_GPIO_PIN_1
#define FAN_PWM_GPIO_Port GPIOB
#define M5_PWM_Pin LL_GPIO_PIN_10
#define M5_PWM_GPIO_Port GPIOB
#define PWR_SD__Pin LL_GPIO_PIN_13
#define PWR_SD__GPIO_Port GPIOB
#define SD_LED_Pin LL_GPIO_PIN_14
#define SD_LED_GPIO_Port GPIOB
#define I2C_OE__Pin LL_GPIO_PIN_15
#define I2C_OE__GPIO_Port GPIOB
#define FRU_WP_Pin LL_GPIO_PIN_8
#define FRU_WP_GPIO_Port GPIOC
#define IRQ2__Pin LL_GPIO_PIN_11
#define IRQ2__GPIO_Port GPIOA
#define IRQ2__EXTI_IRQn EXTI15_10_IRQn
#define IRQ1__Pin LL_GPIO_PIN_12
#define IRQ1__GPIO_Port GPIOA
#define IRQ1__EXTI_IRQn EXTI15_10_IRQn
#define M3_PWM_Pin LL_GPIO_PIN_15
#define M3_PWM_GPIO_Port GPIOA
#define SPI3_SCK_Pin LL_GPIO_PIN_10
#define SPI3_SCK_GPIO_Port GPIOC
#define SPI3_MI_Pin LL_GPIO_PIN_11
#define SPI3_MI_GPIO_Port GPIOC
#define SPI3_MO_Pin LL_GPIO_PIN_12
#define SPI3_MO_GPIO_Port GPIOC
#define SD_DET__Pin LL_GPIO_PIN_2
#define SD_DET__GPIO_Port GPIOD
#define M4_PWM_Pin LL_GPIO_PIN_3
#define M4_PWM_GPIO_Port GPIOB
#define M2_A_Pin LL_GPIO_PIN_4
#define M2_A_GPIO_Port GPIOB
#define M2_B_Pin LL_GPIO_PIN_5
#define M2_B_GPIO_Port GPIOB
#define M1_PWM_Pin LL_GPIO_PIN_6
#define M1_PWM_GPIO_Port GPIOB
#define M2_PWM_Pin LL_GPIO_PIN_8
#define M2_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
