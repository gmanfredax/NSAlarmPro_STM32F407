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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_frontend.h"
#include "can_bus.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_FACTORY_Pin GPIO_PIN_13
#define BTN_FACTORY_GPIO_Port GPIOC
#define ZONE8_IN_Pin GPIO_PIN_0
#define ZONE8_IN_GPIO_Port GPIOC
#define ZONE9_IN_Pin GPIO_PIN_2
#define ZONE9_IN_GPIO_Port GPIOC
#define TAMPER_ANALOG_Pin GPIO_PIN_3
#define TAMPER_ANALOG_GPIO_Port GPIOC
#define ZONE1_IN_Pin GPIO_PIN_0
#define ZONE1_IN_GPIO_Port GPIOA
#define ZONE2_IN_Pin GPIO_PIN_3
#define ZONE2_IN_GPIO_Port GPIOA
#define ZONE3_IN_Pin GPIO_PIN_4
#define ZONE3_IN_GPIO_Port GPIOA
#define ZONE4_IN_Pin GPIO_PIN_5
#define ZONE4_IN_GPIO_Port GPIOA
#define ZONE5_IN_Pin GPIO_PIN_6
#define ZONE5_IN_GPIO_Port GPIOA
#define ZONE6_IN_Pin GPIO_PIN_0
#define ZONE6_IN_GPIO_Port GPIOB
#define ZONE7_IN_Pin GPIO_PIN_1
#define ZONE7_IN_GPIO_Port GPIOB
#define RELAY_SIREN_INT_Pin GPIO_PIN_8
#define RELAY_SIREN_INT_GPIO_Port GPIOE
#define RELAY_SIREN_EXT_Pin GPIO_PIN_9
#define RELAY_SIREN_EXT_GPIO_Port GPIOE
#define RELAY_NEBBIOGENO_Pin GPIO_PIN_10
#define RELAY_NEBBIOGENO_GPIO_Port GPIOE
#define RELAY_OUT1_Pin GPIO_PIN_11
#define RELAY_OUT1_GPIO_Port GPIOE
#define RELAY_OUT2_Pin GPIO_PIN_12
#define RELAY_OUT2_GPIO_Port GPIOE
#define LED_RGB_R_Pin GPIO_PIN_12
#define LED_RGB_R_GPIO_Port GPIOD
#define LED_RGB_G_Pin GPIO_PIN_13
#define LED_RGB_G_GPIO_Port GPIOD
#define LED_RGB_B_Pin GPIO_PIN_14
#define LED_RGB_B_GPIO_Port GPIOD
#define TAMPER_DIGITAL_Pin GPIO_PIN_7
#define TAMPER_DIGITAL_GPIO_Port GPIOC
#define LED_POWER_Pin GPIO_PIN_0
#define LED_POWER_GPIO_Port GPIOD
#define LED_ARMED_Pin GPIO_PIN_1
#define LED_ARMED_GPIO_Port GPIOD
#define LED_MAINT_Pin GPIO_PIN_2
#define LED_MAINT_GPIO_Port GPIOD
#define LED_ALARM_Pin GPIO_PIN_3
#define LED_ALARM_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
